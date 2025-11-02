/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body with IMU, Encoders, and PID control
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <inttypes.h>
#include <stdbool.h>
#include <math.h>
#include "pca9685.h"
#include "bno055_stm32.h"
#include "arm_math.h" 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// PID Structure with DSP filtering
typedef struct {
    float kp;           // Proportional gain
    float ki;           // Integral gain
    float kd;           // Derivative gain
    float prev_error;   // Previous error
    float integral;     // Integral term
    float max_output;   // Maximum output limit
    float min_output;   // Minimum output limit
    
    // DSP Filter components
    arm_biquad_casd_df1_inst_f32 lpf_error;      // Low-pass filter for error
    arm_biquad_casd_df1_inst_f32 lpf_derivative; // Low-pass filter for derivative
    float lpf_error_state[4];                     // Filter state for error (2 stages)
    float lpf_derivative_state[4];                // Filter state for derivative
    float lpf_error_coeffs[5];                    // Biquad coefficients for error filter
    float lpf_derivative_coeffs[5];               // Biquad coefficients for derivative filter
    
    // Additional filtering variables
    float filtered_error;
    float filtered_derivative;
    float prev_measurement;  // For derivative calculation
} PID_Controller;

// Motor structure with encoder
typedef struct {
    int32_t encoder_count;
    int32_t prev_encoder_count;
    float velocity;
    float target_velocity;
    PID_Controller pid;
    
    // Moving average filter for velocity smoothing
    arm_fir_instance_f32 velocity_fir;
    float velocity_fir_state[16];  // FIR filter state (order + blockSize - 1)
    float velocity_fir_coeffs[8];  // FIR filter coefficients (8-tap moving average)
} Motor;

// Robot position structure
typedef struct {
    float x;
    float y;
    float theta;
    float velocity_x;
    float velocity_y;
    float angular_velocity;
} RobotPose;

// Communication buffer
uint8_t rxbuff[32];

// Control parameters
int slow = 75;
int fast = 400;
int Buff1 = 20;
int Buff2 = -20;
int BuffP = 50;
int BuffN = -50;

// Controller inputs
int32_t lx, ly, rx, ry, cro, squ, tri, cir, up, down, left, right, ll1, rr1, ll2, rr2;

// State variables
bool motorState = false;
bool prev_up = false;
bool fwState = false;
bool prev_cro = false;
bool prev_squ = false;
bool prev_down = false;
bool prev_l1 = false;
bool prev_l2 = false;
bool prev_r1 = false;

// Motor instances (assuming 4-wheel drive)
Motor motor_fl, motor_fr, motor_bl, motor_br; // front-left, front-right, back-left, back-right

// Robot pose
RobotPose robot_pose;

// IMU data
bno055_vector_t imu_euler;
bno055_vector_t imu_gyro;
bno055_vector_t imu_accel;

// Timing variables
uint32_t prev_time_ms = 0;
float dt = 0.02f; // 20ms default

// Robot physical parameters (adjust according to your robot)
#define WHEEL_RADIUS 0.05f      // 5cm wheel radius
#define WHEEL_BASE 0.3f         // 30cm distance between front and rear wheels
#define TRACK_WIDTH 0.25f       // 25cm distance between left and right wheels
#define ENCODER_PPR 1000        // Pulses per revolution for encoders

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;  // Added for encoder
TIM_HandleTypeDef htim5;  // Added for encoder
TIM_HandleTypeDef htim8;  // Added for encoder
TIM_HandleTypeDef htim9;  // Added for encoder
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim13;
UART_HandleTypeDef huart4;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId encoderTaskHandle;
osThreadId imuTaskHandle;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM10_Init(void);
static void MX_I2C2_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartEncoderTask(void const * argument);
void StartIMUTask(void const * argument);

/* USER CODE BEGIN PFP */
// Function prototypes
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float max_out, float min_out);
float PID_Calculate(PID_Controller *pid, float setpoint, float measured_value, float dt);
void Motor_Init(Motor *motor, float kp, float ki, float kd);
void Update_Motor_Velocity(Motor *motor, float dt);
void Update_Robot_Pose(float dt);
void Control_Base_Motors(int32_t lx, int32_t ly, int32_t rx);
void Read_Encoders(void);
void PWM_Set_Duty(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t duty);
long map(long x, long in_min, long in_max, long out_min, long out_max);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Calculate biquad low-pass filter coefficients (2nd order Butterworth)
void Calculate_LPF_Coefficients(float *coeffs, float sample_freq, float cutoff_freq) {
    float omega = 2.0f * M_PI * cutoff_freq / sample_freq;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * 0.707f); // Q = 0.707 for Butterworth
    
    float b0 = (1.0f - cs) / 2.0f;
    float b1 = 1.0f - cs;
    float b2 = (1.0f - cs) / 2.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cs;
    float a2 = 1.0f - alpha;
    
    // Normalize coefficients
    coeffs[0] = b0 / a0;
    coeffs[1] = b1 / a0;
    coeffs[2] = b2 / a0;
    coeffs[3] = a1 / a0;
    coeffs[4] = a2 / a0;
}

// Initialize PID controller with DSP filters
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float max_out, float min_out) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->max_output = max_out;
    pid->min_output = min_out;
    pid->filtered_error = 0.0f;
    pid->filtered_derivative = 0.0f;
    pid->prev_measurement = 0.0f;
    
    // Initialize filter states to zero
    memset(pid->lpf_error_state, 0, sizeof(pid->lpf_error_state));
    memset(pid->lpf_derivative_state, 0, sizeof(pid->lpf_derivative_state));
    
    // Calculate low-pass filter coefficients
    // Error filter: 10Hz cutoff at 50Hz sample rate
    Calculate_LPF_Coefficients(pid->lpf_error_coeffs, 50.0f, 10.0f);
    
    // Derivative filter: 5Hz cutoff at 50Hz sample rate (more aggressive filtering)
    Calculate_LPF_Coefficients(pid->lpf_derivative_coeffs, 50.0f, 5.0f);
    
    // Initialize biquad cascade filters (1 stage each)
    arm_biquad_cascade_df1_init_f32(&pid->lpf_error, 1, 
                                    pid->lpf_error_coeffs, 
                                    pid->lpf_error_state);
    
    arm_biquad_cascade_df1_init_f32(&pid->lpf_derivative, 1, 
                                    pid->lpf_derivative_coeffs, 
                                    pid->lpf_derivative_state);
}

// Calculate PID output with DSP filtering
float PID_Calculate(PID_Controller *pid, float setpoint, float measured_value, float dt) {
    float error = setpoint - measured_value;
    
    // Apply low-pass filter to error signal
    arm_biquad_cascade_df1_f32(&pid->lpf_error, &error, &pid->filtered_error, 1);
    
    // Proportional term (using filtered error)
    float proportional = pid->kp * pid->filtered_error;
    
    // Integral term (using filtered error)
    pid->integral += pid->filtered_error * dt;
    
    // Anti-windup: Clamp integral
    float integral_limit = 50.0f;
    if (pid->integral > integral_limit) pid->integral = integral_limit;
    if (pid->integral < -integral_limit) pid->integral = -integral_limit;
    
    float integral = pid->ki * pid->integral;
    
    // Derivative term (derivative on measurement to avoid derivative kick)
    float raw_derivative = -(measured_value - pid->prev_measurement) / dt;
    
    // Apply low-pass filter to derivative
    arm_biquad_cascade_df1_f32(&pid->lpf_derivative, &raw_derivative, 
                               &pid->filtered_derivative, 1);
    
    float derivative = pid->kd * pid->filtered_derivative;
    
    // Calculate output
    float output = proportional + integral + derivative;
    
    // Clamp output
    if (output > pid->max_output) output = pid->max_output;
    if (output < pid->min_output) output = pid->min_output;
    
    // Advanced anti-windup: back-calculate integral if saturated
    if (output == pid->max_output || output == pid->min_output) {
        pid->integral -= pid->filtered_error * dt;
    }
    
    // Update previous values
    pid->prev_error = pid->filtered_error;
    pid->prev_measurement = measured_value;
    
    return output;
}

// Initialize motor with PID and velocity filter
void Motor_Init(Motor *motor, float kp, float ki, float kd) {
    motor->encoder_count = 0;
    motor->prev_encoder_count = 0;
    motor->velocity = 0.0f;
    motor->target_velocity = 0.0f;
    
    // Initialize PID with DSP filters
    PID_Init(&motor->pid, kp, ki, kd, 100.0f, -100.0f);
    
    // Initialize FIR filter state
    memset(motor->velocity_fir_state, 0, sizeof(motor->velocity_fir_state));
    
    // 8-tap moving average filter coefficients (simple averaging)
    for (int i = 0; i < 8; i++) {
        motor->velocity_fir_coeffs[i] = 1.0f / 8.0f;
    }
    
    // Initialize FIR filter instance
    arm_fir_init_f32(&motor->velocity_fir, 8, 
                     motor->velocity_fir_coeffs, 
                     motor->velocity_fir_state, 1);
}

// Update motor velocity from encoder with FIR filtering
void Update_Motor_Velocity(Motor *motor, float dt) {
    int32_t delta_count = motor->encoder_count - motor->prev_encoder_count;
    
    // Calculate raw velocity
    float raw_velocity = (float)delta_count / (ENCODER_PPR * dt) * 2.0f * M_PI * WHEEL_RADIUS;
    
    // Apply FIR moving average filter to velocity
    arm_fir_f32(&motor->velocity_fir, &raw_velocity, &motor->velocity, 1);
    
    motor->prev_encoder_count = motor->encoder_count;
}

// Additional utility: Apply median filter for spike rejection (optional)
float Median_Filter_3(float a, float b, float c) {
    // Simple 3-sample median filter using ARM DSP sort
    float samples[3] = {a, b, c};
    
    // Simple sorting for 3 samples
    if (samples[0] > samples[1]) {
        float temp = samples[0];
        samples[0] = samples[1];
        samples[1] = temp;
    }
    if (samples[1] > samples[2]) {
        float temp = samples[1];
        samples[1] = samples[2];
        samples[2] = temp;
    }
    if (samples[0] > samples[1]) {
        float temp = samples[0];
        samples[0] = samples[1];
        samples[1] = temp;
    }
    
    return samples[1]; // Return median
}

// Enhanced velocity calculation with outlier rejection
void Update_Motor_Velocity_Advanced(Motor *motor, float dt) {
    static float velocity_history[4][3] = {0}; // History for each motor
    static int motor_idx = 0; // Track which motor (0-3)
    
    int32_t delta_count = motor->encoder_count - motor->prev_encoder_count;
    
    // Calculate raw velocity
    float raw_velocity = (float)delta_count / (ENCODER_PPR * dt) * 2.0f * M_PI * WHEEL_RADIUS;
    
    // Store in history and apply median filter
    velocity_history[motor_idx][0] = velocity_history[motor_idx][1];
    velocity_history[motor_idx][1] = velocity_history[motor_idx][2];
    velocity_history[motor_idx][2] = raw_velocity;
    
    float median_filtered = Median_Filter_3(velocity_history[motor_idx][0],
                                           velocity_history[motor_idx][1],
                                           velocity_history[motor_idx][2]);
    
    // Apply FIR moving average filter after median filter
    arm_fir_f32(&motor->velocity_fir, &median_filtered, &motor->velocity, 1);
    
    motor->prev_encoder_count = motor->encoder_count;
    motor_idx = (motor_idx + 1) % 4; // Cycle through motors
}


// Read all encoders
void Read_Encoders(void) {
    motor_fl.encoder_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
    motor_fr.encoder_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
    motor_bl.encoder_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim8);
    motor_br.encoder_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim9);
}

// Set PWM duty cycle
void PWM_Set_Duty(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t duty) {
    switch(channel) {
        case TIM_CHANNEL_1:
            htim->Instance->CCR1 = duty;
            break;
        case TIM_CHANNEL_2:
            htim->Instance->CCR2 = duty;
            break;
        case TIM_CHANNEL_3:
            htim->Instance->CCR3 = duty;
            break;
        case TIM_CHANNEL_4:
            htim->Instance->CCR4 = duty;
            break;
    }
}

// Update robot pose using odometry
void Update_Robot_Pose(float dt) {
    // Calculate wheel velocities
    float v_fl = motor_fl.velocity;
    float v_fr = motor_fr.velocity;
    float v_bl = motor_bl.velocity;
    float v_br = motor_br.velocity;

    // For mecanum wheels or differential drive
    // This is a simplified model - adjust based on your robot configuration
    float v_x = (v_fl + v_fr + v_bl + v_br) / 4.0f;
    float v_y = (-v_fl + v_fr + v_bl - v_br) / 4.0f; // For mecanum wheels
    float omega = (-v_fl + v_fr - v_bl + v_br) / (4.0f * (WHEEL_BASE + TRACK_WIDTH) / 2.0f);

    // Use IMU for more accurate heading
    robot_pose.theta = imu_euler.z * M_PI / 180.0f; // Convert to radians
    robot_pose.angular_velocity = imu_gyro.z * M_PI / 180.0f;

    // Update position
    robot_pose.velocity_x = v_x * cos(robot_pose.theta) - v_y * sin(robot_pose.theta);
    robot_pose.velocity_y = v_x * sin(robot_pose.theta) + v_y * cos(robot_pose.theta);

    robot_pose.x += robot_pose.velocity_x * dt;
    robot_pose.y += robot_pose.velocity_y * dt;
}

// Enhanced motor control with PID
void Control_Base_Motors(int32_t lx, int32_t ly, int32_t rx) {
    // Calculate desired velocities for mecanum drive
    float max_vel = 2.0f; // m/s maximum velocity

    // Normalize joystick inputs (-127 to 127) to velocity commands
    float vx = (float)map(ly, -127, 127, (long)(-max_vel * 1000), (long)(max_vel * 1000)) / 1000.0f;
    float vy = (float)map(lx, -127, 127, (long)(-max_vel * 1000), (long)(max_vel * 1000)) / 1000.0f;
    float omega = (float)map(rx, -127, 127, -2000, 2000) / 1000.0f; // rad/s

    // Mecanum wheel velocity calculations
    float vel_fl = vx - vy - omega * (WHEEL_BASE + TRACK_WIDTH) / 2.0f;
    float vel_fr = vx + vy + omega * (WHEEL_BASE + TRACK_WIDTH) / 2.0f;
    float vel_bl = vx + vy - omega * (WHEEL_BASE + TRACK_WIDTH) / 2.0f;
    float vel_br = vx - vy + omega * (WHEEL_BASE + TRACK_WIDTH) / 2.0f;

    // Set target velocities
    motor_fl.target_velocity = vel_fl;
    motor_fr.target_velocity = vel_fr;
    motor_bl.target_velocity = vel_bl;
    motor_br.target_velocity = vel_br;

    // Calculate PID outputs
    float pid_fl = PID_Calculate(&motor_fl.pid, motor_fl.target_velocity, motor_fl.velocity, dt);
    float pid_fr = PID_Calculate(&motor_fr.pid, motor_fr.target_velocity, motor_fr.velocity, dt);
    float pid_bl = PID_Calculate(&motor_bl.pid, motor_bl.target_velocity, motor_bl.velocity, dt);
    float pid_br = PID_Calculate(&motor_br.pid, motor_br.target_velocity, motor_br.velocity, dt);

    // Convert to PWM values and set motor directions
    uint16_t pwm_fl = (uint16_t)fabs(pid_fl);
    uint16_t pwm_fr = (uint16_t)fabs(pid_fr);
    uint16_t pwm_bl = (uint16_t)fabs(pid_bl);
    uint16_t pwm_br = (uint16_t)fabs(pid_br);

    // Set PWM and direction for each motor
    // Front Left Motor
    PWM_Set_Duty(&htim2, TIM_CHANNEL_1, pwm_fl);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, pid_fl >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, pid_fl >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);

    // Front Right Motor
    PWM_Set_Duty(&htim2, TIM_CHANNEL_2, pwm_fr);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, pid_fr >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, pid_fr >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);

    // Back Left Motor
    PWM_Set_Duty(&htim2, TIM_CHANNEL_3, pwm_bl);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, pid_bl >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, pid_bl >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);

    // Back Right Motor
    PWM_Set_Duty(&htim2, TIM_CHANNEL_4, pwm_br);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, pid_br >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, pid_br >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM11_Init();
  MX_TIM10_Init();
  MX_I2C2_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM13_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();

  /* USER CODE BEGIN 2 */

  // Initialize BNO055 IMU
  bno055_assignI2C(&hi2c2);
  bno055_setup();
  bno055_setOperationModeNDOF();

  // Initialize motors with PID parameters
  Motor_Init(&motor_fl, 5.0f, 0.1f, 0.05f);
  Motor_Init(&motor_fr, 5.0f, 0.1f, 0.05f);
  Motor_Init(&motor_bl, 5.0f, 0.1f, 0.05f);
  Motor_Init(&motor_br, 5.0f, 0.1f, 0.05f);

  // Start encoder timers in encoder mode
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim9, TIM_CHANNEL_ALL);

  // Start PWM timers
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  // Initialize robot pose
  robot_pose.x = 0.0f;
  robot_pose.y = 0.0f;
  robot_pose.theta = 0.0f;
  robot_pose.velocity_x = 0.0f;
  robot_pose.velocity_y = 0.0f;
  robot_pose.angular_velocity = 0.0f;

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityNormal, 0, 256);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of encoderTask */
  osThreadDef(encoderTask, StartEncoderTask, osPriorityHigh, 0, 256);
  encoderTaskHandle = osThreadCreate(osThread(encoderTask), NULL);

  /* definition and creation of imuTask */
  osThreadDef(imuTask, StartIMUTask, osPriorityNormal, 0, 256);
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    HAL_StatusTypeDef status;
    uint32_t current_time = HAL_GetTick();
    dt = (current_time - prev_time_ms) / 1000.0f;
    prev_time_ms = current_time;

    // Receive controller data
    status = HAL_UART_Receive(&huart4, rxbuff, 32, 100);

    if (status == HAL_OK) {
        // Parse controller data
        lx = (rxbuff[1] & 0x80) ? (int32_t)rxbuff[1] - 256 : (int32_t)rxbuff[1];
        ly = (rxbuff[2] & 0x80) ? (int32_t)rxbuff[2] - 256 : (int32_t)rxbuff[2];
        rx = (rxbuff[3] & 0x80) ? (int32_t)rxbuff[3] - 256 : (int32_t)rxbuff[3];
        ry = (rxbuff[4] & 0x80) ? (int32_t)rxbuff[4] - 256 : (int32_t)rxbuff[4];

        cro = (rxbuff[5] & 0x80) ? (int32_t)rxbuff[5] - 256 : (int32_t)rxbuff[5];
        squ = (rxbuff[6] & 0x80) ? (int32_t)rxbuff[6] - 256 : (int32_t)rxbuff[6];
        tri = (rxbuff[7] & 0x80) ? (int32_t)rxbuff[7] - 256 : (int32_t)rxbuff[7];
        cir = (rxbuff[8] & 0x80) ? (int32_t)rxbuff[8] - 256 : (int32_t)rxbuff[8];
        up = (rxbuff[9] & 0x80) ? (int32_t)rxbuff[9] - 256 : (int32_t)rxbuff[9];
        down = (rxbuff[10] & 0x80) ? (int32_t)rxbuff[10] - 256 : (int32_t)rxbuff[10];
        left = (rxbuff[11] & 0x80) ? (int32_t)rxbuff[11] - 256 : (int32_t)rxbuff[11];
        right = (rxbuff[12] & 0x80) ? (int32_t)rxbuff[12] - 256 : (int32_t)rxbuff[12];
        ll1 = (rxbuff[13] & 0x80) ? (int32_t)rxbuff[13] - 256 : (int32_t)rxbuff[13];
        ll2 = (rxbuff[14] & 0x80) ? (int32_t)rxbuff[14] - 256 : (int32_t)rxbuff[14];
        rr1 = (rxbuff[15] & 0x80) ? (int32_t)rxbuff[15] - 256 : (int32_t)rxbuff[15];
        rr2 = (rxbuff[16] & 0x80) ? (int32_t)rxbuff[16] - 256 : (int32_t)rxbuff[16];
    } else {
        // Clear all inputs if no valid data received
        lx = ly = rx = ry = 0;
        cro = squ = tri = cir = up = down = left = right = 0;
        ll1 = ll2 = rr1 = rr2 = 0;
    }

    // Control base motors with enhanced PID control
    Control_Base_Motors(lx, ly, rx);

    // Ball picking mechanism
    if (cro == 1 && !prev_cro) {
        TIM13->CCR1 = 1000;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    }
    prev_cro = (cro == 1);

    if (down == 1 && !prev_down) {
        TIM13->CCR1 = 1000;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
    }
    prev_down = (down == 1);

    if (squ == 1 && !prev_squ) {
        TIM13->CCR1 = 0;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    }
    prev_squ = (squ == 1);

    // BLDC motor control
    if (ll1 == 1 && !prev_l1) {
        TIM3->CCR1 = 1000;
        TIM3->CCR2 = 1000;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    }
    prev_l1 = (ll1 == 1);

    if (rr1 == 1 && !prev_r1) {
        TIM3->CCR1 = 0;
        TIM3->CCR2 = 0;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    }
    prev_r1 = (rr1 == 1);

    if (ll2 == 1 && !prev_l2) {
        TIM3->CCR1 = 100;
        TIM3->CCR2 = 100;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    }
    prev_l2 = (ll2 == 1);

    // Servo control for seedling mechanism (fixed servo control issue)
    if (left == 1) {
        PWM_Set_Duty(&htim10, TIM_CHANNEL_1, 50);  // 0 degrees
    } else if (right == 1) {
        PWM_Set_Duty(&htim10, TIM_CHANNEL_1, 125); // 180 degrees
    }

    // Pneumatic control
    if (cir == 1) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    } else if (tri == 1) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    }

    osDelay(20); // 50Hz control loop
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
    // Debug output
    printf("Robot Pose: X=%.2f, Y=%.2f, Theta=%.2f\n",
           robot_pose.x, robot_pose.y, robot_pose.theta * 180.0f / M_PI);
    printf("Motor Velocities: FL=%.2f, FR=%.2f, BL=%.2f, BR=%.2f\n",
           motor_fl.velocity, motor_fr.velocity, motor_bl.velocity, motor_br.velocity);
    printf("IMU: Yaw=%.2f, Pitch=%.2f, Roll=%.2f\n",
           imu_euler.z, imu_euler.y, imu_euler.x);
    printf("Controller Input: LX=%ld, LY=%ld, RX=%ld\n", lx, ly, rx);

    osDelay(500); // Update every 500ms
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartEncoderTask */
/**
* @brief Function implementing the encoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEncoderTask */
void StartEncoderTask(void const * argument)
{
  /* USER CODE BEGIN StartEncoderTask */
  /* Infinite loop */
	  for(;;)
	  {
	    // Read encoder counts
	    Read_Encoders();

	    // Update motor velocities
	    Update_Motor_Velocity(&motor_fl, dt);
	    Update_Motor_Velocity(&motor_fr, dt);
	    Update_Motor_Velocity(&motor_bl, dt);
	    Update_Motor_Velocity(&motor_br, dt);

	    // Update robot pose using odometry
	    Update_Robot_Pose(dt);

	    // Wait for next update (20ms)
	    osDelay(20);
	  }
	  /* USER CODE END StartEncoderTask */
}

void StartIMUTask(void const * argument)
{
  /* USER CODE BEGIN StartIMUTask */
  /* Infinite loop */
  for(;;)
  {
    // Read IMU Euler angles (Yaw, Pitch, Roll)
    imu_euler = bno055_read_euler_vector();

    // Read IMU gyroscope data
    imu_gyro = bno055_read_gyroscope();

    // Read IMU accelerometer data (optional, in case needed)
    imu_accel = bno055_read_accelerometer();

    // Delay to match the IMU update rate (~100Hz max for BNO055 in NDOF)
    osDelay(10);
  }
  /* USER CODE END StartIMUTask */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
