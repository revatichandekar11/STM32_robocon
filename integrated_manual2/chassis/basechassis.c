/*
 * basechassis.c
 *
 *  Created on: Aug 28, 2024
 *      Author: revat
 */


#include "basechassis.h"

void forward(void){
	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);

	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);

	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);

	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
}
void backward(void){
	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);

	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);

	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);

	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
}
void rightchassis(void)
{
  	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);

  	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);

  	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);

  	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
}
void leftchassis(void)
{
	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);

	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);

	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);

	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
}
void clockwise(void)
{
	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);

	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);

	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);

	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
}
void anticlockwise(void)
{
	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);

	  	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);

	  	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);

	  	   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
}
