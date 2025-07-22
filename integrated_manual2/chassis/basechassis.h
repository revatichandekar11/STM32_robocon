/*
 * basechassis.h
 *
 *  Created on: Aug 28, 2024
 *      Author: revat
 */

#ifndef BASECHASSIS_H_
#define BASECHASSIS_H_

#include "stdio.h"
#include "stm32f4xx_hal.h"

void forward(void);
void backward(void);
void rightchassis(void);
void leftchassis(void);
void anticlockwise(void);
void clockwise(void);

#endif /* BASECHASSIS_H_ */
