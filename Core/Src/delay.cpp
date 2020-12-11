/*
 * delay.cpp
 *
 *  Created on: Dec 11, 2020
 *      Author: aronno
 */

#include "main.h"
#include "delay.h"

void delay_us (uint32_t us, TIM_HandleTypeDef *htim)
{
	__HAL_TIM_SET_COUNTER(htim,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(htim) < us);  // wait for the counter to reach the us input in the parameter
}

