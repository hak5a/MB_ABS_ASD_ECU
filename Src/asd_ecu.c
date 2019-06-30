/*
 * asd_ecu.c
 *
 *  Created on: Sep 23, 2018
 *      Author: hak5a
 */


/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "stm32f1xx_hal.h"
#include "asd_ecu.h"
#include "settings.h"

/* Private defines -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern struct abs_channel abs_channel_FL;
extern struct abs_channel abs_channel_FR;
extern struct abs_channel abs_channel_DIFF;

uint8_t TIM2_overflow = 0;
uint8_t TIM3_overflow = 0;
uint8_t TIM4_overflow = 0;

/* Private function prototypes -----------------------------------------------*/



void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint16_t input_capture;

	if (htim->Instance==TIM2)
	{
		__HAL_TIM_SET_COUNTER(htim, 0);    //reset counter after input capture interrupt occurs
		input_capture= __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);    //read TIM2 channel 1 capture value

//		HAL_GPIO_TogglePin(GreenLED_GPIO_Port, GreenLED_Pin);

		if(TIM2_overflow)
		{
//			HAL_GPIO_TogglePin(GreenLED_GPIO_Port, GreenLED_Pin);

			TIM2_overflow = 0;
			abs_channel_FL.speed = 0.1;
		}
		else
		{
			abs_channel_FL.speed = (float)(F_COUNTER*ABS_PULSE_PRESCALER/(input_capture))/abs_channel_FL.abs_pulse_ratio;
		}
	}
	if (htim->Instance==TIM3)
	{
		__HAL_TIM_SET_COUNTER(htim, 0);    //reset counter after input capture interrupt occurs
		input_capture= __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);    //read TIM3 channel 1 capture value

//		HAL_GPIO_TogglePin(GreenLED_GPIO_Port, GreenLED_Pin);

		if(TIM3_overflow)
		{
//			HAL_GPIO_TogglePin(GreenLED_GPIO_Port, GreenLED_Pin);

			TIM3_overflow = 0;
			abs_channel_FR.speed = 0.1;
		}
		else
		{
			abs_channel_FR.speed = (float)(F_COUNTER*ABS_PULSE_PRESCALER/(input_capture))/abs_channel_FR.abs_pulse_ratio;
		}
	}
	if (htim->Instance==TIM4)
	{
		__HAL_TIM_SET_COUNTER(htim, 0);    //reset counter after input capture interrupt occurs
		input_capture= __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);    //read TIM4 channel 1 capture value

//		HAL_GPIO_TogglePin(GreenLED_GPIO_Port, GreenLED_Pin);

		if(TIM4_overflow)
		{
//			HAL_GPIO_TogglePin(GreenLED_GPIO_Port, GreenLED_Pin);

			TIM4_overflow = 0;
			abs_channel_DIFF.speed = 0.1;
		}
		else
		{
			abs_channel_DIFF.speed = (float)(F_COUNTER*ABS_PULSE_PRESCALER/(input_capture))/abs_channel_DIFF.abs_pulse_ratio;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance==TIM2) //check if the interrupt comes from TIM2
	{
    	TIM2_overflow = 1;
    	abs_channel_FL.speed = 0.1;
	}
    if (htim->Instance==TIM3) //check if the interrupt comes from TIM3
	{
		TIM3_overflow = 1;
		abs_channel_FR.speed = 0.1;
	}
    if (htim->Instance==TIM4) //check if the interrupt comes from TIM4
	{
		TIM4_overflow = 1;
		abs_channel_DIFF.speed = 0.1;
	}
}
