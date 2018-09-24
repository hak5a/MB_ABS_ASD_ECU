/*
 * led_blink.c
 *
 *  Created on: Feb 16, 2018
 *      Author: hak5a
 */

/* Includes ------------------------------------------------------------------*/

#include <stdio.h>
#include "main.h"
#include "stm32f1xx_hal.h"

/* Private variables ---------------------------------------------------------*/

uint8_t LED_State = 0;
uint32_t time_led = 0;

/* Private function prototypes -----------------------------------------------*/


void LedBlink_run(void)
{
	uint32_t time_now = HAL_GetTick();
	if( LED_State == 0 && ( (time_now - time_led) >= 900 ) )
	{
	  HAL_GPIO_WritePin(GreenLED_GPIO_Port, GreenLED_Pin, GPIO_PIN_RESET);	// Led on
	  time_led = time_now;
	  LED_State = 1;
	}
	if( LED_State == 1 && ( (time_now - time_led) >= 100 ) )
	{
	  HAL_GPIO_WritePin(GreenLED_GPIO_Port, GreenLED_Pin, GPIO_PIN_SET);	// Led off
	  time_led = time_now;
	  LED_State = 0;
	}
}
