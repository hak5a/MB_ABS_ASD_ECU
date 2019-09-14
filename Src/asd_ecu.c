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
#include "frequency_counter.h"
#include "settings.h"

/* Private defines -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

extern struct abs_channel abs_channel_FL;
extern struct abs_channel abs_channel_FR;
extern struct abs_channel abs_channel_DIFF;

static uint8_t  asd_valve_state = 0;

/* Private function prototypes -----------------------------------------------*/


void asd_control(void)
{
	static uint8_t  new_asd_valve_state = 0;
	static uint32_t time_valve_on = 0;

	uint32_t time_now = HAL_GetTick();

	if( get_slipping_ratio() > DEFAULT_SLIPPING_TRUE_THRESHOLD && get_vehicle_speed() < DEFAULT_ASD_MAX_SPEED )
		new_asd_valve_state = 1;
	else if ( get_slipping_ratio() < DEFAULT_SLIPPING_FALSE_THRESHOLD )
		new_asd_valve_state = 0;

	if( asd_valve_state == 0 && new_asd_valve_state == 1 )
	{
		time_valve_on = time_now;
		asd_valve_state = 1;
	}

	if( asd_valve_state == 1 && (time_now - time_valve_on) > DEFAULT_VALVE_RELEASE_DELAY )
	{
		asd_valve_state = 0;
	}

	if( asd_valve_state == 1 )
	{
		//HAL_GPIO_WritePin(ASD_Valve_GPIO_Port, ASD_Valve_Pin, GPIO_PIN_SET);		// Valve ON

		HAL_GPIO_TogglePin(ASD_Valve_GPIO_Port, ASD_Valve_Pin); // test!!!!!!!!!!
	}
	else
	{
		HAL_GPIO_WritePin(ASD_Valve_GPIO_Port, ASD_Valve_Pin, GPIO_PIN_RESET);	// Valve OFF
	}

}

uint8_t get_ASD_valve_state(void)
{
	return asd_valve_state;
}

float get_slipping_ratio(void)
{
	float average_speed_f = (get_speed_FL() + get_speed_FR()) / 2;
	float slipping_ratio = get_speed_DIFF() / average_speed_f - 1.0;
	return slipping_ratio;
}

float get_vehicle_speed(void)
{
	float average_speed = (get_speed_FL() + get_speed_FR()) / 2;
	return average_speed;
}

float get_speed_FL(void)
{
	return (float)abs_channel_FL.frequency/abs_channel_FL.abs_pulse_ratio;
}

float get_speed_FR(void)
{
	return (float)abs_channel_FR.frequency/abs_channel_FR.abs_pulse_ratio;
}

float get_speed_DIFF(void)
{
	return (float)abs_channel_DIFF.frequency/abs_channel_DIFF.abs_pulse_ratio;
}

float get_frequency_FL(void)
{
	return (float)abs_channel_FL.frequency;
}

float get_frequency_FR(void)
{
	return (float)abs_channel_FR.frequency;
}

float get_frequency_DIFF(void)
{
	return (float)abs_channel_DIFF.frequency;
}

