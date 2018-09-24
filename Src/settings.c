/*
 * settings.c
 *
 *  Created on: Apr 10, 2018
 *      Author: hak5a
 */

#include "main.h"
#include "stm32f1xx_hal.h"
#include "settings.h"
#include "eeprom.h"
#include "asd_ecu.h"


struct abs_channel abs_channel_FL;
struct abs_channel abs_channel_FR;
struct abs_channel abs_channel_DIFF;

/**
  * @brief  Save Settings
  * @param  None
  * @retval None
  */
void Save_Settings(void)
{
	uint16_t 	EEdata16bit_L;
	uint16_t 	EEdata16bit_H;
	float		EEdatafloat;

	/* Unlock the Flash Program Erase controller */
	HAL_FLASH_Unlock();

	// template
	//EEdatafloat = xxx_float_variable;
	//EEdata16bit_L = *((uint16_t*) &EEdatafloat + 0);
	//EEdata16bit_H = *((uint16_t*) &EEdatafloat + 1);
	//EE_WriteVariable(EEADDR_XXX_L, EEdata16bit_L);
	//EE_WriteVariable(EEADDR_XXX_H, EEdata16bit_H);

	// abs_channel_FL
	EEdatafloat = abs_channel_FL.speed;
	EEdata16bit_L = *((uint16_t*) &EEdatafloat + 0);
	EEdata16bit_H = *((uint16_t*) &EEdatafloat + 1);
	EE_WriteVariable(EEADDR_SPEED_FL_L, EEdata16bit_L);
	EE_WriteVariable(EEADDR_SPEED_FL_H, EEdata16bit_H);
	EEdatafloat = abs_channel_FL.abs_pulse_ratio;
	EEdata16bit_L = *((uint16_t*) &EEdatafloat + 0);
	EEdata16bit_H = *((uint16_t*) &EEdatafloat + 1);
	EE_WriteVariable(EEADDR_PR_FL_L, EEdata16bit_L);
	EE_WriteVariable(EEADDR_PR_FL_H, EEdata16bit_H);

	// abs_channel_FR
	EEdatafloat = abs_channel_FR.speed;
	EEdata16bit_L = *((uint16_t*) &EEdatafloat + 0);
	EEdata16bit_H = *((uint16_t*) &EEdatafloat + 1);
	EE_WriteVariable(EEADDR_SPEED_FR_L, EEdata16bit_L);
	EE_WriteVariable(EEADDR_SPEED_FR_H, EEdata16bit_H);
	EEdatafloat = abs_channel_FR.abs_pulse_ratio;
	EEdata16bit_L = *((uint16_t*) &EEdatafloat + 0);
	EEdata16bit_H = *((uint16_t*) &EEdatafloat + 1);
	EE_WriteVariable(EEADDR_PR_FR_L, EEdata16bit_L);
	EE_WriteVariable(EEADDR_PR_FR_H, EEdata16bit_H);

	// abs_channel_DIFF
	EEdatafloat = abs_channel_DIFF.speed;
	EEdata16bit_L = *((uint16_t*) &EEdatafloat + 0);
	EEdata16bit_H = *((uint16_t*) &EEdatafloat + 1);
	EE_WriteVariable(EEADDR_SPEED_DIFF_L, EEdata16bit_L);
	EE_WriteVariable(EEADDR_SPEED_DIFF_H, EEdata16bit_H);
	EEdatafloat = abs_channel_DIFF.abs_pulse_ratio;
	EEdata16bit_L = *((uint16_t*) &EEdatafloat + 0);
	EEdata16bit_H = *((uint16_t*) &EEdatafloat + 1);
	EE_WriteVariable(EEADDR_PR_DIFF_L, EEdata16bit_L);
	EE_WriteVariable(EEADDR_PR_DIFF_H, EEdata16bit_H);
}

/**
  * @brief  Restore Settings
  * @param  None
  * @retval None
  */
void Restore_Settings(void)
{
	uint16_t 	EEdata16bit;
	float		EEdatafloat;
	uint8_t		err;

//	// Other setting...
//	err = FALSE;
//	if( EE_ReadVariable(EEADDR_xxx_L, &EEdata16bit) != 0 )	// Read value from EEPROM
//		err = TRUE;
//	*((uint16_t*) &EEdatafloat + 0) = EEdata16bit;
//	if( EE_ReadVariable(EEADDR_xxx_H, &EEdata16bit) != 0 )	// Read value from EEPROM
//		err = TRUE;
//	*((uint16_t*) &EEdatafloat + 1) = EEdata16bit;
//	if(err == TRUE)
//	{
//		do_something(1.0);				// if can not read, use default
//	}
//	else
//	{
//		do_something(EEdatafloat);
//	}

	// ABS Pulse Ratio Front Left
	err = FALSE;
	if( EE_ReadVariable(EEADDR_PR_FL_L, &EEdata16bit) != 0 )	// Read value from EEPROM
		err = TRUE;
	*((uint16_t*) &EEdatafloat + 0) = EEdata16bit;
	if( EE_ReadVariable(EEADDR_PR_FL_H, &EEdata16bit) != 0 )	// Read value from EEPROM
		err = TRUE;
	*((uint16_t*) &EEdatafloat + 1) = EEdata16bit;
	if(err == TRUE)
	{
		abs_channel_FL.abs_pulse_ratio = DEFAULT_ABS_PULSE_RATIO_FL;					// if can not read, use default
	}
	else
	{
		abs_channel_FL.abs_pulse_ratio = EEdatafloat;
	}

	// ABS Pulse Ratio Front Right
	err = FALSE;
	if( EE_ReadVariable(EEADDR_PR_FR_L, &EEdata16bit) != 0 )	// Read value from EEPROM
		err = TRUE;
	*((uint16_t*) &EEdatafloat + 0) = EEdata16bit;
	if( EE_ReadVariable(EEADDR_PR_FR_H, &EEdata16bit) != 0 )	// Read value from EEPROM
		err = TRUE;
	*((uint16_t*) &EEdatafloat + 1) = EEdata16bit;
	if(err == TRUE)
	{
		abs_channel_FR.abs_pulse_ratio = DEFAULT_ABS_PULSE_RATIO_FR;					// if can not read, use default
	}
	else
	{
		abs_channel_FR.abs_pulse_ratio = EEdatafloat;
	}

	// ABS Pulse Ratio Diff
	err = FALSE;
	if( EE_ReadVariable(EEADDR_PR_DIFF_L, &EEdata16bit) != 0 )	// Read value from EEPROM
		err = TRUE;
	*((uint16_t*) &EEdatafloat + 0) = EEdata16bit;
	if( EE_ReadVariable(EEADDR_PR_DIFF_H, &EEdata16bit) != 0 )	// Read value from EEPROM
		err = TRUE;
	*((uint16_t*) &EEdatafloat + 1) = EEdata16bit;
	if(err == TRUE)
	{
		abs_channel_DIFF.abs_pulse_ratio = DEFAULT_ABS_PULSE_RATIO_DIFF;					// if can not read, use default
	}
	else
	{
		abs_channel_DIFF.abs_pulse_ratio = EEdatafloat;
	}

	// Front left speed
	err = FALSE;
	if( EE_ReadVariable(EEADDR_SPEED_FL_L, &EEdata16bit) != 0 )	// Read value from EEPROM
		err = TRUE;
	*((uint16_t*) &EEdatafloat + 0) = EEdata16bit;
	if( EE_ReadVariable(EEADDR_SPEED_FL_H, &EEdata16bit) != 0 )	// Read value from EEPROM
		err = TRUE;
	*((uint16_t*) &EEdatafloat + 1) = EEdata16bit;
	if(err == TRUE)
	{
		abs_channel_FL.speed = DEFAULT_SPEED;					// if can not read, use default
	}
	else
	{
		abs_channel_FL.speed = EEdatafloat;
	}
	pulse_generator_set_speed_FL(abs_channel_FL.speed);

	// Front right speed
	err = FALSE;
	if( EE_ReadVariable(EEADDR_SPEED_FR_L, &EEdata16bit) != 0 )	// Read value from EEPROM
		err = TRUE;
	*((uint16_t*) &EEdatafloat + 0) = EEdata16bit;
	if( EE_ReadVariable(EEADDR_SPEED_FR_H, &EEdata16bit) != 0 )	// Read value from EEPROM
		err = TRUE;
	*((uint16_t*) &EEdatafloat + 1) = EEdata16bit;
	if(err == TRUE)
	{
		abs_channel_FR.speed = DEFAULT_SPEED;				// if can not read, use default
	}
	else
	{
		abs_channel_FR.speed = EEdatafloat;
	}
	pulse_generator_set_speed_FR(abs_channel_FR.speed);

	// Diff (rear) speed
	err = FALSE;
	if( EE_ReadVariable(EEADDR_SPEED_DIFF_L, &EEdata16bit) != 0 )	// Read value from EEPROM
		err = TRUE;
	*((uint16_t*) &EEdatafloat + 0) = EEdata16bit;
	if( EE_ReadVariable(EEADDR_SPEED_DIFF_H, &EEdata16bit) != 0 )	// Read value from EEPROM
		err = TRUE;
	*((uint16_t*) &EEdatafloat + 1) = EEdata16bit;
	if(err == TRUE)
	{
		abs_channel_DIFF.speed = DEFAULT_SPEED;				// if can not read, use default
	}
	else
	{
		abs_channel_DIFF.speed = EEdatafloat;
	}
	pulse_generator_set_speed_DIFF(abs_channel_DIFF.speed);
}

/**
  * @brief  Reset Settings
  * @param  None
  * @retval None
  */
void Reset_Settings(void)
{
	abs_channel_FL.abs_pulse_ratio = DEFAULT_ABS_PULSE_RATIO_FL;
	abs_channel_FR.abs_pulse_ratio = DEFAULT_ABS_PULSE_RATIO_FR;
	abs_channel_DIFF.abs_pulse_ratio = DEFAULT_ABS_PULSE_RATIO_DIFF;
	abs_channel_FL.speed = DEFAULT_SPEED;
	abs_channel_FR.speed = DEFAULT_SPEED;
	abs_channel_DIFF.speed = DEFAULT_SPEED;
}
