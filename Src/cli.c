/*
 * cli.c
 *
 *  Created on: Feb 15, 2018
 *      Author: hak5a
 */

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "asd_ecu.h"
#include "settings.h"

/* Private defines -----------------------------------------------------------*/

#define COMMAND_BUFFER_SIZE 20
#define COMMAND_SIZE 		10
#define ATTRIBUTE_SIZE		10

/* Private variables ---------------------------------------------------------*/

extern UART_HandleTypeDef huart1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern struct abs_channel abs_channel_FL;
extern struct abs_channel abs_channel_FR;
extern struct abs_channel abs_channel_DIFF;


unsigned int uptime = 0;
uint32_t time_uptime = 0;

char rx_command_buffer[COMMAND_BUFFER_SIZE];
unsigned char rx_command_buffer_i = 0;

/* Private function prototypes -----------------------------------------------*/

void CLI_Parse_Commands(char *command);
void Print_Settings(void);

extern void Save_Settings(void);
extern void Reset_Settings(void);



void CLI_run(void)
{
	char rx_c;

	if( HAL_UART_Receive (&huart1, (uint8_t *)&rx_c, 1, 10) == HAL_OK )
	{
	  if(rx_command_buffer_i < COMMAND_BUFFER_SIZE)
	  {
		  rx_command_buffer[rx_command_buffer_i++] = rx_c;
		  if(rx_c == '\n')
		  {
			  rx_command_buffer[rx_command_buffer_i] = 0;
			  CLI_Parse_Commands(rx_command_buffer);
			  rx_command_buffer_i = 0;
		  }
	  }
	  else
	  {
		  rx_command_buffer_i = 0;
	  }
	}
#if CLI_PRINT_UPTIME == 1
	uint32_t time_now = HAL_GetTick();
	if( (time_now - time_uptime) >= 10000 )
	{
	  time_uptime = time_now;
	  printf("Uptime: %u sec\n\r", 10 * ++uptime);
	}
#endif
}

void CLI_Parse_Commands(char *full_command)
{
	//printf("debug... Parse command... %s \n\n\ŋ\r", full_command);

	char temp_str[COMMAND_BUFFER_SIZE];
	char parsed_command[COMMAND_SIZE];
	char parsed_attribute[ATTRIBUTE_SIZE];

    strcpy(temp_str, full_command);
    char *token;

    /* get the first token */
    token = strtok(temp_str, " \n");
    if( token != NULL )
        strcpy(parsed_command, token);
    else
        strcpy(parsed_command, " ");
    /* get the second token */
    token = strtok(NULL, " \n");
    if( token != NULL )
        strcpy(parsed_attribute, token);
    else
        strcpy(parsed_attribute, " ");

    // debug...
    if(strcmp(parsed_command, "hello") == 0)
    {
        printf("Hello %s!\n\r", parsed_attribute);
    }


    else if(strcmp(parsed_command, "speed?") == 0 || strcmp(parsed_command, "s") == 0)
    {
    	printf("Speed: \n\r");
        printf("Front Left     : %.1f km/h\n\r", get_speed_FL());
        printf("Front Right    : %.1f km/h\n\r", get_speed_FR());
        printf("Diff           : %.1f km/h\n\r", get_speed_DIFF());
        printf("Vehicle speed  : %.1f km/h\n\r", get_vehicle_speed());
        printf("Slipping ratio : %.1f \n\r"    , get_slipping_ratio());
        printf("ASD valve state: %d   \n\r"    , get_ASD_valve_state());
    }
    else if(strcmp(parsed_command, "save") == 0)
    {
    	Save_Settings();
        printf("OK, All settings saved");
    }
    else if(strcmp(parsed_command, "pr_fl") == 0)
    {
    	abs_channel_FL.abs_pulse_ratio = atof(parsed_attribute);
        printf("OK, Pulse Ratio is now %f\n\r", abs_channel_FL.abs_pulse_ratio);
    }
    else if(strcmp(parsed_command, "pr_fr") == 0)
    {
    	abs_channel_FR.abs_pulse_ratio = atof(parsed_attribute);
        printf("OK, Pulse Ratio is now %f\n\r", abs_channel_FR.abs_pulse_ratio);
    }
    else if(strcmp(parsed_command, "pr_diff") == 0)
    {
    	abs_channel_DIFF.abs_pulse_ratio = atof(parsed_attribute);
        printf("OK, Pulse Ratio is now %f\n\r", abs_channel_DIFF.abs_pulse_ratio);
    }
    else if(strcmp(parsed_command, "p") == 0)
    {
        Print_Settings();
    }
    else if(strcmp(parsed_command, "reset") == 0)
    {
    	Reset_Settings();
    	printf("OK, Factory default settings restored (not saved!)\n\r");
    }
    else
    {
    	printf("** MB 124 ADS ECU **\n\r");
    	printf("Made by: hannu.kopsa@iki.fi\n\r");
    	printf("Supported Commands:\n\r");
        printf("  speed? or s                : Print speed\n\r");
        printf("  pr_fl       [pulse ratio]  : Set ABS sensor pulse ratio\n\r");
        printf("  pr_fr       [pulse ratio]  : Set ABS sensor pulse ratio\n\r");
        printf("  pr_diff     [pulse ratio]  : Set ABS sensor pulse ratio\n\r");
        printf("  p                          : Print settings\n\r");
        printf("  reset                      : Restore factory default settings\n\r");
        printf("  save                       : Save all settings to memory\n\r");
    }
    printf("\n\r");
}

void Print_Settings(void)
{
    printf("\nSettings:\n\r");
    printf("  ABS Pulse Ratio FL:   %f\n\r", abs_channel_FL.abs_pulse_ratio);
    printf("  ABS Pulse Ratio FR:   %f\n\r", abs_channel_FR.abs_pulse_ratio);
    printf("  ABS Pulse Ratio DIFF: %f\n\r", abs_channel_DIFF.abs_pulse_ratio);
    printf("\n\r");
}
