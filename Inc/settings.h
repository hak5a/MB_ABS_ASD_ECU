/*
 * settings.h
 *
 *  Created on: Apr 10, 2018
 *      Author: hak5a
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

// Debug stuff
#define CLI_PRINT_UPTIME	0	// debug thing


// Settings
#define DEFAULT_ABS_PULSE_RATIO_FL 		13.74484598968620000000
#define DEFAULT_ABS_PULSE_RATIO_FR 		13.74484598968620000000
#define DEFAULT_ABS_PULSE_RATIO_DIFF 	13.74484598968620000000

#define F_COUNTER				100000
#define ABS_PULSE_PRESCALER		8

#define DEFAULT_SPEED 1

#define EEADDR_PR_FL_L			0
#define EEADDR_PR_FL_H			1
#define EEADDR_PR_FR_L			2
#define EEADDR_PR_FR_H			3
#define EEADDR_PR_DIFF_L		4
#define EEADDR_PR_DIFF_H		5
//#define EEADDR_					6
//#define EEADDR_					7
//#define EEADDR_					8
//#define EEADDR_					9
//#define EEADDR_					10
//#define EEADDR_					11
//#define EEADDR_					12
//#define EEADDR_					13
//#define EEADDR_					14
//#define EEADDR_					15

struct abs_channel
{
	float abs_pulse_ratio;
	uint32_t raw_speed;
	float speed;
};


#endif /* SETTINGS_H_ */
