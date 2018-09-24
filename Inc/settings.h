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

#define DEFAULT_SPEED 1

#define EEADDR_SPEED_FL_L		0
#define EEADDR_SPEED_FL_H		1
#define EEADDR_SPEED_FR_L		2
#define EEADDR_SPEED_FR_H		3
#define EEADDR_SPEED_DIFF_L		4
#define EEADDR_SPEED_DIFF_H		5
#define EEADDR_PR_FL_L			6
#define EEADDR_PR_FL_H			7
#define EEADDR_PR_FR_L			8
#define EEADDR_PR_FR_H			9
#define EEADDR_PR_DIFF_L		10
#define EEADDR_PR_DIFF_H		11
//#define EEADDR_					12
//#define EEADDR_					13
//#define EEADDR_					14
//#define EEADDR_					15

struct abs_channel
{
	float abs_pulse_ratio;
	float speed;
};


#endif /* SETTINGS_H_ */
