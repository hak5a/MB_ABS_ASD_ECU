/*
 * asd_ecu.h
 *
 *  Created on: Sep 23, 2018
 *      Author: hak5a
 */

#ifndef ASD_ECU_H_
#define ASD_ECU_H_


/* Public function prototypes -----------------------------------------------*/

void asd_control(void);
uint8_t get_ASD_valve_state(void);
float get_slipping_ratio(void);
float get_vehicle_speed(void);
float get_speed_FL(void);
float get_speed_FR(void);
float get_speed_DIFF(void);
float get_frequency_FL(void);
float get_frequency_FR(void);
float get_frequency_DIFF(void);


#endif /* ASD_ECU_H_ */
