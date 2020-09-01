/*
 * brightnessControl.h
 *
 *  Created on: 29 Mar 2020
 *      Author: Steve
 */

#ifndef MAIN_BRIGHTNESSCONTROL_H_
#define MAIN_BRIGHTNESSCONTROL_H_

void adc_setup(void);
void brightness_task( void * pvParameters );
void initialisebrightnessControlPWM(void);
void pausebrightnessControlPWM(void);
void resumebrightnessControlPWM(void);


#endif /* MAIN_BRIGHTNESSCONTROL_H_ */
