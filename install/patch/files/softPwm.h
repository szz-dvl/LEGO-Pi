/*
* This file is part of the LEGO-PI project and at the same time a modification 
* to the file "softPwm.h" contained in the wiringPi project by Gordon Henderson.
* This lines have been copied (actually plagiarized) from the file "pwm.h" of the 
* RPIO project by Chris Hager (with a few modifications needed by the LEGO-PI project).
* You can reach the original file at:
* 
* https://github.com/metachris/RPIO/blob/master/source/c_pwm/pwm.h
* 
* Copyright (C) (Copyplease) 2014 szz-dvl
*
* License
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Affero General Public License as published
* by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Affero General Public License for more details at
* <http://www.gnu.org/licenses/agpl-3.0-standalone.html>
*
*/

#define DELAY_VIA_PWM   0
#define DELAY_VIA_PCM   1
#define LOG_LEVEL_DEBUG 	0
#define LOG_LEVEL_ERRORS 	1
#define LOG_LEVEL_DEFAULT LOG_LEVEL_DEBUG

// Default subcycle time
#define SUBCYCLE_TIME_US_DEFAULT 20000

// Subcycle minimum. We kept seeing no signals and strange behavior of the RPi
#define SUBCYCLE_TIME_US_MIN 3000

// Default pulse-width-increment-granularity
#define PULSE_WIDTH_INCREMENT_GRANULARITY_US_DEFAULT 10


#ifdef __cplusplus
extern "C" {
#endif

//PWM

extern int spwm_setup(int, int);
extern void spwm_set_loglevel(int);

extern int spwm_init_channel(int, int);
extern int spwm_clear_channel(int);
extern int spwm_clear_channel_gpio(int, int);
extern int spwm_print_channel(int);

extern int spwm_add_channel_pulse(int, int, int, int);
extern char* spwm_get_error_message(void);
extern void spwm_set_softfatal(int);

extern int spwm_is_setup(void);
extern int spwm_is_channel_initialized(int);
extern int spwm_get_pulse_incr_us(void);
extern int spwm_get_channel_subcycle_time_us(int);

extern void spwm_shutdown(void);

#ifdef __cplusplus
}
#endif
