/*
* This file is part of LEGO-Pi.
*
* Copyright (Copyplease) szz-dvl.
*
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
*/

#include "lego_shared.h"
#include <gsl/gsl_math.h>
#include <gsl/gsl_statistics.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_interp.h>
#include <gsl/gsl_spline.h>


#define HW_PWM			DELAY_VIA_PWM
#define ST_US		        3000
#define PWIG_DEF 	        15
#define MAX_PW			ST_US / PWIG_DEF
#define BASE			400
#define MAX_THREADS		2
#define V2PW(x)     	        x                       /* if you want to mess with PWM change this to: (x * (MAX_PW/MAX_VEL)) */
#define PW2V(x)     	        x                       /* if you want to mess with PWM change this to: (x / (MAX_PW/MAX_VEL)) */
#define CLK_ID		        CLOCK_PROCESS_CPUTIME_ID
#define MAX_VEL		        200                    
#define ENULL		        -1	               /* Encoder disabled */
#define SETUP		        INT_EDGE_SETUP
#define RISING		        INT_EDGE_RISING
#define MAX_COEF	        21
#define MIN_COEF	        5                      /* AKIMA spline conditions */
#define PIDDEF		        0
#define TTCDEF		        36                     /* ten times x turn*/
#define MIN_VEL		        15
#define USXT_MIN                700
#define USXT_MAX                7000

#define MIN_PORT_MT             0
#define MAX_PORT_MT             1


typedef enum {
  FWD,
  BWD
} dir;

typedef pthread_mutex_t MUTEX;

struct encoder {
  int pin;		                /* pin gpio  */
  int tics;                             /* ticks since the last reset */
  TSPEC tmp;     		        /* TSPEC structure, used for the default ISRs */ 
  void (*isr)(void);                    /* pointer to the function that will be executed every time a tick comes, namely ISR */ 
  MUTEX mtx;			        /* mutex used by the default ISRs */
};
typedef struct encoder ENC;

struct pid {
  bool active;
  int svel;			       /* velocity step */
  double cp[MAX_COEF];                 /* time between ticks array */
  double cd[MAX_COEF];                 /* physical error array */
  double kp;			       /* gains	*/
  double ki;
  double kd;
  double ttc;			       /* PID calibration every "ttc" ticks*/
  gsl_interp_accel *accelM;            /* accelerator for interpolations on the data set */
  gsl_interp_accel *accelD;

};
typedef struct pid PID;

struct motor {
  int id;
  int pinf;                            /* pin responsible of the forward motion */
  int pinr;                            /* pin responsible of the forward motion */
  int chann;                           /* DMA channel, to generate the pulses */
  bool moving;	                       /* bool = true if the motor is in motion, false otherwise */
  int ticsxturn;                       /* expected tics per a turn of the output hub */
  ENC * enc1;                          /* encoder line 1 */
  ENC * enc2;                          /* encoder line 2 */
  PID * pid;                           /* P.I.D */ 
};
typedef struct motor MOTOR;

enum mot1 {M1_PINF = 4, M1_PINR = 17, M1_ENC1 = 27, M1_ENC2 = 22, M1_CHANN = 8}; //8!!
enum mot2 {M2_PINF = 25, M2_PINR = 18, M2_ENC1 = 24, M2_ENC2 = 23, M2_CHANN = 6};


ENC edisable;
#define ECNULL                  &edisable


extern bool       mt_init();
extern bool       mt_set_verbose(int lvl);
extern MOTOR *    mt_new(ENC * e1, ENC * e2, int port);
extern bool       mt_reconf(MOTOR * m, ENC * e1, ENC * e2);
extern int        mt_stop(MOTOR * m, bool reset);
extern int        mt_enc_is_null (MOTOR * m, int eid);
extern bool       mt_pid_conf(MOTOR * m , double micras [], double desv []);
extern int        mt_pid_is_null(MOTOR * m);
extern bool       mt_pid_off (MOTOR * m);
extern bool       mt_pid_on (MOTOR * m);
extern bool       mt_pid_set_gains (MOTOR * m, double Kp, double Ki, double Kd);
extern bool       mt_move (MOTOR * m, dir dir, int vel);
extern bool       mt_wait (MOTOR * m);
extern int        mt_wait_all ();
extern bool       mt_move_t (MOTOR * m, int ticks, dir dir, int vel, double posCtrl);
extern bool       mt_wait_for_stop (MOTOR * m, double delay);
extern bool       mt_reset_enc (MOTOR * m);
extern int        mt_get_ticks (MOTOR * m);
extern TSPEC *    mt_get_time  (MOTOR * m, int eid);
extern int        mt_tticks (MOTOR * m, int turns);
extern int        mt_enc_count (MOTOR * m);
extern bool       mt_calibrate (int samples, double wait_between_samples);
extern bool       mt_lock (MOTOR * m);
extern bool       mt_unlock (MOTOR * m);
extern bool       mt_move_sinc (dir dir, int vel);
extern bool       mt_move_sinc_t (dir dir, int vel, int limit, double posCtrl);
extern void       mt_shutdown ();








