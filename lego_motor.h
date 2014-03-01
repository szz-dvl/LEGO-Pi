#include "lego_shared.h"
#include <gsl/gsl_math.h>
#include <gsl/gsl_statistics.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_interp.h>
#include <gsl/gsl_spline.h>


#define HW_PWM			DELAY_VIA_PWM
#define PWIG_DEF 		PULSE_WIDTH_INCREMENT_GRANULARITY_US_DEFAULT
#define MAX_PW			ST_US / PWIG_DEF
#define BASE			370
#define MAX_THREADS		2
#define V2PW(x)     	        (x * (MAX_PW/MAX_VEL))
#define PW2V(x)     	        (x / (MAX_PW/MAX_VEL))
#define CLK_ID		        CLOCK_PROCESS_CPUTIME_ID
#define MAX_VEL		        200 /*si 9V y VEL = 160 es comportara com amb bateries de LEGO */
#define ENULL		       -1	/* Encoder desactivat */
#define SETUP		        INT_EDGE_SETUP
#define RISING		        INT_EDGE_RISING
#define MAX_COEF	        21
#define MIN_COEF	         5      /* AKIMA spline conditions */
#define PIDNULL		         0
#define PIDDEF		         1
#define TTCDEF		         55.7
#define MIN_VEL		         10
#define ST_US		         20000
#define USXT_MIN                 700
#define USXT_MAX                 7000


typedef pthread_mutex_t MUTEX;

struct encoder {
  int pin;		         /*  pin gpio  */
  int tics;                      /* tics que el motor ha recorregut des de l'ultim reset */
  TSPEC tmp;     		 /* temps base per calcular els delays, esta aqui per comoditat */ /* s'usa diferent per PID*/
  void (*isr)(void);             /* punter a la rutina d'interrupcio us intern de la llibreria */
  MUTEX mtx;			 /* mutex per actualitzar els valors*/
};
typedef struct encoder ENC;

struct pid {
  int svel;			       /* step velocitat */
  double cp[MAX_COEF];                 /* coeficients polinomi interpolador objectiu PID  << DEPRECATED ara son els punts absices directament */
  double cd[MAX_COEF];                 /* coeficients polinomi interpolador error objectiu PID << DEPRECATED ara son els punts absices directament */
  double kp;			       /*     gains	*/
  double ki;
  double kd;
  double ttc;				/* calibracio PID cada "ttc" ticks*/
  gsl_interp_accel *accelM;
  gsl_interp_accel *accelD;
  //podriem definir el tipus de interpolacio dinamicament també...
};
typedef struct pid PID;

struct motor {
  int id;
  int pinf;           /* pin en el que s'ha de genrar el pols si volem avan?ar */
  int pinr;           /* pin en el que hem de generar el pols si volem retrocedir */
  int chann;          /* canal DMA en el que configurarem el pols PWM, ha d'estar entre 0 i 14 (15 canals disponibles) */
  bool moving;	      /* bool = true si el motor s'esta movent, false altrament */
  int ticsxturn;      /* emagatzema els ticsxturn */
  ENC * enc1;         /* struct encoder 1 */
  ENC * enc2;         /* struct encoder 2 */
  PID * pid;
};
typedef struct motor MOTOR;

enum mot1 {M1_PINF = 4, M1_PINR = 17, M1_ENC1 = 27, M1_ENC2 = 22, M1_CHANN = 0}; //22
enum mot2 {M2_PINF = 25, M2_PINR = 18, M2_ENC1 = 24, M2_ENC2 = 23, M2_CHANN = 1};//23

	/* Cosillas del debug */

#define CLK_TST		CLOCK_MONOTONIC

	/* End Cosillas del debug */

extern MOTOR  motor1;
extern MOTOR  motor2;

extern bool wait_for_stop(MOTOR *, double);
extern void init_motors(void);
extern int motor_new(ENC *, ENC *, int);
extern int mot_stop(MOTOR *, bool reset);
extern int move_t (MOTOR *, int, char *, int, double);
extern void wfmtr (MOTOR *);
extern void wfmtrs ();
extern void mot_clear(void);
extern bool is_null (ENC *);
extern void reset_encoders(MOTOR *);
extern int get_ticks(MOTOR * mot);
extern TSPEC * get_time (ENC * enc);
extern int tticks (MOTOR *, int);
extern int ecount (MOTOR *);
extern int mot_reconf(MOTOR *, ENC *, ENC *);
extern void get_params(MOTOR *, int, int *, int *);
extern void cal_motors(int, double);
extern int pid_conf(MOTOR *, double *, double *);
extern int move (MOTOR *, char *, int);
extern bool pid_null(PID *);
extern void pid_setnull(PID *);
extern void pid_setgains (PID *, double, double, double);
extern void lock_motor(MOTOR * );
extern void unlock_motor(MOTOR * );
extern int move_sinc(char *, int);
extern int move_sinc_t (char *,int,int,double);
