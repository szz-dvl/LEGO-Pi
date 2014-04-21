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
#define MAX_VEL		        200                    /*si 9V y VEL = 160 es comportara com amb bateries de LEGO */
#define ENULL		        -1	               /* Encoder desactivat */
#define SETUP		        INT_EDGE_SETUP
#define RISING		        INT_EDGE_RISING
#define MAX_COEF	        21
#define MIN_COEF	        5                      /* AKIMA spline conditions */
#define PIDNULL		        0
#define PIDDEF		        1
#define TTCDEF		        55
#define MIN_VEL		        10
#define ST_US		        20000
#define USXT_MIN                700
#define USXT_MAX                7000


typedef pthread_mutex_t MUTEX;

struct encoder {
  int pin;		                /*  pin gpio  */
  int tics;                             /* tics que el motor ha recorregut des de l'ultim reset */
  TSPEC tmp;     		        /* temps base per calcular els delays, esta aqui per comoditat */ /* s'usa diferent per PID*/
  void (*isr)(void);                    /* punter a la rutina d'interrupcio us intern de la llibreria */ //FAILING NOW!!
  MUTEX mtx;			        /* mutex per actualitzar els valors*/
};
typedef struct encoder ENC;

struct pid {
  int svel;			       /* step velocitat */
  double cp[MAX_COEF];                 /* coeficients polinomi interpolador objectiu PID  << DEPRECATED ara son els punts absices directament */
  double cd[MAX_COEF];                 /* coeficients polinomi interpolador error objectiu PID << DEPRECATED ara son els punts absices directament */
  double kp;			       /*     gains	*/
  double ki;
  double kd;
  double ttc;			       /* calibracio PID cada "ttc" ticks*/
  gsl_interp_accel *accelM;
  gsl_interp_accel *accelD;

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
  PID * pid;          /* struct pid */ 
};
typedef struct motor MOTOR;

enum mot1 {M1_PINF = 4, M1_PINR = 17, M1_ENC1 = 27, M1_ENC2 = 22, M1_CHANN = 0}; 
enum mot2 {M2_PINF = 25, M2_PINR = 18, M2_ENC1 = 24, M2_ENC2 = 23, M2_CHANN = 1};

	/* Cosillas del debug */

#define CLK_TST		CLOCK_MONOTONIC

	/* End Cosillas del debug */

//extern MOTOR  motor1; //mirara como hay que hacer para evitar externos i poder configurar la interrupcción
//extern MOTOR  motor2;

extern void mt_init(void);
extern int  mt_new(MOTOR * m , ENC * e1, ENC * e2, int id);
extern int  mt_reconf(MOTOR *, ENC *, ENC *);
extern int  mt_pid_conf(MOTOR *, double *, double *);
extern int  mt_stop(MOTOR *, bool reset);
extern bool mt_enc_is_null (ENC *);
extern bool mt_pid_is_null(PID *);
extern void mt_pid_set_null(PID *);
extern void mt_pid_set_gains (PID *, double Kp, double Ki, double Kd);
extern int  mt_move (MOTOR *, char * dir, int vel);
extern void mt_wait (MOTOR *);
extern void mt_wait_all ();
extern int  mt_move_t (MOTOR * mot, int ticks, char * dir, int vel, double posCtrl);
extern bool mt_wait_for_stop(MOTOR *, double delay);
extern void mt_reset_enc(MOTOR *);
extern int  mt_get_ticks(MOTOR *);
extern TSPEC * mt_get_time (ENC * enc);
extern int mt_tticks (MOTOR *, int);
extern int mt_enc_count (MOTOR *);
extern void mt_calibrate(int samples, double wait_between_samples);
extern void mt_get_params(MOTOR *, int, int *, int *);
extern void mt_lock(MOTOR * );
extern void mt_unlock(MOTOR * );
extern int mt_move_sinc(char *, int);
extern int mt_move_sinc_t (char *,int,int,double);
extern void mt_shutdown(void);








