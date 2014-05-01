#include "lego_motor.h"

INIT status;

TSPEC t11, t12, t21, t22;

MOTOR motor1, motor2;

MOTOR * m1 = &motor1;
MOTOR * m2 = &motor2;

ENC * e11, * e12, * e21, * e22;

double *acum1, *acum2, *acum3, *acum4;

static uint8_t motor_busy = 0;

double pcoef_def[MAX_COEF] = { 5837.39694089577824342995882034301757812, 4437.29763943063881015405058860778808594, 3737.34827215639961650595068931579589844, 3282.11326188881457710522226989269256592, 2930.39274356875421290169470012187957764, 2626.36729752681549143744632601737976074, 2396.56913931981807763804681599140167236, 2170.58717529550403924076817929744720459, 1969.20963387265896926692221313714981079, 1822.45602936872319332906045019626617432, 1695.73273339300158113474026322364807129, 1569.82084063158617937006056308746337891, 1473.14643636506161783472634851932525635, 1383.18907963632477731152903288602828979, 1306.10837623716815869556739926338195801, 1222.95649083055104711093008518218994141, 1170.99432907457048713695257902145385742, 1094.90766701259735782514326274394989014, 1044.28679423828430117282550781965255737, 1013.68260002433055433357367292046546936, 0.0 };

double dcoef_def[MAX_COEF] = { 1135.86026750179712507815565913915634155, 869.34624357773122937942389398813247681, 764.23768186493850862461840733885765076, 718.21105387992736268643056973814964294, 640.35936035138365696184337139129638672, 611.99801209258998824225272983312606812, 547.67186454544810203515226021409034729, 517.43259316320211382844718173146247864, 478.25867786479875576333142817020416260, 458.86468723775709577239467762410640717, 430.54190130685094572982052341103553772, 406.98481717558024683967232704162597656, 395.62010568577761659980751574039459229, 375.74271902113002852274803444743156433, 361.38899585646299783547874540090560913, 352.61160524510603408998576924204826355, 354.93183388013255807891255244612693787, 338.21632822832532383472425863146781921, 329.61436368804623953110422007739543915, 322.95587145004765261546708643436431885, 0.0 };

/*struct thread_element {
	pthread_t id;
	bool alive;
};
typedef struct thread_element THREAD;*/


struct tharg_mtt {      /*Estructura per cridar al thread que moura el motor fins a un punt concret, evitant que el programa es quedi bloquejat*/
  MOTOR * mot;
  int ticks;
  dir dir;
  int vel;
  double pctr;
};
typedef struct tharg_mtt THARG_MTT;

struct tharg_mv {       /*Estructura per cridar al thread que moura el motor fins a un punt concret, evitant que el programa es quedi bloquejat*/
  MOTOR * mot;
  int vel;
  dir dir;
};
typedef struct tharg_mv THARG_MV;

struct tharg_mcal {      /*Estructura per cridar als threads que calibraran els motors, evitant que el programa es quedi bloquejat*/
  MOTOR * mot;
  int ms;
  int wait;
};
typedef struct tharg_mcal THARG_MCAL;

struct stats {			/* Estructura per calibracio de motors */
  double mean;
  double absd;
};

typedef struct stats STAT;

struct sinc {			/* structura para sincro mototres	*/
  bool acting;
  int t1;
  int t2;
  double d1;
  double d2;
  int id1;
  int id2;
  bool flag1;
  bool flag2;
  bool s1;
  bool s2;
  int first;
  bool set1;
  bool set2;
  bool mkrset1;
  bool mkrset2;
  TSPEC tstamp;
  
  /* solo para sincro + limite de tics*/
  
  bool fin1;
  bool fin2;
};
typedef struct sinc SINC;

SINC mtr_sincro;				//structura global usada para sincronizacion de motores;

SINC * msinc = &mtr_sincro;

struct posCtrl_sinc {			/* structura para sincro mototres + control posicion */
  bool arr1;
  bool arr2;
  int newmin;
  int newbase;
  int newmax;
  int sp;
  int first;
  bool changes;
};
typedef struct posCtrl_sinc PSINC;

PSINC mtr_sincro_posCtrl;

PSINC * mpsinc = &mtr_sincro_posCtrl;

//THREAD threads[MAX_THREADS];


static bool mbusy(int port);
static int set_pulse(int, int, int);
static void * mtt_thread (void * arg);
static int conf_motor(MOTOR *, ENC *, ENC *);
static void reset_encoder(ENC * enc);
static bool c_enc(MOTOR *, ENC *, int);
static int mode(int);
//static void prstat(STAT [], STAT [], int); TESTING
static void get_stats(STAT *, MOTOR *, int);
static void init_acums(int, MOTOR *);
static void reset_acums(int, MOTOR *);
static void free_acums(MOTOR *);
static int get_len(double []);
static void get_vels (int , double [], int);
static void cal_weight(int, double [], double, double);
static double avg(int, double[]);
static void cmp_res(PID *, STAT * [], STAT * [], int);
//static int table_tam(int); DEPRECATED
//static void prod(mpf_t, int, int [], int); DEPRECATED
//long long prod(int, int [], int); DEPRECATED
static void mot_cal(MOTOR *, int, double);
static void * mcal_thread(void * arg);
//static void setup_sighandlers(void);
//static void terminate(void);
//static void cal_coef(double[], double[], int, int); DEPRECATED
static void mpid_load_coef(PID *, double *, double *, int);
static void prac(int, double *);
//static void prai (int, int *);
static void pid_launch(MOTOR *, int, int, dir, double, bool);
//static int get_MV(MOTOR *, int);
static void * mv_thread(void *);
static int mot_fwd(MOTOR *, int);
static int mot_rev(MOTOR *, int);
//static int mot_unexportall(void); >> SHARED
//static void init_threads(void);
//static double difft(TSPEC *, TSPEC *); >> PUBLICA
static int destroy_mutex(void);
static int reset_pulse (MOTOR * m, long long usPidOut, int act_pw, int gpio, int maxpw, int minpw, int basepw, int PHbkup);
static int us_to_vel(int us, MOTOR *);
static long long get_MVac(MOTOR * m, long long*, int *, int , int , int ,int);
//static int get_ticksdt(MOTOR * m);
static bool start_pwm();
static int move_till_ticks_b (MOTOR *, int, dir, int , bool, double);
static int get_pid_new_vals(MOTOR *, int, int * , int * , int * , double, int);
static int get_sincro_new_vals(MOTOR *, int, int *, int *, int *, int, int, int, int, double);

//_______________________________Internally used____________________________________________________//

static int mot_stop(MOTOR * mot, bool reset);
static bool is_null (ENC * enc);
static bool move (MOTOR * m, dir dir, int vel);
static bool move_t (MOTOR * mot, int ticks, dir dir, int vel, double posCtrl);
static bool mot_lock (MOTOR * m);
static bool mot_unlock (MOTOR * m);
static bool reset_encs (MOTOR * mot);
static bool wait_for_stop(MOTOR * m, double diff);
static bool get_params (MOTOR * m, int vel, int * ex_micras, int * ex_desv);
static bool mot_reconf(MOTOR * m, ENC * e1, ENC * e2);
static int get_ticks (MOTOR * mot);
static int tticks (MOTOR * m, int turns);
static int ecount (MOTOR * m);
static bool m_wait(MOTOR * m);
static int pid_conf(MOTOR * m, double * pcoef, double * dcoef);
static void pid_on (PID * pid);
static void pid_off (PID * pid);
static void pid_set_gains (PID * pid, double kp, double ki, double kd);
static bool pid_is_null (PID * pid);

//static int nextpw (int, int, int, int);
//static void handl_alrm(void);

static void isr_cal_11(void){
  if(m1->moving){
    clock_gettime(CLK_ID, &t11);
    acum1[e11->tics] = difft(&e11->tmp, &t11); //e11->tics == 0 ? (double)((e11->tmp.tv_sec * 1000000) + (e11->tmp.tv_nsec/1000)) : (e11->delay); /* NANOS TO MICR0S */
    pthread_mutex_lock(&e11->mtx);
    e11->tics++;
    pthread_mutex_unlock(&e11->mtx);
    clock_gettime(CLK_ID, &e11->tmp);
  }
}

static void isr_cal_12(void){
  if(m1->moving){
    clock_gettime(CLK_ID, &t12);
    acum3[e12->tics] = difft(&e12->tmp, &t12);//e12->tics == 0 ? (double)((e12->tmp.tv_sec * 1000000) + (e12->tmp.tv_nsec/1000)) : (e12->delay); /* NANOS TO MICR0S */
    pthread_mutex_lock(&e12->mtx);
    e12->tics++;
    pthread_mutex_unlock(&e12->mtx);
    clock_gettime(CLK_ID, &e12->tmp);
  }
}

static void isr_cal_21(void){
  if(m2->moving){
    clock_gettime(CLK_ID, &t21);
    acum2[e21->tics] = difft(&e21->tmp, &t21);//e21->tics == 0 ? (double)((e21->tmp.tv_sec * 1000000) + (e21->tmp.tv_nsec/1000)) : (e21->delay); /* NANOS TO MICR0S */
    pthread_mutex_lock(&e21->mtx);
    e21->tics++;
    pthread_mutex_unlock(&e21->mtx);
    clock_gettime(CLK_ID, &e21->tmp);
  }
}

static void isr_cal_22(void){
  if(m2->moving){
    clock_gettime(CLK_ID, &t22);
    acum4[e22->tics] = difft(&e22->tmp, &t22);//e22->tics == 0 ? (double)((e22->tmp.tv_sec * 1000000) + (e22->tmp.tv_nsec/1000)) : (e22->delay); /* NANOS TO MICR0S */
    pthread_mutex_lock(&e22->mtx);
    e22->tics++;
    pthread_mutex_unlock(&e22->mtx);
    clock_gettime(CLK_ID, &e22->tmp);
  }
}

static void isr_def_11(void){
  if(m1->moving){
    pthread_mutex_lock(&e11->mtx);
    e11->tics++;
    pthread_mutex_unlock(&e11->mtx);
  }
  
}

static void isr_def_12(void){
  if(m1->moving){
    pthread_mutex_lock(&e12->mtx);
    e12->tics++;
    pthread_mutex_unlock(&e12->mtx);
  }
}

static void isr_def_21(void){
  if(m2->moving){
    pthread_mutex_lock(&e21->mtx);
    e21->tics++;
    pthread_mutex_unlock(&e21->mtx);
  }
}

static void isr_def_22(void){
  if(m2->moving){
    pthread_mutex_lock(&e22->mtx);
    e22->tics++;
    pthread_mutex_unlock(&e22->mtx);
  }
}

//____________________________________________Internally used externs_____________________________________________________//

extern int mt_stop(MOTOR * m, bool reset){
  if(!status.mt){
    not_critical("mt_stop: Motor interface not initialised.\n");
    return FAIL;
  } else if (!mbusy(m->id)) {
    not_critical("mt_stop: Motor %d not initialised properly.\n", m->id);
    return FAIL;
  } else
    return (mot_stop(m, reset));

}

extern int mt_enc_is_null (MOTOR * m, int eid) {
  if(!status.mt){
    not_critical("mt_enc_is_null: Motor interface not initialised.\n");
    return FAIL;
  } else if (!mbusy(m->id)) {
    not_critical("mt_enc_is_null: Motor %d not initialised properly.\n", m->id);
    return FAIL;
  } else
    return(is_null(eid == 1 ? m->enc1 : m->enc2));
}

extern bool mt_move (MOTOR * m, dir dir, int vel){
  if(!status.mt){
    not_critical("mt_move: Motor interface not initialised.\n");
    return false;
  } else if (!mbusy(m->id)) {
    not_critical("mt_move: Motor %d not initialised properly.\n", m->id);
    return false;
  } else 
    return (move(m, dir, vel));
}

extern bool mt_get_params (MOTOR * m, int vel, int * ex_micras, int * ex_desv){
  return (get_params (m, vel, ex_micras, ex_desv));
}

extern bool mt_move_t (MOTOR * m, int ticks, dir dir, int vel, double posCtrl){
  if(!status.mt){
    not_critical("mt_move_t: Motor interface not initialised.\n");
    return false;
  } else if (!mbusy(m->id)) {
    not_critical("mt_move_t: Motor %d not initialised properly.\n", m->id);
    return false;
  } else if(ecount(m) == 0) {
    not_critical("mt_move_t: At least one encoder needed\n", m->id);
    return false;
  } else
    return(move_t(m, ticks, dir, vel, posCtrl));
}

extern bool mt_lock (MOTOR * m) {
  if(!status.mt){
    not_critical("mt_lock: Motor interface not initialised.\n");
    return false;
  } else if (!mbusy(m->id)) {
    not_critical("mt_lock: Motor %d not initialised properly.\n", m->id);
    return false;
  } else
    return(mot_lock(m));
}

extern bool mt_unlock (MOTOR * m) {
  if(!status.mt){
    not_critical("mt_unlock: Motor interface not initialised.\n");
    return false;
  } else if (!mbusy(m->id)) {
    not_critical("mt_unlock: Motor %d not initialised properly.\n", m->id);
    return false;
  } else
    return(mot_unlock(m));
}

extern bool mt_reset_enc (MOTOR * m) {

  if(!status.mt){
    not_critical("mt_reset_enc: Motor interface not initialised.\n");
    return false;
  } else if (!mbusy(m->id)) {
    not_critical("mt_reset_enc: Motor %d not initialised properly.\n", m->id);
    return false;
  } else
    return(reset_encs(m));
}

extern bool mt_wait_for_stop (MOTOR * m, double diff) {
  
  if(!status.mt){
    not_critical("mt_wait_for_stop: Motor interface not initialised.\n");
    return false;
  } else if (!mbusy(m->id)) {
    not_critical("mt_wait_for_stop: Motor %d not initialised properly.\n", m->id);
    return false;
  } else
    return(wait_for_stop(m, diff));
}

extern bool mt_reconf(MOTOR * m, ENC * e1, ENC * e2) {
 
  if(!status.mt){
    not_critical("mt_reconf: Motor interface not initialised.\n");
    return false;
  } else if (!mbusy(m->id)) {
    not_critical("mt_reconf: Motor %d not initialised properly.\n", m->id);
    return false;
  } else
    return(mot_reconf(m, e1, e2));
}

extern int mt_get_ticks (MOTOR * m) {
  return(get_ticks(m));
}

extern int mt_tticks (MOTOR * m, int turns) {
  return(tticks(m, turns));
}

extern int mt_enc_count (MOTOR * m){ 
  return (ecount(m));
}

extern bool mt_wait(MOTOR * m) {
  return (m_wait(m));
}

extern bool mt_pid_is_null (PID * pid){

  return (pid->active);

}

extern void mt_pid_off (PID * pid){

  pid->active = false;
  
}

extern void mt_pid_on (PID * pid){

  pid->active = true;
  
}

extern void mt_pid_set_gains (PID * pid, double kp, double ki, double kd){
  if(kp == 0 && ki == 0 &&  kd == 0)
    not_critical("mt_pid_set_gains: seting gains to 0, PID won't act.\n");
  return(pid_set_gains (pid, kp, ki, kd));
}

extern int mt_pid_conf(MOTOR * m, double * pcoef, double * dcoef) {
  return (pid_conf(m, pcoef, dcoef));
}


extern bool mt_init(){
  
  if(!status.wpi)
    status.wpi = wiringPiSetupGpio() == 0;

  msinc->acting = false;
  status.mt = start_pwm();
  setup_sighandlers();
  m1->id = 0;
  m2->id = 0;
  return status.mt;
  
}

static bool mbusy(int port) {

  return (motor_busy & 1 << port);

}

extern bool mt_new ( MOTOR * m, ENC * e1, ENC * e2, int port ){
  
  
  if (port < MIN_PORT_MT && port > MAX_PORT_MT) {
    not_critical("mt_new: id \"%d\" out of range, must be between \"%d\" and \"%d\".\n", port, MIN_PORT_MT, MAX_PORT_MT);
    return false;
  } else if(mbusy(port)) {
    not_critical("mt_new: Motor %d busy\n", m->id);
    return false;
  } 

  MOTOR * maux;
  maux = port == 1 ? m1 : m2;
  maux->id = port;
  maux->enc1 = (ENC *)malloc(sizeof(ENC));
  maux->enc2 = (ENC *)malloc(sizeof(ENC));
  maux->pid  = (PID *)malloc(sizeof(PID));
  
  if (port == 1) {
    //m1 = m;
    e11 = maux->enc1;
    e12 = maux->enc2;
  } else {
    //m2 = m;
    e21 = maux->enc1;
    e22 = maux->enc2;
  }
  
  if (!conf_motor(maux,e1,e2)){
    not_critical("mt_new: Error configuring motor %d\n", port);
    maux->id = 0;
    return false;
    
  } 
  
  motor_busy |= 1 << port;
  m = maux;
  
  
  return true;
  
}

static bool start_pwm(){

  spwm_set_loglevel(status.pr_debug ? LOG_LEVEL_DEBUG : LOG_LEVEL_ERRORS);
  return ((spwm_setup(PWIG_DEF, HW_PWM) == 0) ? true : false);

}

static int conf_motor(MOTOR * mot, ENC * enc1, ENC * enc2){   //ALLOCATE accelerador interpolació per aki...

  int cenc = 0, ret, res_pwm_init;
  int ex_pin1 = mot->id == 1 ? M1_ENC1 : M2_ENC1;
  int ex_pin2 = mot->id == 1 ? M1_ENC2 : M2_ENC2;
  int ex_pinf = mot->id == 1 ? M1_PINF : M2_PINF;
  int ex_pinr = mot->id == 1 ? M1_PINR : M2_PINR;
  mot->chann = mot->id == 1  ? M1_CHANN : M2_CHANN;
  
  mot->pinf = ex_pinf;
  mot->pinr = ex_pinr;
  pinMode(mot->pinf, OUTPUT);
  pinMode(mot->pinr, OUTPUT);
  
  pthread_mutex_init(&mot->enc1->mtx, NULL);
  pthread_mutex_init(&mot->enc2->mtx, NULL);
  
  /* a pid_conf resetejo l'accelerador, per tant aki he d'allocatar-lo */
  mot->pid->accelM = gsl_interp_accel_alloc();
  mot->pid->accelD = gsl_interp_accel_alloc();
  ret = pid_conf(mot, pcoef_def, dcoef_def);
  pid_on(mot->pid);
  
  if(!ret)
    not_critical("conf_motor: Error configuring PID on motor %d\n", mot->id);
  
  res_pwm_init = spwm_init_channel(mot->chann, ST_US) == 0 ? OK : FAIL;
  
  if(!res_pwm_init)
    not_critical("conf_motor: Error initializing PWM on motor %d\n", mot->id);
  
  if(enc1!= NULL){
    cenc += (c_enc(mot, enc1, 1) && (mot->enc1->pin != ENULL)) ? 1 : 0;
    
  } else {
    ENC aux;
    aux.pin = ex_pin1;
    aux.isr = mot->id == 1 ? &isr_def_11 : &isr_def_21;
    cenc += c_enc(mot, &aux, 1) ? 1 : 0;
  }

  if(enc2 != NULL){
    cenc += (c_enc(mot, enc2, 2) && (mot->enc2->pin != ENULL)) ? 1 : 0;
  } else { /* defaults */
    ENC aux;
    aux.pin = ex_pin2;
    aux.isr = mot->id == 1 ? &isr_def_12 : &isr_def_22;
    cenc += c_enc(mot, &aux, 2) ? 1 : 0;
  }
  
  mot->ticsxturn = (cenc == 2) ? 720 : (cenc == 1) ? 360 : FAIL;
  
  if (mot->ticsxturn == FAIL)
    not_critical("conf_motor: Configuration let motor %d without encoders", mot->id);
  
  return (ret && res_pwm_init);
}

static bool mot_reconf(MOTOR * m, ENC * e1, ENC * e2){
  
  bool ret;

  if (e1 != NULL && e2 != NULL){
    /* load args */
    ret = ( (c_enc(m,e1,1) && c_enc(m,e2,2)) );
  } else if (e1 == NULL && e2 != NULL){
    /* reconf e2 */
    ret = c_enc(m,e2,2);
  } else if (e1 != NULL && e2 == NULL){
    /* reconf e1*/
    ret = c_enc(m,e1,1);
  } else { /* 2 nuls */
    /* reload defaults */
    m->enc1->pin = m->id == 1 ? M1_ENC1 : M2_ENC1;
    m->enc2->pin = m->id == 1 ? M1_ENC2 : M2_ENC2;
    m->enc1->isr = m->id == 1 ? &isr_def_11 : &isr_def_21;
    m->enc2->isr = m->id == 1 ? &isr_def_12 : &isr_def_22;
    ret = ( (c_enc(m, m->enc1,1) && c_enc(m,m->enc2,2)) );
  }

  if(ret){
    if(ecount(m) == 0){
      not_critical("mt_reconf: Configuration let motor %d without encoders\n", m->id);
      m->ticsxturn = 0;
    } else 
      m->ticsxturn = ecount(m) == 1 ? 360 : 720;
  }

  return ret;
}

static int pid_conf(MOTOR * m, double * pcoef, double * dcoef){ 

  int sizep,sized;

  for (sizep = 0; pcoef[sizep] != 0; sizep++);
  for (sized = 0; dcoef[sized] != 0; sized++);
 
  if (((pcoef == NULL) && (dcoef == NULL)) || ((sizep == sized) == 0))
    pid_off(m->pid);
  else if((pcoef != NULL) && (dcoef != NULL)){
   if(sized == sizep)
     m->pid->svel = (int)(MAX_VEL/sized);
   else {
     not_critical("Incoherent table sizes specified, %d\n", WEIRD);
     return FAIL;
   }
   
   mpid_load_coef(m->pid, pcoef, dcoef, sizep);
   pid_set_gains(m->pid, PIDDEF, PIDDEF, PIDDEF);
   m->pid->ttc = TTCDEF;
  }
  else {
    not_critical("Bad coeficients specified\n");
    return FAIL;
 }
  
  return OK;
}

static void mpid_load_coef(PID * pid, double * pcoef, double * dcoef, int len){

  int i;
  
  for(i = 0; i <len ; i++ ){
    pid->cp[i] = pcoef[i];
    pid->cd[i] = dcoef[i];
  }
  
  gsl_interp_accel_reset(pid->accelM);
  gsl_interp_accel_reset(pid->accelD);

}

static bool c_enc(MOTOR * m, ENC * e, int id){

  ENC * ex_en = id == 1 ? m->enc1 : m->enc2;
  int ex_pin = m->id == 1 ? id == 1 ? M1_ENC1 : M1_ENC2 : id == 1 ? M2_ENC1 : M2_ENC2;
  int md = mode(e->pin);
  
  if(e->pin == ex_pin){ /* Configure a licit interrupt */
    ex_en->pin = ex_pin;
    ex_en->isr = e->isr;
    if (wiringPiISR (ex_en->pin, md, ex_en->isr) < 0){
      not_critical("c_enc: Failed to config ISR on motor: %d, encoder: %d\n", m->id, id);
      return false;
    }
    clock_gettime(CLK_ID, &ex_en->tmp);
  } else {
    if (md == SETUP){
      if (wiringPiISR (ex_en->pin, md, NULL) < 0){ 
	/* If we already set an interrupt before, and we want to disable it */
	not_critical("c_enc: Failed to config ISR on motor: %d, encoder: %d\n", m->id, id);
	return false;
      }
    } else /* We want to disable encoder and never was enabled before */
      pinMode(ex_en->pin, INPUT);
    
    ex_en->pin = ENULL;
    ex_en->isr = NULL;
  }
  
  ex_en->tics = 0;
  return true;
}

static int mode (int pin){

  struct stat st;
  char gpio_rec [128];
  
  sprintf (gpio_rec, "/sys/class/gpio/gpio%d/edge", pin) ;
  if (stat(gpio_rec,&st) < 0)
    return RISING;
  else
    return SETUP;
}

static int set_pulse (int vel, int gpio, int chann){
  
  if (vel < 0 || vel > MAX_VEL)
    fatal("set_pulse: Velocity must be an integer between 0 - %d\n", MAX_VEL);
  
  if (vel == MAX_VEL)
    digitalWrite(gpio, HIGH);
  else if(vel != 0)
    return((spwm_add_channel_pulse(chann, gpio, 0, V2PW(vel) ==  MAX_PW ? MAX_PW -1 : V2PW(vel)) == 0) ? OK : FAIL);
  else {
    digitalWrite(gpio, LOW);
    not_critical("set_pulse: Motor configured as \"in motion\", but velocity is 0 \n");
  }
  
  return OK;
}

static bool is_null (ENC * enc) {
   return(enc->pin == ENULL);
}

static int mot_stop(MOTOR * mot, bool reset){
  
  mot->moving = false;
  int ticks;

  if(spwm_clear_channel_gpio(mot->chann,mot->pinf) != 0)
    not_critical("mot_stop: Error clearing DMA channel: %d, on motor: %d", mot->chann, mot->id);

  if(spwm_clear_channel_gpio(mot->chann,mot->pinr) != 0)
    not_critical("mot_stop: Error clearing DMA channel: %d, on motor: %d", mot->chann, mot->id);
  
  /*if (spwm_clear_channel(mot->chann) != 0)
    not_critical("mot_stop: Error clearing DMA channel: %d, on motor: %d", mot->chann, mot->id);*/

  digitalWrite(mot->pinf, LOW);
  digitalWrite(mot->pinr, LOW);
  ticks = get_ticks(mot);
  
  if(reset)
    reset_encs(mot);
  
  msinc->acting = false;
  return ticks;
}

static void reset_encoder(ENC * enc){

  pthread_mutex_lock(&enc->mtx);
  enc->tics = 0;
  pthread_mutex_unlock(&enc->mtx);

}

static int mot_fwd(MOTOR * mot, int vel){

  //PID auxpid; //save pid state.
  bool altered = false;

  if(vel < MIN_VEL && !mt_pid_is_null(mot->pid)) {
    not_critical("mot_fwd: Setting PID null, the minimun velocity allowed with PID control is %d\n", MIN_VEL);
    //mt_pid_set_gains(&auxpid, mot->pid->kp, mot->pid->ki, mot->pid->kd);
    pid_off(mot->pid);
    altered = true;
  }

  if (!is_null(mot->enc1)){
    reset_encoder(mot->enc1);
    clock_gettime(CLK_ID, &mot->enc1->tmp);
  }
  if (!is_null(mot->enc2)){
    reset_encoder(mot->enc2);
    clock_gettime(CLK_ID, &mot->enc2->tmp);
  }
  
  digitalWrite(mot->pinr, LOW);
  if(set_pulse(vel,mot->pinf,mot->chann) == FAIL) {
    not_critical("mot_fwd: Failed to configure PWM on motor: %d, GPIO: %d", mot->id, mot->pinf);
    return FAIL;
  }
  
  mot->moving = true;

  if (!pid_is_null(mot->pid))
    pid_launch(mot, vel, 0, FWD, 0, msinc->acting);
  
  if(altered)
    pid_on(mot->pid);

  return OK;
}


static bool pid_is_null (PID * pid){

  return (!pid->active);

}

static void pid_off (PID * pid){

  pid->active = false;
  
}

static void pid_on (PID * pid){

  pid->active = true;
  
}

static void pid_set_gains (PID * pid, double kp, double ki, double kd){

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  
}


static int mot_rev (MOTOR * mot, int vel){
  
  //PID auxpid; //save pid state.
  bool altered;

  if(vel < MIN_VEL && !pid_is_null(mot->pid)) {
    not_critical("mot_rev: Setting PID null, the minimun velocity allowed with PID control is %d\n", MIN_VEL);
    //mt_pid_set_gains(&auxpid, mot->pid->kp, mot->pid->ki, mot->pid->kd);
    pid_off(mot->pid);
    altered = true;
  }

  if (!is_null(mot->enc1)){
    reset_encoder(mot->enc1);
    clock_gettime(CLK_ID, &mot->enc1->tmp);
  }
  
  if (!is_null(mot->enc2)){
    reset_encoder(mot->enc2);
    clock_gettime(CLK_ID, &mot->enc2->tmp);
  }
  
  digitalWrite(mot->pinf, LOW);
  if(set_pulse(vel,mot->pinr,mot->chann) == FAIL) {
    not_critical("mot_rev: Failed to configure PWM on motor: %d, GPIO: %d", mot->id, mot->pinr); 
    return FAIL;
  }

  mot->moving = true;

  if (!pid_is_null(mot->pid))
    pid_launch(mot, vel, 0, BWD, 0, msinc->acting);

  if(altered)
    pid_on(mot->pid);

  return OK;
}

static bool move (MOTOR * m, dir dir, int vel){

  THARG_MV * args = (THARG_MV *)malloc(sizeof(THARG_MV));
  pthread_t worker; 
  pthread_attr_t tattr;
  
  if(m->id != 0) {
    if(!m->moving) {
      if(!pid_is_null(m->pid)) {
	
	args->mot = m;
	args->vel = vel;
	args->dir = dir;
	
	pthread_attr_init(&tattr);
	pthread_attr_setdetachstate(&tattr,PTHREAD_CREATE_DETACHED);
	if(pthread_create(&worker, &tattr, &mv_thread, args)!=0) {
	  not_critical("move: Error creating thread.\n");
	  return false;
	}
	pthread_attr_destroy(&tattr);
	
      } else {
	if (dir == FWD){
	  if(!mot_fwd(m,vel)) {
	    not_critical("move: Motor %d, Error in motor_fwd\n", m->id);
	    return false;
	  }
	} else if (dir == BWD){
	  if(!mot_rev(m,vel)) {
	    not_critical("move: Motor %d, Error in motor_rev\n", m->id);
	    return false;
	  }
	} else { //
	  not_critical("move: Motor %d, direction \"%s\" not understood \n", m->id, dir);
	  return false;
	}
      }
    } else {
      not_critical("move: Motor %d already moving, stop it first\n", m->id);
      return false;
    }
  } else {
    not_critical("move: Motor not properly initialised\n");
    return false;
  }
  
  return true;
}

static void * mv_thread (void * arg){
  
  THARG_MV * args = (THARG_MV *) arg;
  
  if (args->dir == FWD){
    if(!mot_fwd(args->mot,args->vel))
      not_critical("move-thread: Motor %d, Error in motor_fwd\n", args->mot->id);
  } else if (args->dir == BWD){
    if(!mot_rev(args->mot,args->vel))
      not_critical("move-thread: Motor %d, Error in motor_rev\n", args->mot->id);
  } else {
    not_critical("move-thread: Motor %d, direction \"%s\" not understood \n", args->mot->id, args->dir);
  }
  
  pthread_exit(NULL);
}


static int move_till_ticks_b (MOTOR * mot, int ticks, dir dir, int vel, bool reset, double posCtrl){
  
  //printf("entro con pid -> kp : %f, ki: %f, kd: %f\n", mot->pid->kp, mot->pid->ki, mot->pid->kd);

  if(mot->id != 0) {
    if(ecount(mot) != 0) {
      bool initial_pid = mot->pid->active;//save pid state.
      bool restored = false;

      //mt_pid_set_gains(&auxpid, mot->pid->kp, mot->pid->ki, mot->pid->kd);
      //printf("auxpid -> kp : %f, ki: %f, kd: %f\n", auxpid.kp, auxpid.ki, auxpid.kd);
      pid_off(mot->pid); //force pid null since we will launch it later on.
      

      if (dir == FWD){
	if(!mot_fwd(mot, vel)) {
	  not_critical("move_till_ticks: Motor %d, Error in motor_fwd\n", mot->id);
	  return FAIL;
	}
      } else if (dir == BWD){
	if(!mot_rev(mot, vel)) {
	  not_critical("move_till_ticks: Motor %d, Error in motor_rev\n", mot->id);
	  return FAIL;
	}
      } else {
	not_critical("move_till_ticks: Motor %d, Direction \"%s\" not understood \n", mot->id, dir);
	return FAIL;
      }
      
      if(vel < MIN_VEL && initial_pid) 
	not_critical("move_till_ticks: Setting PID null, the minimun velocity allowed with PID control is %d\n", MIN_VEL);
      else if (initial_pid) {
	pid_on(mot->pid);
	restored = true;
      }
      //printf("despues de reset pid -> kp : %f, ki: %f, kd: %f\n", mot->pid->kp, mot->pid->ki, mot->pid->kd);
      
      if(pid_is_null(mot->pid)) {
	while(get_ticks(mot) < ticks);
      } else {
	pid_launch(mot, vel, ticks, dir, posCtrl, msinc->acting);
      }
      if (!restored && initial_pid)
	pid_on(mot->pid);
      
      return (mot_stop(mot, reset));
      
    } else {
      not_critical("move_till_ticks: At least one encoder needed to move till ticks \n");
      return FAIL;
    }
  } else {
    not_critical("move_till_ticks: Motor not properly initialised \n");
    return FAIL;
  }
}

extern void mt_shutdown () { //aki "free" d'accelerador de interpolacions 
  
  spwm_shutdown();
  if(!destroy_mutex())
    not_critical("mot_shutdown: Error destroying mutexs\n");
  if(!status.ag)
    unexportall();
  if(m1->id !=0){
    gsl_interp_accel_free(m1->pid->accelM);
    gsl_interp_accel_free(m1->pid->accelD);
  }
  
  if(m2->id !=0){
    gsl_interp_accel_free(m2->pid->accelM);
    gsl_interp_accel_free(m2->pid->accelD);
  }
  
  status.mt = false;
}

static int destroy_mutex (void){

  MOTOR * m;
  int i = 1, ret = OK;

  for(; i <= 2; i++){
    m = i == 1 ? m1 : m2;
    if(m->id != 0){
      ret = ret ? ( pthread_mutex_destroy(&m->enc1->mtx) != 0 ) ? FAIL : OK : FAIL;
      ret = ret ? ( pthread_mutex_destroy(&m->enc2->mtx) != 0 ) ? FAIL : OK : FAIL;
    }
  }

  return ret;
}

static bool m_wait(MOTOR * m){
  if(m->id !=0) {
    while (m->moving)
      udelay(2500);
    return true;
  } else {
    not_critical("mt_wait: Motor not properly initialised.\n");
    return false;
  }
}

extern bool mt_wait_all(){
  
  if (m1->id == 0)
    return (m_wait(m2));
  else if(m2->id == 0)
    return (m_wait(m1));
  else {
    while (m1->moving || m2->moving)
      udelay (2500);
    return true;
  }
}

static bool move_t (MOTOR * mot, int ticks, dir dir, int vel, double posCtrl){

  THARG_MTT * arg = (THARG_MTT *)malloc(sizeof(THARG_MTT));
  pthread_t worker; //= &threads[0].id; //pos 1 per mtt
  pthread_attr_t tattr;
  bool ret = true;
  
  if (!mot->moving){
    
    arg->mot = mot;
    arg->ticks = ticks;
    arg->dir = dir;
    arg->vel = vel;
    arg->pctr = posCtrl;
    mot->moving = true;
    /* Abstraer en función para hacer threads*/
    pthread_attr_init(&tattr);
    pthread_attr_setdetachstate(&tattr,PTHREAD_CREATE_DETACHED);
    pthread_attr_setscope(&tattr, PTHREAD_SCOPE_SYSTEM);
    ret = pthread_create(&worker, &tattr, &mtt_thread, arg) == 0 ? true : false;
    pthread_attr_destroy(&tattr);

  } else {

    not_critical("move_t: Motor %d already moving, stop it first\n", mot->id);
    return false;
    
  }

  //threads[0].alive = ret ? true : false;
  return ret;
}

static void * mtt_thread (void * arg){

  THARG_MTT * args = (THARG_MTT *) arg;
  
  move_till_ticks_b(args->mot, args->ticks, args->dir, args->vel, false, args->pctr); //force reset = false
  //return ((void *) res);
  pthread_exit(NULL);
}


static bool wait_for_stop(MOTOR * m, double diff){   //pensar con difft

  //double diff -> segundos con decimales....
  
  TSPEC taux;
  TSPEC * tini1 = &m->enc1->tmp;
  TSPEC * tini2 = &m->enc2->tmp;
    
  long enano1, enano2;
  int esec1, esec2;
  double el1, el2;
  
  if ((!is_null(m->enc1)) && (!is_null(m->enc2))){
    do {
      clock_gettime(CLK_ID, &taux);
      enano1 = (taux.tv_nsec - tini1->tv_nsec);
      esec1 = (int)(taux.tv_sec - tini1->tv_sec);
      el1 = esec1+(enano1*0.000000001);
      enano2 = (taux.tv_nsec - tini2->tv_nsec);
      esec2 = (int)(taux.tv_sec - tini2->tv_sec);
      el2 = esec2+(enano2*0.000000001);
    } while ( (el1 < diff) || (el2 < diff) );
    
  } else if (is_null(m->enc1) && !is_null(m->enc2)) {
    do {
      clock_gettime(CLK_ID, &taux);
      enano2 = (taux.tv_nsec - tini2->tv_nsec);
      esec2 = (int)(taux.tv_sec - tini2->tv_sec);
      el2 = esec2+(enano2*0.000000001);
    } while (el2 < diff);
    
  } else if (!is_null(m->enc1) && is_null(m->enc2) ){
    do {
      clock_gettime(CLK_ID, &taux);
      enano1 = (taux.tv_nsec - tini1->tv_nsec);
      esec1 = (int)(taux.tv_sec - tini1->tv_sec);
      el1 = esec1+(enano1*0.000000001);
    } while (el1 < diff);
  } else {
    not_critical("wait_for_stop: At least one encoder needed.\n");
    return false;
  }
  
  return true;
  
}

static bool reset_encs (MOTOR * mot){

  if(!is_null(mot->enc1))
    reset_encoder(mot->enc1);
  if(!is_null(mot->enc2))
    reset_encoder(mot->enc2);
  return true;
}

static int get_ticks (MOTOR * mot) {

  int ticks = 0;

  if(mot -> id != 0) {
    if (is_null(mot->enc1)){
      pthread_mutex_lock(&mot->enc2->mtx);
      ticks = mot->enc2->tics;
      pthread_mutex_unlock(&mot->enc2->mtx);
    } else if (is_null(mot->enc2)){
      pthread_mutex_lock(&mot->enc1->mtx);
      ticks = mot->enc1->tics;
      pthread_mutex_unlock(&mot->enc1->mtx);
    } else {
      mot_lock(mot);
      ticks = (mot->enc1->tics + mot->enc2->tics);
      mot_unlock(mot);
    }
  
  } else 
    not_critical("get_ticks: Motor not properly initialised.\n");

  //printf("salgo de get_ticks: %d ...\n", ticks);

  return ticks;
}

extern TSPEC * mt_get_time (ENC * enc) {
  
  if (!is_null(enc))
    return(&enc->tmp);
  else
    return NULL;
}

static int tticks (MOTOR * m, int turns){
  
  if(m->id != 0) 
    return(m->ticsxturn * turns);
  else {
    not_critical("tticks: Motor not properly initialised.\n");
    return 0;
  }

}

static int ecount (MOTOR * m){
  
  if(m->id != 0)
    return (!is_null(m->enc1) + !is_null(m->enc2));
  else {
    not_critical("ecount: Motor not properly initialised.\n");
    return 0;
  }
}

extern bool mt_calibrate (int mostres, double wait){

  
  THARG_MCAL * arg1 = (THARG_MCAL *)malloc(sizeof(THARG_MCAL));
  THARG_MCAL * arg2 = (THARG_MCAL *)malloc(sizeof(THARG_MCAL));
  pthread_t calib1, calib2;

  if(m1->id != 0 && m2->id != 0) {
    if(!m1->moving && !m2->moving) {
      arg1->mot = m1;
      arg1->ms = mostres;
      arg1->wait = wait;
      arg2->mot = m2;
      arg2->ms = mostres;
      arg2->wait = wait;
      
      pthread_create(&calib1, NULL, &mcal_thread, arg1);
      pthread_create(&calib2, NULL, &mcal_thread, arg2);
      
      pthread_join(calib1, NULL);
      pthread_join(calib2, NULL);
      
      free_acums(m1);
      free_acums(m2);
      
      return true;

    } else { 
      not_critical("cal_motors: Motors already moving, stop it first\n");
      return false;
    }
  } else if(m1->id != 0 && m2->id == 0) {
    if(!m1->moving) {
      
      mot_cal(m1, mostres, wait);
      free_acums(m1);
      
    } else {
      not_critical("cal_motors: Motor %d already moving, stop it first\n", m1->id);
      return false;
    }

  } else if(m1->id == 0 && m2->id != 0) {
    if(!m2->moving) {
      
      mot_cal(m2, mostres, wait);
      free_acums(m2);
      
    } else {
      not_critical("cal_motors: Motor %d already moving, stop it first\n", m1->id);
      return false;
    }

  } else {
    not_critical("cal_motors: Motors not properly initialised\n");
    return false;
  }
  
  return true;
}

static void * mcal_thread (void * arg){

  THARG_MCAL * args = (THARG_MCAL *) arg;
  
  mot_cal(args->mot, args->ms, args->wait);
  
  return ((void *) 1);
}


static void mot_cal (MOTOR * m, int mostres, double wait){
  
  int ms;
  ms = mostres > MAX_COEF-1 ? MAX_COEF-1 : mostres < MIN_COEF ? MIN_COEF : mostres;
  
  if(ms != mostres) 
    not_critical("mot_cal: Setting samples to %d, the %s allowed\n", MIN_COEF, ms < MIN_COEF ? "minimum" : "maximum");
  

  int turns = 8, vel, index = 0;
  int step = (MAX_VEL/ms);
  dir  dir = FWD;
  bool reset = true;
  
  STAT * es1[ms];// = (STAT *) malloc(ms * sizeof(STAT)); 
  STAT * es2[ms];// = (STAT *) malloc(ms * sizeof(STAT));
  int ex_pin1 = m->id ==1 ? M1_ENC1 : M2_ENC1;
  int ex_pin2 = m->id ==1 ? M1_ENC2 : M2_ENC2;
  ENC aux1, aux2; //save previous state;
  //PID auxpid;
  //mt_pid_set_gains(&auxpid, m->pid->kp, m->pid->ki, m->pid->kd);
  pid_off(m->pid); //force pid null since we will launch it later on.
  
  aux1.pin = m->enc1->pin;
  aux1.isr = m->enc1->isr;
  aux2.pin = m->enc2->pin;
  aux2.isr = m->enc2->isr;
  
  m->enc1->pin = ex_pin1;
  m->enc1->isr = m->id == 1 ? &isr_cal_11 : &isr_cal_21;
  m->enc2->pin = ex_pin2;
  m->enc2->isr = m->id == 1 ? &isr_cal_12 : &isr_cal_22;
  
  mot_reconf(m, m->enc1, m->enc2); //configure ISr to store all the delays between ticks
  m->pid->svel = step;
  
  init_acums(turns, m);			   //init & reset global arrays.
  memset(m->pid->cp,0.0,MAX_COEF*sizeof(double));
  memset(m->pid->cd,0.0,MAX_COEF*sizeof(double));
  
  for ( vel = MIN_VEL; index < ms ; vel+= step, index ++ ){ //interpolaremos con estos parametros, nos interesa terminar en MAX_VEL
    if( index  == ms-1 ) //TERNARY CARADECUL
      vel = MAX_VEL;
    if ( wait != 0 )
      wait_for_stop(m, wait);
    //printf("entro con wait = %f\n", wait);
    //printf("Despres de fer el reset:\n");
    //prac(((BASE*turns)+((BASE*turns)*0.2)), acum1);
    reset_acums(turns, m);
    es1[index] = (STAT *) malloc(sizeof(STAT));
    es2[index] = (STAT *) malloc(sizeof(STAT));
    move_till_ticks_b (m, tticks(m, turns), dir, vel, reset, 0);
    get_stats(es1[index], m, 1);
    get_stats(es2[index], m, 2);
    //printf("motor: %d, vel: %d, mean1: %f, mean2: %f, dev1: %f, dev2: %f\n", m->id, vel ,es1[index].mean, es2[index].mean, es1[index].absd, es2[index].absd);
  }

  /* Cacota que hay que quitar
  printf("\n");
  printf("\nMOTOR %d: \n", m->id);
  prstat(es1, es2, ms);
  END Cacota que hay que quitar */
  cmp_res(m->pid, es1, es2, ms);
  pid_on(m->pid);
  //mt_pid_set_gains(m->pid, auxpid.kp, auxpid.ki, auxpid.kd);
  mot_reconf(m, &aux1, &aux2);
  //free_acums(m);  <=== Un pokito poltergaist
}

static void get_vels (int svel, double out[], int size) {
  
  int i;

  out[0] = MIN_VEL;
  for(i = 1; i < size; i++)
    out[i] = i != size-1 ? MIN_VEL + (i * svel) : MAX_VEL;

}

static bool get_params (MOTOR * m, int vel, int * ex_micras, int * ex_desv){
  
  if(m->id != 0) {
    int size, i;
    for (size = 0; m->pid->cp[size] != 0; size++);
    double x[size];
    int v = vel < MIN_VEL ? MIN_VEL : vel; //si al final la dejo externa vigilar VMAX tambien
    int *res;
    gsl_interp *interp;
    
    if(vel < MIN_VEL)
      not_critical("get_params: Getting parameters for %d velocity, the minimum allowed\n", MIN_VEL);

    *ex_micras = 0;
    *ex_desv = 0;
    
    /* Funcion para calcular puntosX (velocidades para mi) a partir de svel (step velocidad)*/
    
    get_vels(m->pid->svel, x, size);
    interp = gsl_interp_alloc(gsl_interp_akima_periodic, size);
  
    for(i = 0; i < 2; i++){
      res = i == 0 ? ex_micras : ex_desv; 
      gsl_interp_init(interp, x, i == 0 ? m->pid->cp : m->pid->cd, size);
      *res = (int)gsl_interp_eval(interp, x, i == 0 ? m->pid->cp : m->pid->cd, (double)v, i == 0 ? m->pid->accelM : m->pid->accelD);
    }
    
    gsl_interp_free(interp);
 
  } else {
    not_critical("get_params: Motor not properly initialised\n");
    return false;
  }

  return true;
}


static void cmp_res(PID * pid, STAT * es1[], STAT * es2[], int size){
  
  int i;
  double points[size];
  double dv[size];
  
  for (i = 0; i < size; i++){
    points[i] = ((es1[i]->mean + es2[i]->mean)/2);
    dv[i] = ((es1[i]->absd + es2[i]->absd)/2);
  }
 
  mpid_load_coef(pid, points, dv, size);

}

/*Terminar de lijar ...*/
void get_stats(STAT * out, MOTOR * m , int eid){

  double * data = m->id == 1 ? eid == 1 ? acum1 : acum3 : eid == 1 ? acum2 : acum4;
  int len = get_len(data);
  //printf("DEBUG: entering get stats with len: %d\n", len);
  double weights[len];
  avg(len, data);
  double absd_aut; //sd_aut;
  
  //printf("DATA PUNTERO enc_%d: len = %d\n", eid, len);
  
  //prac(len+10, data);
  
  /*	if(m->enc1 == e11){
	printf("\n\n EMPIEZA ACUM: \n\n");
	prac(len, data);
	}
	for (i = 0; i < len; i++){  //==> fer-ho dintre de avg amb mijana parcial.
		if (data[i] < 650)
		data[i] = aver;
		}
  */
  cal_weight(len, weights, 1, 3);
  out->mean = gsl_stats_wmean (weights, 1, data, 1, len);
  absd_aut = gsl_stats_wabsdev_m (weights, 1, data, 1, len, out->mean);
  //  sd_aut = gsl_stats_wsd_m (weights, 1,data, 1, len, out->mean);
 //printf("La cosa enc_%d: len -> %d, mean -> %f, absd -> %f, sd -> %f, whatIuse -> %f%s", eid, len, out->mean, absd_aut, sd_aut, ((absd_aut + sd_aut)/2), eid == 1 ? "\n" : "\n\n" );
 
  if (absd_aut == 0 || absd_aut != absd_aut /*|| sd_aut == 0 || sd_aut != sd_aut*/) {
    printf("FAIL: DATA PUNTERO\n");
    prac(len, data);
  }
  
  out->absd = absd_aut;//((absd_aut + sd_aut)/2);//absd = absd_aut;
}

static void cal_weight(int len, double weights[], double less, double max){

int i;
int head = 10;
int tail = len - head;
int first = len/10;
int last = len - first;

    for (i=0; i<len; i++){
      if ((i <= head) || (i >= tail))
	weights[i] = 0;
      else if ((i <= first) || (i >= last))
	weights[i] = (double)less;
      else
	weights[i] = (double)max;
    }
}


static double avg (int len, double * data){

  long long sum = 0;
  int i;
 
  for (i = 50; i < len-50; i++) {
    if (data[i] > USXT_MIN)
      sum += data[i];
    else {
      sum += sum/(i-49);
      data[i] = USXT_MIN;
    }
  }
  return ((sum/(len-100)));
}


static int get_len(double in[]){

  int i;

  for(i = 0; !(in[i] == 0.0 && in[i+1] == 0.0 && in[i+2] == 0.0); i++);

  return i;

}

/*Cacota que hay que quitar */
void prac (int len, double * vec){

int i=0;
double val;
 for (; i < len; i++){
   val = vec[i];
   if( ( (i != 0) && ((i%30) == 0) ) )
     printf("%5.25f\n", (double)val);
   else
     printf("%5.25f, ", (double)val);
 }
 printf("\n\n");

}


static void init_acums(int turns, MOTOR * m){
  
  int size = (int)((BASE*turns)+((BASE*turns)*0.2));
  if( m->id == 1 ){
    if(!is_null(e11))
      //acum1 = (double *)calloc(size,sizeof(double));
      acum1 = (double *)realloc(acum1,size*sizeof(double));
    if(!is_null(e12))
      //acum3 = (double *)calloc(size,sizeof(double)); 
      acum3 = (double *)realloc(acum3,size*sizeof(double));
  } else {
    if(!is_null(e21))
      //acum2 = (double *)calloc(size,sizeof(double)); 
      acum2 = (double *)realloc(acum2,size*sizeof(double));
    if(!is_null(e22))
      //acum4 = (double *)calloc(size,sizeof(double)); 
      acum4 = (double *)realloc(acum4,size*sizeof(double));
  }
}

static void free_acums(MOTOR * m){
  
  if( m->id == 1 ){
    if(!is_null(e11)){
      free(acum1);
      acum1 = NULL;
    }
    if(!is_null(e12)){
      free(acum3);
      acum3 = NULL; 
    }
  } else {
    if(!is_null(e21)){
      free(acum2);
      acum2 = NULL;
    }
    if(!is_null(e22)){
      free(acum4);
      acum4 = NULL;
    }
  }
}

static void reset_acums(int turns, MOTOR * m){

  int size = ((BASE*turns)+((BASE*turns)*0.2));
  if( m->id == 1 ){
    if(!is_null(e11))
      memset(acum1,0.0,size*sizeof(double));
    if(!is_null(e12))
      memset(acum3,0.0,size*sizeof(double));
  } else {
    if(!is_null(e21))
      memset(acum2,0.0,size*sizeof(double));
    if(!is_null(e22))
      memset(acum4,0.0,size*sizeof(double));
  }
}

static int us_to_vel(int us, MOTOR * m){

int usTicks, unused, vel;

 get_params(m, MIN_VEL, &usTicks, &unused);

 for (vel = MIN_VEL+1;(usTicks > us && vel <= MAX_VEL); vel++)
   get_params(m, vel, &usTicks, &unused);
 
 return vel--;

}

/*int nextpw (int err, int inte, int der, int pherr){

	int tocomp = err + inte + der;

	return (((tocomp > 0) && (tocomp <= pherr*3)) ? 1 : (tocomp > pherr*3) ? 0 : (tocomp < -pherr*3) ? 2 : 4);


}*/

static void pid_launch (MOTOR * m, int vel, int limit, dir dir, double posCtrl, bool sinc){

  //limit: en cas de voler arribar fins a uns ticks determinats > 0, 0 altrament.
  
  //#FROM: http://en.wikipedia.org/wiki/PID_controller#Pseudocode
  TSPEC ini, fi;
  int usSP, absd, Mv, MvAux, prevErr = 0, Err, now_tk;  //abs es pot entendre com el error fisic del motor, axi doncs compensem l'error llegit amb aquest
  get_params(m, vel, &usSP, &absd);      //get PID objective
  int PhysErr = absd;
  int act_pw = (int)V2PW(vel);
  double ttc = m->pid->ttc; 		       //ttc => ticks_to_calibr., es a dir pel desfas de temps utilitzo el temps de ticks esperat com a base i multiplico per akesta var.
  int dt = (int)(usSP * (ttc/2));
  double Kp = m->pid->kp;
  double Ki = m->pid->ki;
  double Kd = m->pid->kd;
  long double Der, Int = 0;
  long long Out;
  double diff = 0.15;
  //bool forced = false;
  
  int ticks_dt;    /*Para realizar los calculos del err derivativo i el integr. usare los ticks que hayan pasado entre cada muestra tomada, de tal manera los resultados seran reales,
		     pese a que el delay de tiempo sea siempre el mismo los ticks que recivamos pueden variar, de hecho variaran (ttc double), es decir se abstary el delay de tiempo 
		     para trabajar con los ticks, los datos mas cercanos a la realidad con los que contamos */

  /* control errores */
  
  double tdtrange = vel < 170 ? 1.45 : 1.45 + (0.8 - ((double)(MAX_VEL - vel)/100)); 
  int ticks_dt_min, ticks_dt_max, ticks_dt_base;

  /* params reset pulse */

  int dErrmin , dErrmax, dErr, velAux, usSPbkup = usSP, minbkup, basebkup, maxbkup, unused; /*minDev, maxDev,unused*/

  int basepw = basebkup = act_pw;
  //int basephErr = vel <= 170 ? PhysErr : (int)(( ((double)((MAX_VEL - vel)/100.0)*2)+0.05) * PhysErr);

  get_params(m, PW2V((basepw + (int)(basepw*diff))), &velAux, &dErrmax);
  dErrmax = abs(V2PW( us_to_vel(velAux + dErrmax, m) ) - ( basepw + (int)(basepw*diff) ));
  
  get_params(m, PW2V((basepw - (int)(basepw*diff))), &velAux, &dErrmin);
  dErrmin = abs(( basepw - (int)(basepw*diff) ) - V2PW( us_to_vel(velAux - dErrmin, m) ));

  //dErrmax = dErrmax + (dErrmax * (double)(vel/MAX_VEL));
  //dErrmin = dErrmin + (dErrmin * 1 - (double)(vel/MAX_VEL));

  dErr = (int)(((dErrmin + dErrmax)/2)*0.7) ;

  int maxpw  = maxbkup = ((basepw + (int)(basepw*diff)) + dErr) > MAX_PW ? MAX_PW : ((basepw + (int)(basepw*diff)) + dErr);
  int minpw  = minbkup = ((basepw - (int)(basepw*diff)) - dErr) < V2PW(MIN_VEL) ? V2PW(MIN_VEL) : ((basepw - (int)(basepw*diff)) - dErr);
  
  int minus, maxus;
  
  get_params(m, PW2V(maxpw), &maxus, &unused);
  get_params(m, PW2V(minpw), &minus, &unused);

  int gpio = dir == FWD  ? m->pinf : m->pinr;
  
  long long LstAc = 0; 		//pillar medias parciales

  PhysErr = vel >= 170 ? (int)(PhysErr * (1 + (double)(1 - (double)((double)vel/MAX_VEL) ) ) ) : (int)(PhysErr * 1.5);

  //printf("MOTOR_%d: maxpw: %d >> basepw: %d >> minpw: %d\nErrmax: %d, Errmin: %d, diff %f, multi: %f, PW: %d, limit: %d \n", m->id, maxpw, basepw, minpw, dErrmax, dErrmin, diff, (1 + (double)(1 - (double)((double)vel/MAX_VEL) ) ), V2PW(vel), limit);

  /* Control posicion*/

  int cont = 0, times = posCtrl * 30, lparam = 0;
  bool pctrlact = false;

  /* Sincro dos motores*/

  int stimes = 10, scont = 0, errCont = 0;
  
  bool * sstble = m->id == 1  ? &msinc->s1 : &msinc->s2;
  bool * sstblen = m->id == 1  ? &msinc->s2 : &msinc->s1;

  bool * flag = m->id == 1  ? &msinc->flag1 : &msinc->flag2;
  bool * flagn = m->id == 1  ? &msinc->flag2 : &msinc->flag1;
  
  int * tsinc = m->id == 1  ? &msinc->t1 : &msinc->t2;
  double * dsinc = m->id == 1  ? &msinc->d1 : &msinc->d2;

  int * id = m->id == 1  ? &msinc->id1 : &msinc->id2;
  
  /* Sincro dos motores + posCtrl*/

  bool * myarr = m->id == 1 ? &mpsinc->arr1 : &mpsinc->arr2;
  bool * nearr = m->id == 1 ? &mpsinc->arr2 : &mpsinc->arr1;
  bool * myend = m->id == 1 ? &msinc->fin1  : &msinc->fin2;
  bool * neend = m->id == 1 ? &msinc->fin2  : &msinc->fin1;

	/* varios */
  
  double range = ((double)((MAX_VEL-vel)/2)/100); // la puta crema
  bool stop = false;
  
  while (get_ticks(m) == 0); //wait hasta tener datos
  
  if(limit == 0){
    while (m->moving){
      clock_gettime(CLK_ID, &ini);
      
      ticks_dt_min = act_pw == basepw ? (int)((double)(dt/usSP)/tdtrange) : act_pw == maxpw ? (int)((double)(dt/maxus)/tdtrange) : (int)((double)(dt/minus)/tdtrange);
      ticks_dt_max = act_pw == basepw ? (int)((double)(dt/usSP)*tdtrange) : act_pw == maxpw ? (int)((double)(dt/maxus)*tdtrange) : (int)((double)(dt/minus)*tdtrange);
      ticks_dt_base = act_pw == basepw ? (int)(dt/usSP) : act_pw == maxpw ? (int)(dt/maxus) : (int)(dt/minus);

      MvAux = get_MVac(m, &LstAc, &ticks_dt, ticks_dt_min, ticks_dt_base, ticks_dt_max, errCont);
      
      Mv = MvAux <= USXT_MIN ? usSP : MvAux;
      Err = (((usSP > Mv) && (act_pw == maxpw)) || ((usSP < Mv) && (act_pw == minpw))) ? (usSP - Mv) > (int)(range*PhysErr) ? ((usSP - Mv) - (int)(range*PhysErr)) : (usSP - Mv) < -(int)(range*PhysErr) ? (usSP - Mv) + (range*PhysErr) : 0 : (abs((usSP - Mv)) > PhysErr) ? (usSP - Mv) < 0 ? (usSP - Mv) + PhysErr : (usSP - Mv) - PhysErr : 0; //compensacion error fisico
      Int = Int + (Err * ticks_dt);
      Der = (Err - prevErr)/ticks_dt;
      Out = (Kp * Err) + (Ki * Int) + (Kd * Der);

      debug("PID_MOTOR_%d: Set point: %d, Mesured value: %d, Physical error: %d, ticks_dt: %d, act_pw : %d, Last: %lld, scont: %d\n", m->id, usSP, Mv, PhysErr, ticks_dt, act_pw, LstAc, scont);
      debug("PID_MOTOR_%d: Output value: %lld\n", m->id, Out );
      act_pw = reset_pulse(m, Out, act_pw, gpio, maxpw, minpw, basepw, PhysErr);
      
      if (sinc) {	//sincro 2 motores
	*sstble = act_pw == basepw;
	if (*sstble && *sstblen) { //wait for stabilization
	  if(scont == stimes){
	    *tsinc = LstAc;
	    *dsinc = difft(&msinc->tstamp, &ini);
	    
	    if(!(*flag)){
	      *flag = true;
	      msinc->first = msinc->first == 0 ? m->id : msinc->first;
	      debug("SINCRO: FLAG%d = TRUE, first -> %d \n", m->id, msinc->first);
	    } else {
	      if((*flagn)){
		if(msinc->first == m->id ){
		  msinc->first = 0;
		} else {
		  if(msinc->first == 0)
		    *id = *id + 1;
		}
	      }
	    }
	    
	    if((*id != 0)){
	      usSP = get_sincro_new_vals(m, usSP, &minpw, &basepw, &maxpw, basebkup, minbkup, maxbkup, usSPbkup, diff);
	      scont = 0;
	    }
	    
	  } else {
	    scont ++;
	  }
	} else {
	  scont = 0;
	}
      }
      
      prevErr = Err;
      errCont ++;
      clock_gettime(CLK_ID, &fi);
      while ( (difft(&ini, &fi) < dt) && (m->moving) )
	clock_gettime(CLK_ID, &fi);
    }
  } else {
    
    while (!stop) {
      
      now_tk = get_ticks(m);
      //printf("now_tk: %d\n ", now_tk);
      
      if (now_tk < limit){
	
	clock_gettime(CLK_ID, &ini);
	
	ticks_dt_min = act_pw == basepw ? (int)((double)(dt/usSP)/tdtrange) : act_pw == maxpw ? (int)((double)(dt/maxus)/tdtrange) : (int)((double)(dt/minus)/tdtrange);
	ticks_dt_max = act_pw == basepw ? (int)((double)(dt/usSP)*tdtrange) : act_pw == maxpw ? (int)((double)(dt/maxus)*tdtrange) : (int)((double)(dt/minus)*tdtrange);
	ticks_dt_base = act_pw == basepw ? (int)(dt/usSP) : act_pw == maxpw ? (int)(dt/maxus) : (int)(dt/minus);
	
	MvAux = get_MVac(m, &LstAc, &ticks_dt, ticks_dt_min, ticks_dt_base, ticks_dt_max, errCont);
	debug ("PID_MOTOR_%d: posCtrl = %f, sinc = %s, now_tk = %d\n", m->id, posCtrl, sinc ? "\"true\"" : "\"false\"", now_tk);

	if( posCtrl != 0 && !sinc){  //control de posicion
	  if ( ( posCtrl >= ( 1 - (double)( (double)now_tk/(double)limit ) ) ) ){
	    if (cont == times){
	      pctrlact = true;
	      if ( ( act_pw == basepw || act_pw == minpw ) ){
		lparam = (int)(((double)(limit-LstAc)/(double)ticks_dt)/(double)times);
		usSP = get_pid_new_vals(m, usSP, &minpw, &basepw, &maxpw, posCtrl, lparam);
		cont = 0;
	      }
	    } else {
	      cont ++;
	    }
	  }
	}
	
	if(posCtrl != 0 && sinc) {  //control de posicion + sincro
	  if ( ( posCtrl >= ( 1 - (double)( (double)now_tk/(double)limit ) ) ) ){
	    if(mpsinc->first == 0 || mpsinc->first == m->id){  //dejamos que mande el primero que llege al punto de frenada
	      *myarr = true; //este *myarr ara referencia al motor que haya llegado primero
	      mpsinc->first = m->id;
	      debug ("SINCRO_%d_POS: first: %d,  posCtrl: %f, spend: %f, cont: %d, actpw: %s, arrived1: %s, arrived2: %s, ticks_rec : %d\n", m->id, mpsinc->first, posCtrl, (double)now_tk/(double)limit, cont, act_pw == basepw ? "\" base \"" : act_pw == minpw ? "\" min \"" : "\" max \"", (*myarr) ? "\" true \"" : "\" false \"", (*nearr) ? "\" true \"" : "\" false \"", now_tk);
	      if (cont == times){
		if ( ( act_pw == basepw || act_pw == minpw ) && (*nearr)){
		  pctrlact = true;
		  lparam = (int)(((double)(limit-LstAc)/(double)ticks_dt)/(double)times);
		  usSP = get_pid_new_vals(m, usSP, &minpw, &basepw, &maxpw, posCtrl, lparam);
		  mpsinc->sp = usSP;
		  mpsinc->newmin = minpw;
		  mpsinc->newbase = basepw;
		  mpsinc->newmax = maxpw;
		  mpsinc->changes = true;
		  cont = 0;
		}
	      } else
		cont ++;
	      
	    } else {
	      *myarr = true; //este *myarr ara referencia al  motor que haya llegado mas tarde.
	      if(mpsinc->changes){
		pctrlact = true;
		usSP = mpsinc->sp;
		minpw = mpsinc->newmin;
		basepw = mpsinc->newbase;
		maxpw = mpsinc->newmax;
		mpsinc->changes = false;
		debug ("POSCTRL_%d_SLAVE: newmin: %d, newbase: %d, newmax : %d\n", m->id, mpsinc->newmin, mpsinc->newbase, mpsinc->newmax );
	      }
	    }
	    
	  }
	}
	
	
	Mv = MvAux <= USXT_MIN ? usSP : MvAux;
	Err = (((usSP > Mv) && (act_pw == maxpw)) || ((usSP < Mv) && (act_pw == minpw))) ? (usSP - Mv) > (int)(range*PhysErr) ? ((usSP - Mv) - (int)(range*PhysErr)) : (usSP - Mv) < -(int)(range*PhysErr) ? (usSP - Mv) + (range*PhysErr) : 0 : (abs((usSP - Mv)) > PhysErr) ? (usSP - Mv) < 0 ? (usSP - Mv) + PhysErr : (usSP - Mv) - PhysErr : 0; //compensacion error fisico
	Int = Int + (Err * ticks_dt);
	Der = pctrlact ? 0 : (Err - prevErr)/ticks_dt; //si control de posicion activo desactivamos Derivativo, de la manga izquierda...pero va mejor
	Out = (Kp * Err) + (Ki * Int) + (Kd * Der);
	
	debug("MOTOR_%d: Set point: %d, Mesured value: %d, Physical error: %d, ticks_dt: %d, act_pw : %d, Last : %lld, posCtrl: %f, ratio rec: %f, poscont: %d\n",m->id,  usSP, Mv, PhysErr, ticks_dt, act_pw, LstAc, posCtrl, (double)((double)(LstAc*2)/(double)limit), cont);
	debug("MOTOR_%d: Output value: %lld\n", m->id, Out);
	act_pw = reset_pulse(m, Out, act_pw, gpio, maxpw, minpw, basepw, PhysErr);
	
	if (sinc && !pctrlact) { //sincro 2 motores
	  *sstble = act_pw == basepw;
	  if (*sstble && *sstblen) { //wait for stabilization
	    if(scont == stimes){
	      *tsinc = LstAc;
	      *dsinc = difft(&msinc->tstamp, &ini);
	      
	      if(!(*flag)){
		*flag = true;
		msinc->first = msinc->first == 0 ? m->id : msinc->first;
		debug("SINCRO: FLAG%d = TRUE, first -> %d \n", m->id, msinc->first);
	      } else {
		if((*flagn)){
		  if(msinc->first == m->id){
		    msinc->first = 0;
		  } else {
		    if(msinc->first == 0)
		      *id = *id + 1;
		  }
		}
	      }
	      
	      if((*id != 0)){
		usSP = get_sincro_new_vals(m, usSP, &minpw, &basepw, &maxpw, basebkup, minbkup, maxbkup, usSPbkup, diff);
		scont = 0;
	      }
	      
	    } else
	      scont ++;
	  } else
	    scont = 0;
	}
	
     //printf("M_%d: SALGO DE SINCRO!!!, stop = \"%s\", myarr = \"%s\", nearr = \"%s\"\n", m->id, stop ? "true" : "false", (*myarr) ? "true" : "false", (*nearr) ? "true" : "false");

	prevErr = Err;
	errCont ++;
	clock_gettime(CLK_ID, &fi);
	while ( (difft(&ini, &fi) < dt) && (get_ticks(m) < limit) )
	  clock_gettime(CLK_ID, &fi);
	
      } else {
	debug ("MOTOR_%d: ARRIVED TO SET POINT\n", m->id);
	*myend = true;
	stop = !(*neend) && sinc ? false : !sinc ? true : true;
      }
      
    }
    
  }
  
}

static int get_sincro_new_vals(MOTOR *m, int usSP, int *minpw, int *basepw, int *maxpw, int basebkup, int minbkup, int maxbkup, int usSPbkup, double diff){
  
  int newsp = usSP, newbase, newmin, newmax, dErrmin, dErrmax, dErr, velAux, inc = 100;
  
  int * t2 = &msinc->t2;
  double * d1 = &msinc->d1;
  int * t1 = &msinc->t1;
  double * d2 = &msinc->d2;
  
  int * id1 = &msinc->id1;
  int * id2 = &msinc->id2;
  
  bool * myset = m->id == 1 ? &msinc->set1 : &msinc->set2;
  bool * neset = m->id == 1 ? &msinc->set2 : &msinc->set1;
  bool * myrset = m->id == 1 ? &msinc->mkrset1 : &msinc->mkrset2;
  bool * nerset = m->id == 1 ? &msinc->mkrset2 : &msinc->mkrset1;
  
  if(*id1 == *id2){  //solamente entrara uno de los 2 motores para cada sincronizaciÃn
    int tticks = m->id == 1 ? (*d1)/(*t1) : (*d2)/(*t2);
    int desf = 30;
    int trgtTicks = m->id == 1 ? (*t1) : (*t2);
    int neighTicks = m->id == 1 ? (*t2) : (*t1);
    double trgtDly = m->id == 1 ? (*d1) : (*d2);
    double neighDly = m->id == 1 ? (*d2) : (*d1);
    int diffdelay = abs((msinc->d1)-(msinc->d2));
    
    debug("SINCRO_%d: tticks > %d, difft/tticks : %d, t1 > %d, t2 > %d, delay1: %f, delay2: %f, diffdelay : %d, minpw : %d, basepw : %d, maxpw : %d\n", m->id, tticks, (int)(diffdelay/tticks), msinc->t1, msinc->t2, msinc->d1, msinc->d2, diffdelay, minbkup, basebkup, maxbkup);
    
    if( ( ( (trgtTicks - neighTicks > desf) && (trgtDly < neighDly) ) || ( ( (trgtTicks > neighTicks) && (trgtDly > neighDly) ) && ( ((diffdelay/tticks) - (trgtTicks - neighTicks)) < -desf ) ) || ( ( (trgtTicks < neighTicks) && (trgtDly < neighDly) ) && ( ((diffdelay/tticks) - (trgtTicks - neighTicks)) > desf ) ) ) ){ // sacar relacion entre el wait total i el delay para saber si tics vienen bien
      newsp = usSP + inc;
      int bse = V2PW(us_to_vel(newsp, m));
      newbase = bse >= MAX_PW ? MAX_PW : bse;
      
      get_params(m, PW2V((newbase + (int)(newbase * diff))), &velAux, &dErrmax);
      dErrmax = abs(V2PW( us_to_vel(velAux + dErrmax, m) ) - ( newbase + (int)(newbase*diff) ));
      
      get_params(m, PW2V((newbase - (int)(newbase*diff))), &velAux, &dErrmin);
      dErrmin = abs(( newbase - (int)(newbase * diff) ) - V2PW( us_to_vel(velAux - dErrmin, m) ));
      
      dErr = (int)(((dErrmin + dErrmax)/2)*0.7) ;
      
      newmax  = ((newbase + (int)(newbase * diff)) + dErr) > MAX_PW ? MAX_PW : ((newbase + (int)(newbase * diff)) + dErr);
      newmin  = ((newbase - (int)(newbase * diff)) - dErr) < V2PW(MIN_VEL) ? V2PW(MIN_VEL) : ((newbase - (int)(newbase * diff)) - dErr);
      
      *myset  = true;
      *minpw  = newmin;
      *basepw = newbase;
      *maxpw  = newmax;
      
      debug("SINCRO_%d FRENA: newmin: %d, newbase: %d, newmax : %d\n", m->id, newmin, newbase, newmax );
      
    } else if ( ( ( (trgtTicks - neighTicks <  -desf) && (trgtDly > neighDly)) || (((trgtTicks < neighTicks) && (trgtDly < neighDly))&& (((diffdelay/tticks) - abs(trgtTicks - neighTicks)) < -desf ) ) || (((trgtTicks > neighTicks) && (trgtDly > neighDly))&& (((diffdelay/tticks) - (trgtTicks - neighTicks)) > desf) ) ) ){
      newsp = usSP - inc;
      int bse = V2PW(us_to_vel(newsp, m));
      newbase = bse >= MAX_PW ? MAX_PW : bse;
      
      get_params(m, PW2V((newbase + (int)(newbase * diff))), &velAux, &dErrmax);
      dErrmax = abs(V2PW( us_to_vel(velAux + dErrmax, m) ) - ( newbase + (int)(newbase*diff) ));
      
      get_params(m, PW2V((newbase - (int)(newbase*diff))), &velAux, &dErrmin);
      dErrmin = abs(( newbase - (int)(newbase * diff) ) - V2PW( us_to_vel(velAux - dErrmin, m) ));
      
      dErr = (int)(((dErrmin + dErrmax)/2)*0.7) ;
      
      newmax  = ((newbase + (int)(newbase * diff)) + dErr) > MAX_PW ? MAX_PW : ((newbase + (int)(newbase * diff)) + dErr);
      newmin  = ((newbase - (int)(newbase * diff)) - dErr) < V2PW(MIN_VEL) ? V2PW(MIN_VEL) : ((newbase - (int)(newbase * diff)) - dErr);
      
      *myset  = true;
      *minpw  = newmin;
      *basepw = newbase;
      *maxpw  = newmax;
      
      debug("SINCRO_%d ACEL: newmin: %d, newbase: %d, newmax : %d\n", m->id, newmin, newbase, newmax );
      
    } else if((*myset) || (*myrset)){ //reset, en caso de estar compensando error y llegar a un desfas de ticks admitido reseteamos los pulsos
      if((*neset) && !(*nerset))
	*nerset = true;
      *myset  = false;
      *myrset  = false;
      *minpw  = minbkup;
      *basepw = basebkup;
      *maxpw  = maxbkup;
      newsp  = usSPbkup;
      debug("SINCRO_%d RES: newmin: %d, newbase: %d, newmax : %d\n", m->id, minbkup, basebkup, maxbkup );
			
    }//si no estamos bien

  } else if((*myrset)){ //aki entramos con el motor que actualmente no este calibrando, por si hay que resetearlo
    *myset  = false;
    *myrset = false;
    *minpw  = minbkup;
    *basepw = basebkup;
    *maxpw  = maxbkup;
    newsp  = usSPbkup;
    debug ("SINCRO_%d RES_INV: newmin: %d, newbase: %d, newmax : %d\n", m->id, minbkup, basebkup, maxbkup );
    
  }
  
  return newsp;
  
}

static int get_pid_new_vals(MOTOR * m, int sp, int * minpw, int * basepw, int * maxpw, double pctr, int ticksdt){

  int maxsp, unused, res, auxmin = *minpw, ret;

  get_params(m, MIN_VEL, &maxsp, &unused);

  int gain = ((maxsp - sp)/ticksdt);
  double gmul = ( ( (1 - pctr)*10)/2 );
  ret = sp + (int)(gain*(gmul/2));
  res = (sp + (gain * gmul));
  
  int newbase = V2PW(us_to_vel(res, m));
  int newmax  = V2PW(us_to_vel(sp, m));
  int ex_min  = V2PW(us_to_vel(res + gain, m));
  int newmin  = (auxmin < ex_min) ? auxmin : ex_min <= V2PW(MIN_VEL) ? V2PW(MIN_VEL) : ex_min;

  *maxpw  = newmax;
  *basepw = newbase;
  *minpw  = newmin;

  debug ("POSCTRL_%d: newmin: %d, newbase: %d, newmax : %d, gain multiplier: %f\n", m->id, newmin, newbase, newmax, (((1 - pctr)*10)/2) );
  return ret;

}

static long long get_MVac(MOTOR * m, long long *last, int *tdt, int tdtmin, int tdtbase, int tdtmax, int errCont){

  TSPEC taux;
  int t1, t2, t, d1, d2, d, dt, newdt;

  mot_lock(m);
  t1 = m->enc1->tics;
  t2 = m->enc2->tics;
  mot_unlock(m);
  
  clock_gettime(CLK_ID, &taux);
  
  d1 = difft(&m->enc1->tmp, &taux);
  d2 = difft(&m->enc2->tmp, &taux);
  
  t = t1 == 0 || t2 == 0 ? 1 :(int)((t1 + t2)/2);
  d = (int)((d1 + d2)/2);
  
  dt = t - *last;
  *last = t;
  *tdt = newdt =(dt < tdtmin || dt > tdtmax) && errCont >=5 ? (abs(dt-tdtbase) <= abs(dt-tdtmin) && abs(dt-tdtbase) <= abs(dt-tdtmax)) ? tdtbase : (abs(dt-tdtbase) > abs(dt-tdtmin) && abs(dt-tdtmin) < abs(dt-tdtmax)) ? tdtmin : (abs(dt-tdtbase) > abs(dt-tdtmax) && abs(dt-tdtmin) > abs(dt-tdtmax)) ? tdtmax : dt : dt;

  if(*tdt != dt)
    debug ("\n\nTICS_%d: entro restabilizacion tics!! ticks_dt : %d, tdtaux : %d, tmin: % d, tmax: %d \n\n", m->id, *tdt, dt, tdtmin, tdtmax);
  
  clock_gettime(CLK_ID, &m->enc1->tmp);
  clock_gettime(CLK_ID, &m->enc2->tmp);
  return (d/newdt);
  
}

static int reset_pulse (MOTOR * m, long long usPidOut, int act_pw, int gpio, int maxpw, int minpw, int basepw, int PHbkup){

  /*
    
    Despues del semifracaso anterior, para evitar resetear mucho los canales DMA he implementado otra version de resetear pulso, menos agresiva, en que solo tenemos 3 estados:
    
	1- Estamos en un error admisible para la velocidad, mantenemos el pulso.
	2- Hemos dejado atras ticks, es decir el motor rueda a una velocidad menor de la esperada, augmentamos el pulso al maximo permitido (PW2V(maxpw)) hasta reducir el error
	3- Hemos recorrido mas ticks de los esperados, rodamos a mayor velocidad de lo esperado, reducimos el pulso al maximo hasta qreducir el error


*/

  int incr=0, newpw;

  if ((abs(usPidOut) <= PHbkup*3) && (act_pw != basepw)) {

    debug ("MOTOR_%d: ENTRO BASE: PIDout: %lld, PHbackup * 3 : %d\n", m->id, usPidOut, PHbkup*3);
    if (act_pw == maxpw || (act_pw > basepw && act_pw > minpw)){ //vengo de maxima
      //mot_lock(m);
      spwm_clear_channel_gpio(m->chann,gpio);
      spwm_add_channel_pulse(m->chann, gpio, 0, basepw == MAX_PW ? MAX_PW - 1 : basepw );
      //mot_unlock(m);
      
    } else if(act_pw == minpw || (act_pw < maxpw && act_pw < basepw)){  //vengo de la minima,
      incr = basepw - act_pw;
      //mot_lock(m);
      spwm_add_channel_pulse(m->chann, gpio, basepw, (basepw + incr) >= MAX_PW ? ((MAX_PW - 1) - basepw) : incr );
      //mot_unlock(m);
    }
    
    debug ("MOTOR_%d: BASE: PIDout: %lld, max_pw: %d >= act_pw: %d >= min_pw: %d, incr : %d\n\n", m->id, usPidOut, maxpw, basepw, minpw, incr);
    return basepw;
    
  } else if ((act_pw != maxpw) && (usPidOut < -(PHbkup*3))) { //hay error y no lo estamos corrigiendo
    if (act_pw == minpw) 	//si venimos de la minima estavamos corrigiendo un error back_to_def para evitar mucha fluctuacion
      newpw = basepw;
    else
      newpw = maxpw;
    
    incr = newpw - act_pw;
    //mot_lock(m);
    spwm_add_channel_pulse(m->chann, gpio, act_pw, (act_pw + incr) >= MAX_PW ? ((MAX_PW - 1) - act_pw) : incr );
    //mot_unlock(m);
    debug ("MOTOR_%d: MAX: PIDout: %lld, max_pw: %d >= act_pw: %d >= min_pw: %d, incr: %d\n\n", m->id, usPidOut, maxpw, newpw, minpw, incr);
    return newpw;
    
  } else if ((act_pw != minpw) && (usPidOut > (PHbkup*3))) { //error inverso sin tratar
    if (act_pw == maxpw) //si venimos de la maxima estavamos corrigiendo un error back_to_def para evitar mucha fluctuacion
      newpw = basepw;
    else
      newpw = minpw;
    
    //mot_lock(m);
    spwm_clear_channel_gpio(m->chann,gpio);
    spwm_add_channel_pulse(m->chann, gpio, 0, newpw);
    //mot_unlock(m);
    debug ("MOTOR_%d: MIN: PIDout: %lld, max_pw: %d >= act_pw: %d >= min_pw: %d\n\n", m->id, usPidOut, maxpw, newpw, minpw);
    return newpw;
  } //si no no_act
  
  
  debug ("MOTOR_%d: NO ACT: PIDout: %lld, max_pw: %d >= act_pw: %d >= min_pw: %d\n\n", m->id, usPidOut, maxpw, act_pw, minpw);
  return act_pw;
  
}


static bool mot_lock (MOTOR * m){
  
  pthread_mutex_lock(&m->enc1->mtx);
  pthread_mutex_lock(&m->enc2->mtx);
  return true;  

}

static bool mot_unlock (MOTOR * m){

  pthread_mutex_unlock(&m->enc1->mtx);
  pthread_mutex_unlock(&m->enc2->mtx);
  return true;

}

extern int mt_move_sinc (dir dir, int vel){

  /* mover los 2 motores en sincronia */
  int ret = OK;
  
  if(m1->id != 0 && m2->id != 0) {
    if(!m1->moving && !m2->moving){
      
      msinc->acting = true;
      msinc->t1 = msinc->d1 = msinc->t2 = msinc->d2 = 0;
      
      int usTick1, usTick2, usDev1, usDev2, usMean, vel1, vel2; //unused, usDv1, usDv2;
      
      /* tratamos las diferencias entre los tiempos entre tics esperados para el motor1 y el motor2 ambos con el mismo voltage */
      
      get_params(m1, vel, &usTick1, &usDev1);
      get_params(m2, vel, &usTick2, &usDev2);
      
      usMean = (usTick1 + usTick2)/2; 	/*	Cuando tengamos la media de tiempo entre tics de los 2 motore para el voltage V2PW(vel), sacamos la velocidad que le corresponde a cada uno*/
    
      vel1 = us_to_vel(usMean, m1);
      vel2 = us_to_vel(usMean, m2);
      
      msinc->id1 = msinc->id2 = 0;
      msinc->flag1 = false;
      msinc->flag2 = false;
      msinc->first = 0;
      msinc->s1 = false;
      msinc->s2 = false;
      msinc->set1 = false;
      msinc->set2 = false;
      msinc->mkrset1 = false;
      msinc->mkrset2 = false;
      msinc->fin1 = msinc->fin2 = false;
      
      /* aqui llamamos a la funcion publica move para cada motor, que creara un thread para cada uno, como hemos activado la sincronia, el control PID se precupara de ir sincronizando los motores */

      clock_gettime(CLK_ID, &msinc->tstamp); //timestamp que compartiran los motores, para ir sincronizandolos
      ret = move(m1, dir, vel1);
      ret = ret ? move(m2, dir, vel2) : FAIL;
      
    } else {

      not_critical("move_sinc: Motors alrady moving, stop them first\n");
      ret = FAIL;

    }
    
  } else {
    
    not_critical("move_sinc: Motors not properly initialised\n");
    return FAIL;

  }
    
  return ret;

}

extern int mt_move_sinc_t (dir dir, int vel, int lim, double posCtrl){

  /* mover los 2 motores en sincronia hasta ticks = lim */
  int ret = OK;
  
  if(m1->id != 0 && m2->id != 0){
    if(!m1->moving && !m2->moving){

      msinc->acting = true;
      msinc->t1 = msinc->d1 = msinc->t2 = msinc->d2 = 0;
      
      int usTick1, usTick2, usDev1, usDev2, usMean, vel1, vel2; //unused, usDv1, usDv2;
    
      /* tratamos las diferencias entre los tiempos entre tics esperados para el motor1 y el motor2 ambos con el mismo voltage */
    
      get_params(m1, vel, &usTick1, &usDev1);
      get_params(m2, vel, &usTick2, &usDev2);
      
      usMean = (usTick1 + usTick2)/2;  /* Cuando tengamos la media de tiempo entre tics de los 2 motore para el voltage V2PW(vel), sacamos la velocidad que le corresponde a cada uno */
      
      vel1 = us_to_vel(usMean, m1);
      vel2 = us_to_vel(usMean, m2);
    
      /*	inicializamos struct sincro */
      
      msinc->id1 = msinc->id2 = 0;
      msinc->flag1 = false;
      msinc->flag2 = false;
      msinc->first = 0;
      msinc->s1 = false;
      msinc->s2 = false;
      msinc->set1 = false;
      msinc->set2 = false;
      msinc->mkrset1 = false;
      msinc->mkrset2 = false;
      msinc->fin1 = msinc->fin2 = false;
    
      /*	inicializamos struct sincro + posCtrl */
      
      if(posCtrl != 0){
      
	mpsinc->arr1 = mpsinc->arr2 = false;
	mpsinc->newmin =  mpsinc->newbase =  mpsinc->newmax = 0;
	mpsinc->first = 0;
	mpsinc->sp = 0;
	mpsinc->changes = false;
      }

      /* aqui llamamos a la funcion publica move_t para cada motor, que creara un thread para cada uno, como hemos activado la sincronia, el control PID se precupara de ir sincronizando los motores */

      clock_gettime(CLK_ID, &msinc->tstamp); // timestamp que compartiran los motores, para ir sincronizandolos
      ret = move_t(m1, lim, dir, vel1, posCtrl);
      ret = ret ? move_t(m2, lim, dir, vel2, posCtrl) : FAIL;
      
    } else {
      not_critical("move_sinc_t: Motors alrady moving, stop it first\n");
      ret = FAIL; 
    }
  } else {
    
    not_critical("move_sinc_t: Motors not properly initialised\n");
    return FAIL;    
  
  }

  return ret;
  
}






