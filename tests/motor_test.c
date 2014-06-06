#include <lego/lego_motor.h>
#include <gsl/gsl_sort.h>
#include <getopt.h>

#define STATS_SIZE	18
#define PER_SIZE	11
//#define INT1_KEY	1
//#define INT2_KEY	2
#define MAXM		40
#define ENC_RES		4
#define PRAC_TRUNC      25

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

TSPEC t11, t12, t21, t22;

MOTOR * mt1 = NULL , * mt2 = NULL, * mt = NULL;
MOTOR * m1isr, * m2isr; 

double *acum4 = NULL, *acum3 = NULL, *acum2 = NULL, *acum1 = NULL;


struct result {
	double res[STATS_SIZE];
	double per[PER_SIZE];
};
typedef struct result RESULT;

struct resfive{
	double txsec;
	double tturn;
	double e1[ENC_RES];
	double e2[ENC_RES];

};
typedef struct resfive RFIVE;

int get_in(char *to_print, int type);
int stats(RESULT *, bool, double *, bool, bool, int, int, bool, int, bool);
void prac(int, double *);
void prw(int, double[]);
double avg (int, double *);
void cal_weight(int,double *,double,double);
void smart_weights(int, double *, double, double, double *, int, double []);
int no_quantil (int, double *, double, double, double);
void pres (double **, int, int, int);
void presf (RESULT *, int, int);
void comp_vel(int, int, double **, int);
void comp_perc(int, int, double **, int);
void comp_res(RESULT *, int, int, int);
void col_to_row(double *,double **, int, int);
void pr_stats(double *, int);
int cpacum(double *, int, int, int);
int cptable(double *, int, double *);
void get_limits(double[], double *, double *, double, double, double *, double *);
void init_acums(int, MOTOR *);
void reset_acums(int, MOTOR *);
void free_acums(MOTOR * m);
bool gt_pars (MOTOR * m, int vel, int * ex_micras, int * ex_desv);
void tr_enc(double * [], int);
void prfive(RFIVE *, int);
double difft (TSPEC *, TSPEC *);
void prwcr (int len, double per[]);

extern void isr_print_11(void){
  if(m1isr->moving){
    m1isr->enc1->tics++;
    if ((m1isr->enc1->tics%300) == 0)
      printf("Hi, I'm the ISR 11\n");
  }
  clock_gettime(CLK_ID, &m1isr->enc1->tmp);
}

extern void isr_print_12(void){
  if(m1isr->moving){
    m1isr->enc2->tics++;
    if ((m1isr->enc2->tics%400) == 0)
      printf("Hi, I'm the ISR 12\n");
  }
  clock_gettime(CLK_ID, &m1isr->enc2->tmp);
}

extern void isr_print_21(void){
  if(m2isr->moving){
    m2isr->enc1->tics++;
    if ((m2isr->enc1->tics%300) == 0)
      printf("Hi, I'm the ISR 21\n");
  }
  clock_gettime(CLK_ID, &m2isr->enc1->tmp);
}

extern void isr_print_22(void){
  if(m2isr->moving){
    m2isr->enc2->tics++;
    if ((m2isr->enc2->tics%400) == 0)
      printf("Hi, I'm the ISR 22\n");
  }
  clock_gettime(CLK_ID, &m2isr->enc2->tmp);
}

extern void dbg_isr_11(void){
  if(m1isr->moving){
    clock_gettime(CLK_ID, &t11);
    acum1[m1isr->enc1->tics] = difft(&m1isr->enc1->tmp, &t11);//e11->tics == 0 ? (double)((e11->tmp.tv_sec * 1000000) + (e11->tmp.tv_nsec/1000)) : (e11->delay); /* NANOS TO MICR0S */
    pthread_mutex_lock(&m1isr->enc1->mtx);
    m1isr->enc1->tics++;
    pthread_mutex_unlock(&m1isr->enc1->mtx);
    clock_gettime(CLK_ID, &m1isr->enc1->tmp);
  }
}

extern void dbg_isr_12(void){
  
  if(m1isr->moving){
    clock_gettime(CLK_ID, &t12);
    acum3[m1isr->enc2->tics] = difft(&m1isr->enc2->tmp, &t12);//e12->tics == 0 ? (double)((e12->tmp.tv_sec * 1000000) + (e12->tmp.tv_nsec/1000)) : (e12->delay); /* NANOS TO MICR0S */
    pthread_mutex_lock(&m1isr->enc2->mtx);
    m1isr->enc2->tics++;
    pthread_mutex_unlock(&m1isr->enc2->mtx);
    clock_gettime(CLK_ID, &m1isr->enc2->tmp);
  }
}

extern void dbg_isr_21(void){
  if(m2isr->moving){
    clock_gettime(CLK_ID, &t21);
    acum2[m2isr->enc1->tics] = difft(&m2isr->enc1->tmp, &t21);//e21->tics == 0 ? (double)((e21->tmp.tv_sec * 1000000) + (e21->tmp.tv_nsec/1000)) : (e21->delay); /* NANOS TO MICR0S */
    pthread_mutex_lock(&m2isr->enc1->mtx);
    m2isr->enc1->tics++;
    pthread_mutex_unlock(&m2isr->enc1->mtx);
    clock_gettime(CLK_ID, &m2isr->enc1->tmp);
  }
}

extern void dbg_isr_22(void){
  if(m2isr->moving){
    clock_gettime(CLK_ID, &t22);
    acum4[m2isr->enc2->tics] = difft(&m2isr->enc2->tmp, &t22);//e22->tics == 0 ? (double)((e22->tmp.tv_sec * 1000000) + (e22->tmp.tv_nsec/1000)) : (e22->delay); /* NANOS TO MICR0S */
    pthread_mutex_lock(&m2isr->enc2->mtx);
    m2isr->enc2->tics++;
    pthread_mutex_unlock(&m2isr->enc2->mtx);
    clock_gettime(CLK_ID, &m2isr->enc2->tmp);
  }
}
                                                                                                                                                                                    

double pcoef1[MAX_COEF] = { 5769.92181090883605065755546092987060547, 4479.09869805242851725779473781585693359, 3815.51846175456466880859807133674621582, 3281.67669122572169726481661200523376465,2960.91753850553914162446744740009307861, 2677.93018281985223438823595643043518066, 2443.18133088759122983901761472225189209, 2256.43226464750478044152259826660156250, 2076.36443362455156602663919329643249512, 1880.84519361533693881938233971595764160, 1766.91845025184238693327642977237701416, 1627.52019487895177007885649800300598145, 1510.59062333105975994840264320373535156, 1405.75873144575962214730679988861083984, 1300.50733998499845256446860730648040771, 1199.64103984038956696167588233947753906, 1126.43123210844078130321577191352844238,1077.82681899074850662145763635635375977, 1047.62979268326716919546015560626983643, 1024.72136637642188361496664583683013916, 0.0 };

double dcoef1[MAX_COEF] = { 1280.81678440747532476962078362703323364, 1080.55413534975264155946206301450729370, 962.37714162269867301802150905132293701, 794.70516615830206319515127688646316528, 681.73197952620921569177880883216857910, 617.17903483213251547567779198288917542, 570.02783788164811085152905434370040894, 514.28307792829866684769513085484504700, 485.39651961788968037581071257591247559, 434.73204754488267553824698552489280701, 411.49080190769848286436172202229499817, 396.74984722622548360959626734256744385, 376.29638030608435883550555445253849030, 365.62140796971101508461288176476955414, 343.04282202860895267804153263568878174, 323.94336764183344712364487349987030029, 314.57135151003888040577294304966926575, 287.30168694078963653737446293234825134, 278.15403125283489771391032263636589050, 228.85462293561045044043567031621932983, 0.0 };

double pcoef2[MAX_COEF] = { 5784.08437837837846018373966217041015625, 4445.20846790107862034346908330917358398, 3747.11780970736981544177979230880737305, 3279.89248824115747993346303701400756836,2910.45785170638373529072850942611694336, 2633.61340680736202557454816997051239014, 2381.54762904100789455696940422058105469, 2163.21015912026996375061571598052978516, 1999.84504192275835521286353468894958496, 1844.27244297564470798533875495195388794, 1715.28248146105988780618645250797271729, 1597.04232922647133818827569484710693359, 1481.29561624296366062480956315994262695, 1377.86708383419227175181731581687927246, 1301.24395926704619341762736439704895020, 1248.96952471514123317319899797439575195, 1149.37726195246636962110642343759536743,1117.65528375465146382339298725128173828, 1097.79113254987851178157143294811248779, 1071.86196162545297738688532263040542603, 0.0 };

double dcoef2[MAX_COEF] = { 1129.16526209913718048483133316040039062, 883.12832455475847837078617885708808899, 819.31381796521941396349575370550155640, 742.33985454770515843847533687949180603, 685.03265188058412604732438921928405762, 666.17199380024021593271754682064056396, 627.18803536872428594506345689296722412, 599.68030858976464969600783661007881165, 585.16856491592193378892261534929275513, 571.32514217480252227687742561101913452, 553.57073691453115316107869148254394531, 548.80894970490135165164247155189514160, 537.02872878985044735600240528583526611, 525.58145446398498279449995607137680054, 506.85942229072389864086289890110492706, 487.50514143674456590815680101513862610, 439.71799989441075240392819978296756744, 425.31908384162727543298387899994850159, 411.85678335247609993530204519629478455, 409.68090676785232062684372067451477051, 0.0 };

static double kp = 0, ki = 0, kd = 0, pctr = 0;
static bool pid = false, log_dbg = false, to_calib = false;
static int vel = 60;
static int calib = 20;
static int step = 10; 
static int tst = 0;
static int port = 0;
static int verb = 0;
static int ttc = TTCDEF;
static int turns = 8;
static int enc = 2;
static int mostres = 10;
static dir dr = FWD;

static void print_usage(const char *prog)
{
	printf("Usage: %s [-tvpVcsPIDTrebSdCl]\n", prog);
	puts("  -t --test      test number to perform [1-11]\n"
	     "  -v --vel       velocity [0-200]\n"
	     "  -p --port      motor port [0-1], <2> both\n"
	     "  -V --verbose   verbose level [0-3] \n"
	     "  -c --calib     calibration samples, [5-20]\n"
	     "  -s --step      only apply to some tests\n"
	     "  -C --pid       use P.I.D control [no arg]\n"
	     "  -P --kp        proportional gain for P.I.D control (double)\n"
	     "  -I --ki        integral gain for P.I.D control (double)\n"
	     "  -D --kd        derivative gain for P.I.D control (double)\n"
	     "  -T --ttc       ttc field of P.I.D control [use with caution]\n"
	     "  -l --dbg       Set the library in debug mode [no arg]\n"
	     "  -r --turns     turns of the output hub, only apply to some tests\n"
	     "  -e --encod     Encoder lines active per motor [1-2]\n"
	     "  -b --psctrl    Position control (double) [0-1]\n"
	     "  -S --samples   Samples to store, aplly only to some tests\n"
	     "  -d --dir       direction to move the motor/s 0 = FWD, 1 = BWD\n");
}


   
static void parse_opts(int argc, char *argv[])
{
  while (1) {
    static const struct option lopts[] = {
      { "test",    1, 0, 't' },
      { "vel" ,    1, 0, 'v' },
      { "port",    1, 0, 'p' },
      { "verbose", 1, 0, 'V' },
      { "calib",   1, 0, 'c' },
      { "step",    1, 0, 's' },
      { "kp",      1, 0, 'P' },
      { "ki",      1, 0, 'I' },
      { "kd",      1, 0, 'D' },
      { "ttc",     1, 0, 'T' },
      { "turns",   1, 0, 'r' },
      { "encod",   1, 0, 'e' },
      { "psctrl",  1, 0, 'b' },
      { "samples", 1, 0, 'S' },
      { "dir",     1, 0, 'd' },
      { "pid",     0, 0, 'C' },
      { "dbg",     0, 0, 'l' },
      { 0, 0, 0, 0 }
    };
 
    int c;
    
    c = getopt_long(argc, argv, "t:v:p:V:c:s:P:I:D:T:r:e:b:S:d:Cl", lopts, NULL);
    
    if (c == -1)
      break;
    
    switch (c) {
    case 't':
      tst = atoi(optarg);
      break;
    case 'v':
      vel = atoi(optarg);
      break;
    case 'p':
      port = atoi(optarg);
      break;
    case 'V':
      verb = atoi(optarg);
      break;
    case 'c':
      calib = atoi(optarg);
      to_calib = (calib >= 5 && calib <= 20) ?  true : false;
      break;
    case 's':
      step = atoi(optarg);
      break;
    case 'P':
      kp = atof(optarg);
      break;
    case 'I':
      ki = atof(optarg);
      break;
    case 'D':
      kd = atof(optarg);
      break;
    case 'C':
      pid = true;
      break;
    case 'T':
      ttc = atoi(optarg);
      break;
    case 'l':
      log_dbg = true;
      break;
    case 'r':
      turns = atoi(optarg);
      break;
    case 'e':
      enc = atoi(optarg);
      break;
    case 'b':
      pctr = atof(optarg);
      break;
    case 'S':
      mostres = atoi(optarg);
      break;
    case 'd':
      dr = (dir)atoi(optarg);
      break;
    default:
      print_usage(argv[0]);
      exit(EXIT_FAILURE);
      break;
    }
  }
}
 



int main (int argc, char * argv[]) {

  
  parse_opts(argc, argv);
  
  ENC en11, en12, en21, en22;
  
  bool ret = false;
  bool needdbg = tst == 3 || tst == 4 || tst == 5 ;

  /*for tests 3, 4 & 5*/
  en11.pin = M1_ENC1;
  en11.isr = &dbg_isr_11;
  en12.pin = M1_ENC2;
  en12.isr = &dbg_isr_12;
  en21.pin = M2_ENC1;
  en21.isr = &dbg_isr_21;
  en22.pin = M2_ENC2;
  en22.isr = &dbg_isr_22;

  ret = mt_init();
  if (ret){
    mt_set_verbose(log_dbg ? LOG_LVL_DBG : LOG_LVL_ADV);
    
    if(port < 2){
      if(tst == 11 || tst == 10) {
	printf("Two motors needed for tests %d \n", tst);
	mt_shutdown();
	exit(EXIT_FAILURE);
      }
      if(enc == 2)
	mt = mt_new(needdbg ? port == 0 ? &en11 : &en21 : NULL, needdbg ? port == 0 ? &en12 : &en22 : NULL, port);
      else if (enc == 1)
	mt = mt_new(needdbg ? port == 0 ? &en11 : &en21 : NULL, ECNULL, port);
      else {
	printf("Encoder line parameter not understood\n");
	mt_shutdown();
	exit(EXIT_FAILURE);
      }

      if(port == 0)
	m1isr = mt;
      else
	m2isr = mt; 
      
    } else if (port == 2) {
      
      if(tst != 11 && tst != 10 && tst != 7 && tst != 8) {
	printf("Only one motor needed for test %d \n", tst);
	mt_shutdown();
	exit(EXIT_FAILURE);
      }
      
      if(enc == 2) { 
	mt1 = mt_new(needdbg ? &en11 : NULL, needdbg ? &en12 : NULL, 0);
	mt2 = mt_new(needdbg ? &en21 : NULL, needdbg ? &en22 : NULL, 1);
      } else if (enc == 1) {
	mt1 = mt_new(tst == 7 || tst == 8 ? &en11 : NULL, ECNULL, 0);
	mt2 = mt_new(tst == 7 || tst == 8 ? &en21 : NULL, ECNULL, 1);
      } else {
	printf("Encoder line parameter not understood\n");
	mt_shutdown();
	exit(EXIT_FAILURE);
      }
      
      m1isr = mt1;
      m2isr = mt2;

    } else {
      printf("Motor port not understood\n");
      mt_shutdown();
      exit(EXIT_FAILURE);
    }
    
    
    if(to_calib) {
      port < 2 ? printf("Calibrating MOTOR %d\n\n", port) : printf("Calibrating MOTORS\n\n") ;
      
      mt_calibrate(calib, 0.8);
      if(verb > 0) {
	if(port == 2){
	  printf("time between ticks 0: \n");prwcr(calib, mt1->pid->cp);
	  printf("time between ticks 1: \n");prwcr(calib, mt2->pid->cp);
	  printf("physical error 0: \n");prwcr(calib, mt1->pid->cd);
	  printf("physical error 1: \n");prwcr(calib, mt2->pid->cd);
	}else{
	  printf("time between ticks %d: ", port);prwcr(calib, mt->pid->cp);
	  printf("physical error %d:    ", port);prwcr(calib, mt->pid->cd);
	}
	printf("\n\n");
      }
    }

    if(pid){
      if(port < 2){
	mt_pid_on(mt);
	mt_pid_set_gains(mt,kp, ki, kd);
      } else {
	mt_pid_on(mt1);
	mt_pid_set_gains(mt1, kp, ki, kd);
	mt_pid_on(mt2);
	mt_pid_set_gains(mt2, kp, ki, kd);
      }
    }
  } else {
    printf("error in mt_init()\n");
    exit(EXIT_FAILURE);
  }


struct timespec time;
time.tv_sec = 5;
time.tv_nsec = 0;

 int ticks, thread_rtn;
 switch (tst){
 case 0:
   {
     print_usage(argv[0]);
     printf("\nAt least the test number is needed!\n");
   }
   break;;
 case 1: //Basic tests:
   {
       
     if(verb > 0) {
       printf("MOTOR %d: %d,%d,%d,%d\n",mt->id-1, (int)mt->pinf, (int)mt->pinr, mt_enc_is_null(mt,1) ? ENULL : (int)mt->enc1->pin, mt_enc_is_null(mt,2) ? ENULL : (int)mt->enc2->pin);
       printf("Motor %d: PID is %s", mt->id-1, mt_pid_is_null(mt) ? "UNACTIVE" : "ACTIVE");
       mt_pid_is_null(mt) ? printf("\n") : printf(" Kp = %.2f, Ki = %.2f, Kd = %.2f, ttc = %d\n", mt->pid->kp, mt->pid->ki, mt->pid->kd, (int)mt->pid->ttc);
       printf("turns = %d, PosCtrl = %.2f\n\n", turns, pctr);     
     }

     if(!mt_move(mt,dr, vel))
       printf("mt_move FAILED %s direction!\n", dr ? "BWD" : "FWD");
     nanosleep(&time, NULL);
     ticks = mt_stop(mt, true);
     mt_wait_for_stop(mt, 0.8);
     printf("Motor %d: ticks received: %d\n", mt->id-1, ticks);
     
     if(!mt_move(mt, !dr, vel))
       printf("mt_move FAILED %s direction!\n", dr ? "FWD" : "BWD");
     nanosleep(&time, NULL);
     ticks = mt_stop(mt, true);
     mt_wait_for_stop(mt, 0.8);
     printf("Motor %d: ticks received: %d\n", mt->id-1, ticks);
    
     mt_move_t(mt, mt_tticks(mt, turns), dr, vel, pctr); mt_wait(mt);
     ticks = mt_get_ticks(mt);
     printf("Motor %d: ticks received: %d, turns = %d, enc_active: %d\n", mt->id-1, ticks, turns, mt_enc_count(mt));
   }
   break;;
 case 2: //Test move till a set point, to ensure the user code have the program flow while motor turns.
   {  
     if(verb > 0) {
       printf("MOTOR %d: %d,%d,%d,%d\n",mt->id-1, (int)mt->pinf, (int)mt->pinr, mt_enc_is_null(mt,1) ? ENULL : (int)mt->enc1->pin, mt_enc_is_null(mt,2) ? ENULL : (int)mt->enc2->pin);
       printf("Motor %d: PID is %s", mt->id-1, mt_pid_is_null(mt) ? "UNACTIVE" : "ACTIVE");
       mt_pid_is_null(mt) ? printf("\n") : printf(" Kp = %.2f, Ki = %.2f, Kd = %.2f, ttc = %d\n", mt->pid->kp, mt->pid->ki, mt->pid->kd, (int)mt->pid->ttc);
       printf("turns = %d, PosCtrl = %.2f\n\n", turns, pctr);
     }
     
     thread_rtn = mt_move_t(mt, mt_tticks(mt, turns), dr, vel, pctr);
     while (mt->moving){
       printf("moving %d\n", mt_get_ticks(mt));
       sleep(1);
     }
     printf("pthread_create says: %d, ticks received: %d, ticks expected: %d\n", thread_rtn, mt_get_ticks(mt), mt_tticks(mt, turns));
   }
   break;;
 case 3: //interrupts handling, Test to get some stats of a rotation of one motor
   {
     RESULT out;
     int tot, encid;
       
     if(verb > 0) {
       printf("MOTOR %d: %d,%d,%d,%d\n",mt->id-1, (int)mt->pinf, (int)mt->pinr, mt_enc_is_null(mt,1) ? ENULL : (int)mt->enc1->pin, mt_enc_is_null(mt,2) ? ENULL : (int)mt->enc2->pin);
       printf("Motor %d: PID is %s", mt->id-1, mt_pid_is_null(mt) ? "UNACTIVE" : "ACTIVE");
       mt_pid_is_null(mt) ? printf("\n") : printf(" Kp = %.2f, Ki = %.2f, Kd = %.2f, ttc = %d\n", mt->pid->kp, mt->pid->ki, mt->pid->kd, (int)mt->pid->ttc);
       printf("turns = %d, PosCtrl = %.2f\n\n", turns, pctr);
     }
     
     init_acums(turns, mt);
     reset_acums(turns, mt);
     
     mt_reset_enc(mt);
     thread_rtn = mt_move_t(mt, mt_tticks(mt, turns), dr, vel, pctr);mt_wait(mt);
     
     mt_wait_for_stop(mt,2);
     tot = mt_get_ticks(mt);
     
     if(mt_enc_count(mt) == 1){
       encid = mt_enc_is_null(mt,1) ? 2 : 1;
       stats(&out,true, NULL, true, true, tot, mt->id, true, encid, verb < 2);
     } else {
       stats(&out,true, NULL, true, true, tot, mt->id, true, 1, verb < 2);
       printf("\n");
       stats(&out,true, NULL, true, true, tot, mt->id, true, 2, verb < 2);
     }   
   }
   break;;
 case 4: 
           /*Test for all the velocities from "step" to MAX_VEL [200] incrmenting the velocity "step" for each change, for every velocity                                                          the test will take "mostres" samples of "turns" turns each sample*/
   {
     int i, velo, index, alloc, encid;
     RESULT rese1[(int)((MAX_VEL/step) * mostres)];
     RESULT rese2[(int)((MAX_VEL/step) * mostres)];
     
     if(verb > 0) {
       printf("MOTOR %d: %d,%d,%d,%d\n",mt->id-1, (int)mt->pinf, (int)mt->pinr, mt_enc_is_null(mt,1) ? ENULL : (int)mt->enc1->pin, mt_enc_is_null(mt,2) ? ENULL : (int)mt->enc2->pin);
       printf("Motor %d: PID is %s", mt->id-1, mt_pid_is_null(mt) ? "UNACTIVE" : "ACTIVE");
       mt_pid_is_null(mt) ? printf("\n") : printf(" Kp = %.2f, Ki = %.2f, Kd = %.2f, ttc = %d\n", mt->pid->kp, mt->pid->ki, mt->pid->kd, (int)mt->pid->ttc);
       printf("turns = %d, PosCtrl = %.2f\n\n", turns, pctr);
     }
     
     init_acums(turns, mt);
     reset_acums(turns, mt);
     for (velo = step; velo <= MAX_VEL; velo+=step){
       for (i = 0; i < mostres; i++){
	 index = ((((velo/step)-1)*mostres) + i);
	 if ((index%50 == 0) && (index != 0))
	   printf("stored samples: %d\n", index);
	 mt_reset_enc(mt);
	 mt_move_t(mt, mt_tticks(mt, turns), dr, velo, 0);mt_wait(mt);
	 alloc = mt_get_ticks(mt);
	 mt_wait_for_stop(mt,2);
	 if(mt_enc_count(mt) == 1){
	   encid = mt_enc_is_null(mt,1) ? 2 : 1;
	   stats(encid == 1 ? &rese1[index] : &rese2[index], verb < 2, NULL, true, true, alloc, mt->id, true, encid, verb < 3);
	 } else {
	   stats(&rese1[index], verb > 1, NULL, true, true, alloc, mt->id, true,1, verb < 3);
	   stats(&rese2[index], verb > 1, NULL, true, true, alloc, mt->id, true,2, verb < 3);
	 }
	 reset_acums(turns, mt);
       }
     }
     
     printf ("\nMOTOR %d, STEP: %d, SAMPLES %d x step:\n\n\t\t\tENC 1:\n", mt->id, step, mostres);
     if(!mt_enc_is_null(mt,1))
       comp_res(rese1, mostres, step, mt->id);
     printf ("\n\n\t\t\tENC: 2\n");
     if(!mt_enc_is_null(mt,2))
       comp_res(rese2, mostres, step, mt->id);
   }
   break;;
 case 5:
       /*Test for one velocity, the test will turn from "step" turns to MAXM (40) turns at "vel" velocity,                                                                                    used to see if the ticks per turn are stables, hence the velocities 'real'*/
   {
     
     TSPEC tini, taux;
     long enano;
     double txturn, tbticks, elapsed, txsec;
     long long int elmicras;
     int esec, index, ticks;
     RESULT res[MAXM/step], res2[MAXM/step];
     RFIVE data [MAXM/step];
     bool to_pr = verb >= 1;

     if(verb > 0) {
       printf("MOTOR %d: %d,%d,%d,%d\n",mt->id-1, (int)mt->pinf, (int)mt->pinr, mt_enc_is_null(mt,1) ? ENULL : (int)mt->enc1->pin, mt_enc_is_null(mt,2) ? ENULL : (int)mt->enc2->pin);
       printf("Motor %d: PID is %s", mt->id-1, mt_pid_is_null(mt) ? "UNACTIVE" : "ACTIVE");
       mt_pid_is_null(mt) ? printf("\n") : printf(" Kp = %.2f, Ki = %.2f, Kd = %.2f, ttc = %d\n", mt->pid->kp, mt->pid->ki, mt->pid->kd, (int)mt->pid->ttc);
       printf("turns = %d, PosCtrl = %.2f\n\n", turns, pctr);
     }
     
     init_acums((int)MAXM, mt);
     
     for (turns = step; turns <= MAXM; turns += step){
       
       index = ((turns/step)-1);
       mt_wait_for_stop(mt, 1);
       reset_acums((int)MAXM, mt);
       mt_reset_enc(mt);
       clock_gettime(CLK_ID, &tini);
       mt_move_t(mt, mt_tticks(mt, turns), dr, vel,0);mt_wait(mt);
       ticks = mt_get_ticks(mt);
       clock_gettime(CLK_ID, &taux);
       enano = (taux.tv_nsec - tini.tv_nsec);
       esec = (int)(taux.tv_sec - tini.tv_sec);
       elapsed = esec+(enano*0.000000001);
       elmicras = (long long int)((enano/1000) + (long long int)(1000000 * esec));
       txsec = (ticks / elapsed);
       txturn = (elmicras / turns);
       tbticks = (txturn/mt->ticsxturn);
       if(to_pr)
	 printf("turns: %d, telapsed: %.5lf sec\ntxturn: %.5lf sec\ntbticks_total: %.5lf micros\nticks/sec: %.5lf\n", turns, elapsed , txturn/1000000, tbticks, txsec);
       data[index].tturn = txturn;
       data[index].txsec = txsec;
       if(mt_enc_count(mt) == 1){
	 int encid = mt_enc_is_null(mt,1) ? 2 : 1;
	 if(encid == 1){
	   stats(&res[index], verb >= 2, NULL, true, true, ticks, mt->id, true, encid, verb < 3);
	   if(to_pr)
	     printf("\nenc1,\nmean:\t     %f\ntbticks_e1: %.5f\nrange_min:  %f\nrange_max:  %f\n", res[index].res[0],(double)(elmicras/mt->enc1->tics), res[index].res[14], res[index].res[13]);
	   data[index].e1[0] = res[index].res[0];
	   data[index].e1[1] = (elmicras/mt->enc1->tics);
	   data[index].e1[2] = res[index].res[14];
	   data[index].e1[3] = res[index].res[13];
	 }else{
	   stats(&res2[index], verb >= 2, NULL, true, true, ticks, mt->id, true, encid, verb < 3);
	   if(to_pr)
	     printf("\nenc2,\nmean:\t     %f\ntbticks_e2: %.5f\nrange_min:  %f\nrange_max:  %f\n", res2[index].res[0],(double)(elmicras/mt->enc2->tics), res2[index].res[14], res2[index].res[13]);
	   data[index].e2[0] = res2[index].res[0];
	   data[index].e2[1] = (elmicras/mt->enc2->tics);
	   data[index].e2[2] = res2[index].res[14];
	   data[index].e2[3] = res2[index].res[13];
	   
	 }
       } else {
	 stats(&res[index], verb >= 2, NULL, true, true, ((ticks/2)+5), mt->id, true,1, verb < 3); 
	 if(to_pr)
	   printf("\nenc1,\nmean:\t    %f\ntbticks_e1: %.5f\nrange_min:  %f\nrange_max:  %f\n", res[index].res[0], (double)(elmicras/mt->enc1->tics), res[index].res[14], res[index].res[13]);
	 data[index].e1[0] = res[index].res[0];
	 data[index].e1[1] = (elmicras/mt->enc1->tics);
	 data[index].e1[2] = res[index].res[14];
	 data[index].e1[3] = res[index].res[13];
	 
	 stats(&res2[index], verb >= 2, NULL, true, true, ((ticks/2)+5), mt->id, true,2, verb < 3);
	 if(to_pr)
	   printf("\nenc2,\nmean:\t    %f\ntbticks_e2: %.5f\nrange_min:  %f\nrange_max:  %f\n", res2[index].res[0],(double)(elmicras/mt->enc2->tics), res2[index].res[14], res2[index].res[13]);
	 data[index].e2[0] = res2[index].res[0];
	 data[index].e2[1] = (elmicras/mt->enc2->tics);
	 data[index].e2[2] = res2[index].res[14];
	 data[index].e2[3] = res2[index].res[13];
	 
       }
       if(to_pr)
	 printf("\n------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");

     }
     printf("MOTOR: %d, step: %d, vel: %d\n\n", mt->id, step, vel);
     prfive(data, MAXM/step);
     free_acums(mt);
     
   }
   break;;
 case 6: 
        /* Test for motor reconf, we will se here if we actually are able to disable encoder lines on demand, and also, change the ISR of each line to whatever function */
   {
     ENC * e1 = mt->enc1;
     ENC * e2 = mt->enc2;
     int pin1 = mt->id == 1 ? M1_ENC1 : M2_ENC1;
     int pin2 = mt->id == 1 ? M1_ENC2 : M2_ENC2;
     int e2pin, e1pin;
     ENC e2aux, e1aux;
     
     if(verb > 0) {
       printf("MOTOR %d: %d,%d,%d,%d\n",mt->id-1, (int)mt->pinf, (int)mt->pinr, mt_enc_is_null(mt,1) ? ENULL : (int)mt->enc1->pin, mt_enc_is_null(mt,2) ? ENULL : (int)mt->enc2->pin);
       printf("Motor %d: PID is %s", mt->id-1, mt_pid_is_null(mt) ? "UNACTIVE" : "ACTIVE");
       mt_pid_is_null(mt) ? printf("\n") : printf(" Kp = %.2f, Ki = %.2f, Kd = %.2f, ttc = %d\n", mt->pid->kp, mt->pid->ki, mt->pid->kd, (int)mt->pid->ttc);
       printf("turns = %d, PosCtrl = %.2f\n\n", turns, pctr);
     }
     
     printf("both encoders active, m_e1: %d, m_e2: %d\n", e1->pin, e2->pin);
     mt_move_t(mt, mt_tticks(mt, turns), dr, vel, pctr); mt_wait(mt);
     printf("ticks received: %d, ticks expected: %d, tics e1: %d, tics e2: %d\n", mt_get_ticks(mt), mt_tticks(mt, turns), e1->tics, e2->tics);
     printf("disabling encoder 2 ... \n");
     mt_wait_for_stop(mt,2);
     mt_reconf(mt, NULL, ECNULL); //e1 untouched.
     e2pin = mt_enc_is_null(mt,2) ? ENULL : e2->pin;
     printf("m_e2 disabled , m_e1: %d, m_e2: %d,\n", e1->pin, e2pin);
     
       
     mt_reset_enc(mt);
     mt_move_t(mt, mt_tticks(mt, turns), dr, vel, pctr); mt_wait(mt);
     printf("ticks received: %d, ticks expected: %d, tics e1: %d, tics e2: %d\n",  mt_get_ticks(mt), mt_tticks(mt, turns), e1->tics, mt_enc_is_null(mt,2) ? 0 : e2->tics);
     printf("re-enabling encoder 2 custom ISR ...\n");
     mt_wait_for_stop(mt,2);
     e2aux.pin = pin2;
     e2aux.isr = mt->id == 1 ? &isr_print_12 : &isr_print_22; 
     mt_reconf(mt, NULL, &e2aux); // e1 untouched
     printf("both encoders active, m_e1: %d, m_e2: %d\n", e1->pin, e2->pin);
     
     
     mt_reset_enc(mt);
     mt_move_t(mt, mt_tticks(mt, turns), dr, vel, 0);mt_wait(mt);
     printf("ticks received: %d, ticks expected: %d, tics e1: %d, tics e2: %d\n", mt_get_ticks(mt), mt_tticks(mt, turns), e1->tics, mt_enc_is_null(mt,2) ? 0 : e2->tics);
     printf("disabling encoder 1 ... \n");
     mt_wait_for_stop(mt,2);
     mt_reconf(mt, ECNULL, NULL); //e2 untouched.
     e1pin = mt_enc_is_null(mt,1) ? ENULL : e1->pin;
     printf("m_e1 disabled , m_e1: %d, m_e2: %d,\n", e1pin, e2->pin);
     
  
     mt_reset_enc(mt);
     mt_move_t(mt, mt_tticks(mt, turns), dr, vel, 0);mt_wait(mt);
     printf("ticks received: %d, ticks expected: %d, tics e1: %d, tics e2: %d\n",  mt_get_ticks(mt), mt_tticks(mt, turns), mt_enc_is_null(mt,1) ? 0 : e1->tics, e2->tics);
     printf("re-enabling encoder 1 custom ISR ...\n");
     e1aux.pin = pin1;
     e1aux.isr = mt->id == 1 ? &isr_print_11 : &isr_print_21;
     mt_wait_for_stop(mt,2);
     mt_reconf(mt, &e1aux, NULL); // e2 untouched
     printf("both encoders active, m_e1: %d, m_e2: %d,\n", e1->pin, e2->pin);
     
      
     mt_reset_enc(mt);
     mt_move_t(mt, mt_tticks(mt, turns), dr, vel, 0);mt_wait(mt);
     printf("ticks received: %d, ticks expected: %d, tics e1: %d, tics e2: %d\n",  mt_get_ticks(mt), mt_tticks(mt, turns), e1->tics, e2->tics);
     printf("back to defaults ...\n");
     mt_wait_for_stop(mt,2);
     mt_reconf(mt, NULL, NULL);
     printf("both encoders active, (default ISRs) m_e1: %d, m_e2: %d,\n", e1->pin, e2->pin);
     
     mt_reset_enc(mt);
     mt_move_t(mt, mt_tticks(mt, turns), dr, vel, 0);mt_wait(mt);
     printf("ticks received: %d, ticks expected: %d, tics e1: %d, tics e2: %d\n",  mt_get_ticks(mt), mt_tticks(mt, turns), mt_enc_is_null(mt,1) ? 0 : e1->tics, e2->tics);
     printf("trying to disable both encoders at a time ...\n");
     mt_wait_for_stop(mt,2);
     mt_reconf(mt, ECNULL, ECNULL);
   }
   break;;
 case 7: /* Test for calibration, step will be overwritten here, and samples must range between 5 and 20, the values will be computed 
            from velocity 20 to MAX_VEL (200) in steps of "MAX_VEL/samples", after the calibration the test will interpolate the data set 
	    for the middle points values*/
   {
    
     double twait = 1.7;
     int micras   = 0, desv = 0, micras2 = 0, desv2 = 0, mtot = 0, dtot = 0, i;
     step         = (MAX_VEL / mostres);
     
     if (mostres < 5)
       mostres = 5;
     else if (mostres > 20)
       mostres = 20;
     
     if(verb > 0) {
       if(port<2) 
	 printf("MOTOR %d: %d,%d,%d,%d\n",mt->id-1, (int)mt->pinf, (int)mt->pinr, mt_enc_is_null(mt,1) ? ENULL : mt->enc1->pin, mt_enc_is_null(mt,2) ? ENULL : mt->enc2->pin);	 
       else {
	 printf("MOTOR 0: %d,%d,%d,%d\n", (int)mt1->pinf, (int)mt1->pinr, mt_enc_is_null(mt1,1) ? ENULL : mt1->enc1->pin, mt_enc_is_null(mt1,2) ? ENULL : mt1->enc2->pin);	 
	 printf("MOTOR 1: %d,%d,%d,%d\n", (int)mt2->pinf, (int)mt2->pinr, mt_enc_is_null(mt2,1) ? ENULL : mt2->enc1->pin, mt_enc_is_null(mt2,2) ? ENULL : mt2->enc2->pin);	 
       }

       printf("PID is %s", mt_pid_is_null(port == 2 ? mt1 : mt) ? "UNACTIVE" : "ACTIVE");
       mt_pid_is_null(port == 2 ? mt1 : mt) ? printf("\n") : printf(" Kp = %.2f, Ki = %.2f, Kd = %.2f, ttc = %d\n", port == 2 ? mt1->pid->kp : mt->pid->kp, port == 2 ? mt1->pid->ki : mt->pid->ki, port == 2 ? mt1->pid->kd : mt->pid->kd, port == 2 ? (int)mt1->pid->ttc : (int)mt->pid->ttc);
       printf("step = %d, samples = %d\n\n", step, mostres);
     }
     
     port < 2 ? printf("Calibrating MOTOR %d\n\n", port) : printf("Calibrating MOTORS\n\n") ;     
     if(mt_calibrate(mostres, twait)){
       
       if(verb >= 1) {
	 if(port == 2){
	   printf("time between ticks 0: \n");prwcr(mostres, mt1->pid->cp);
	   printf("time between ticks 1: \n");prwcr(mostres, mt2->pid->cp);
	   printf("physical error 0: \n");prwcr(mostres, mt1->pid->cd);
	   printf("physical error 1: \n");prwcr(mostres, mt2->pid->cd);
	 } else {
	   printf("time between ticks %d: \n", port);prwcr(mostres, mt->pid->cp);
	   printf("physical error %d: \n", port);prwcr(mostres, mt->pid->cd);
	 }
	 printf("\n");
       }
       
       for ( i =  MIN_VEL+(step/2); i <= MAX_VEL; i += step ){
	 if (i == MAX_VEL)
	   i = (MAX_VEL - (step/4));
	 if(port < 2)
	   gt_pars(mt,i,&mtot, &dtot);
	 else {
	   gt_pars(mt1,i,&micras, &desv);
	   gt_pars(mt2,i,&micras2, &desv2);
	   mtot = (micras + micras2)/2;
	   dtot = (desv + desv2)/2;
	 }
	 if(port < 2)
	   printf("params for %3d vel >> tbticks: %d, err: %d\n", i, mtot, dtot);
	 else {
	   printf("MOTOR 0: params for %3d vel >> tbticks: %d, err: %d\n", i, micras, desv);
	   printf("MOTOR 1: params for %3d vel >> tbticks: %d, err: %d\n", i, micras2, desv2);
	   //printf(" %d, %d\n", mtot, dtot); //AVG    : params for %3d vel >> 
	 }
       }
     } else 
       printf("Error calibrating motors.\n");
       
   }
   break;;
   case 8: //This test is useless to the user, however it was implemented to get the default time between ticks and physical error values for the library 
     {
       int i, k;
       double twait = 0.7;
       int iters = 5;
       int micras1 = 0, desv1 = 0, micras2 = 0, desv2 = 0;
       step = (MAX_VEL / mostres);
       double sum1[mostres], sum2 [mostres], defp1[mostres], defp2[mostres], defd1[mostres], defd2[mostres];
       
       if(verb > 0) {
       if(port<2) 
	 printf("MOTOR %d: %d,%d,%d,%d\n",mt->id-1, (int)mt->pinf, (int)mt->pinr, mt_enc_is_null(mt,1) ? ENULL : mt->enc1->pin, mt_enc_is_null(mt,2) ? ENULL : mt->enc2->pin);	 
       else {
	 printf("MOTOR 0: %d,%d,%d,%d\n", (int)mt1->pinf, (int)mt1->pinr, mt_enc_is_null(mt1,1) ? ENULL : mt1->enc1->pin, mt_enc_is_null(mt1,2) ? ENULL : mt1->enc2->pin);	 
	 printf("MOTOR 1: %d,%d,%d,%d\n", (int)mt2->pinf, (int)mt2->pinr, mt_enc_is_null(mt2,1) ? ENULL : mt2->enc1->pin, mt_enc_is_null(mt2,2) ? ENULL : mt2->enc2->pin);	 
       }

       printf("PID is %s", mt_pid_is_null(port == 2 ? mt1 : mt) ? "UNACTIVE" : "ACTIVE");
       mt_pid_is_null(port == 2 ? mt1 : mt) ? printf("\n") : printf(" Kp = %.2f, Ki = %.2f, Kd = %.2f, ttc = %d\n", port == 2 ? mt1->pid->kp : mt->pid->kp, port == 2 ? mt1->pid->ki : mt->pid->ki, port == 2 ? mt1->pid->kd : mt->pid->kd, port == 2 ? (int)mt1->pid->ttc : (int)mt->pid->ttc);
       printf("step = %d, samples = %d\n\n", step, mostres);
     }

       for(i = 0; i < iters; i++){
	 mt_calibrate(mostres, twait);
	 if(verb >= 3) {
	   if(port == 2){
	     printf("time between ticks 0: \n");prwcr(mostres, mt1->pid->cp);
	     printf("time between ticks 1: \n");prwcr(mostres, mt2->pid->cp);
	     printf("physical error 0: \n");prwcr(mostres, mt1->pid->cd);
	     printf("physical error 1: \n");prwcr(mostres, mt2->pid->cd);
	   } else {
	     printf("time between ticks %d: \n", port);prwcr(mostres, mt->pid->cp);
	     printf("physical error %d: \n", port);prwcr(mostres, mt->pid->cd);
	   }
	   printf("\n");
	 }

	 for (k = 0; k<mostres; k++){
	   if (port == 2){
	     defp1[k] += mt1->pid->cp[k];
	     defd1[k] += mt1->pid->cd[k];
	     defp2[k] += mt2->pid->cp[k];
	     defd2[k] += mt2->pid->cd[k];
	     sum1[k] += (mt1->pid->cp[k] + mt2->pid->cp[k])/2;
	     sum2[k] += (mt1->pid->cd[k] + mt2->pid->cd[k])/2;
	   } else {
	     defp1[k] += mt->pid->cp[k];
	     defd1[k] += mt->pid->cd[k];
	   }
	 }
	 if(verb >= 2) {
	   for (k = MIN_VEL+(step/2); k < MAX_VEL; k += step){
	     if (k == MAX_VEL)
	       k = (MAX_VEL - (step/4));
	     if(port < 2)
	       gt_pars(mt,i,&micras1, &desv1);
	     else {
	       gt_pars(mt1,i,&micras1, &desv2);
	       gt_pars(mt2,i,&micras2, &desv2);
	     }
	     
	     if(port < 2)
	       printf("params for %2d%% power mot 1 >> tbticks: %d, desv: %d\n", k, micras1, desv1);
	     else {
	       printf("MOTOR 0: params for %2d%% power mot 1 >> tbticks: %d, desv: %d\n",k,micras1, desv1);
	       printf("MOTOR 1: params for %2d%% power mot 2 >> tbticks: %d, desv: %d\n",k,micras2, desv2);
	     }
	   }
	   printf("\n"); 
	 }
	
       }
       
       for (i = 0; i < mostres; i++){
	 if(port == 2){
	   defp1[i] = defp1[i]/iters;
	   defd1[i] = defd1[i]/iters;
	   defp2[i] = defp2[i]/iters;
	   defd2[i] = defd2[i]/iters;
	   sum1[i] = sum1[i]/iters;
	   sum2[i] = sum2[i]/iters;
	 } else {
	   defp1[i] = defp1[i]/iters;
	   defd1[i] = defd1[i]/iters;
	 }
       }
       
       if(port == 2){
	 printf("DEFAULTS TBT_M0: ");prwcr(mostres, defp1);
	 printf("DEFAULTS ERR_M0: ");prwcr(mostres, defd1);
	 printf("DEFAULTS TBT_M1: ");prwcr(mostres, defp2);
	 printf("DEFAULTS ERR_M1: ");prwcr(mostres, defd2);
	 printf("DEFAULTS TBT: ");prwcr(mostres, sum1);
	 printf("DEFAULTS ERR: ");prwcr(mostres, sum2);
       } else {
	 printf("DEFAULTS TBT_M%d:", port);prwcr(mostres, defp1);
	 printf("DEFAULTS ERR_M%d:", port);prwcr(mostres, defd1);
       }
     }
     break;;
 case 9: //Very simple test for one motor to test P.I.D behavior.
   {
     if(verb > 0) {
       printf("MOTOR 0: %d,%d,%d,%d\n", (int)mt1->pinf, (int)mt1->pinr, mt_enc_is_null(mt1,1) ? ENULL : mt1->enc1->pin, mt_enc_is_null(mt1,2) ? ENULL : mt1->enc2->pin);	 
       printf("MOTOR 1: %d,%d,%d,%d\n", (int)mt2->pinf, (int)mt2->pinr, mt_enc_is_null(mt2,1) ? ENULL : mt2->enc1->pin, mt_enc_is_null(mt2,2) ? ENULL : mt2->enc2->pin);
       printf("PID is %s", mt_pid_is_null(mt1) ? "UNACTIVE" : "ACTIVE");
       mt_pid_is_null(mt1) ? printf("\n") : printf(" Kp = %.2f, Ki = %.2f, Kd = %.2f, ttc = %d\n", mt1->pid->kp , mt1->pid->ki, mt1->pid->kd , (int)mt1->pid->ttc);
       printf("step = %d, samples = %d\n\n", step, mostres);
     }
     
     if(mt_pid_is_null(mt))
       printf("This tests is meant to test P.I.D, and P.I.D is null...\n DEBUG [-l] mode is recommended too...\n");
     
     printf("\n\nSTARTING MOVE_T\n\n");
     mt_move_t(mt, mt_tticks(mt, turns), dr, vel, pctr);mt_wait(mt);
   }
   
   break;;
 case 10://Very simple test to test move sinc, when 1 is entered to the standard input, the motors will stop turning 
   { 
     
     if(verb > 0) {
       printf("MOTOR 0: %d,%d,%d,%d\n", (int)mt1->pinf, (int)mt1->pinr, mt_enc_is_null(mt1,1) ? ENULL : mt1->enc1->pin, mt_enc_is_null(mt1,2) ? ENULL : mt1->enc2->pin);	 
       printf("MOTOR 1: %d,%d,%d,%d\n", (int)mt2->pinf, (int)mt2->pinr, mt_enc_is_null(mt2,1) ? ENULL : mt2->enc1->pin, mt_enc_is_null(mt2,2) ? ENULL : mt2->enc2->pin);
       printf("PID is %s", mt_pid_is_null(mt1) ? "UNACTIVE" : "ACTIVE");
       mt_pid_is_null(mt1) ? printf("\n") : printf(" Kp = %.2f, Ki = %.2f, Kd = %.2f, ttc = %d\n", mt1->pid->kp , mt1->pid->ki, mt1->pid->kd , (int)mt1->pid->ttc);
       printf("step = %d, samples = %d\n\n", step, mostres);
     }
     
     if(!log_dbg)
       printf("DEBUG [-l] mode is recommended ...\n");
     
     mt_move_sinc(dr, vel);
     while(get_in("stop?", 1) != 1);
     mt_stop(mt1,true);
     mt_stop(mt2,true);
   }
   break;;
 case 11: //Simple test to test sincro till a set point
   {
     
     if(verb > 0) {
       printf("MOTOR %d: %d,%d,%d,%d\n",mt->id-1, (int)mt->pinf, (int)mt->pinr, mt_enc_is_null(mt,1) ? ENULL : (int)mt->enc1->pin, mt_enc_is_null(mt,2) ? ENULL : (int)mt->enc2->pin);
       printf("Motor %d: PID is %s", mt->id-1, mt_pid_is_null(mt) ? "UNACTIVE" : "ACTIVE");
       mt_pid_is_null(mt) ? printf("\n") : printf(" Kp = %.2f, Ki = %.2f, Kd = %.2f, ttc = %d\n", mt->pid->kp, mt->pid->ki, mt->pid->kd, (int)mt->pid->ttc);
       printf("turns = %d, PosCtrl = %.2f\n\n", turns, pctr);
     }
     
     if(!log_dbg)
       printf("DEBUG [-l] mode is recommended, and redirect the output to a file too ...\n");
     
     printf("SINCRO: Set point = %d ticks\n", (turns*720));
     int res = mt_move_sinc_t(dr, vel, (turns*720), pctr);mt_wait_all();
     printf("move_sinc-says: \"%s\"\n", res ? "OK" : "FAIL");
     mt_stop(mt1,true);
     mt_stop(mt2,true);
   }
   break;;
 default:
   break;;
 }
 
 if(port < 2)
   mt_stop(mt, true);
 else {
   mt_stop(mt1, true);
   mt_stop(mt2, true);
 }
 mt_shutdown();
 
 return ret;
 
}

bool gt_pars (MOTOR * m, int vel, int * ex_micras, int * ex_desv){
  
    int size, i;
    for (size = 0; m->pid->cp[size] != 0; size++);
    double x[size];
    int v = vel < MIN_VEL ? MIN_VEL : vel > MAX_VEL ? MAX_VEL : vel;
    int *res;
    gsl_interp *interp;
    
    *ex_micras = 0;
    *ex_desv = 0;
    
    x[0] = MIN_VEL;
    for(i = 1; i < size; i++)
      x[i] = i != size-1 ? MIN_VEL + (i * m->pid->svel) : MAX_VEL;
    
    interp = gsl_interp_alloc(gsl_interp_akima_periodic, size);
  
    for(i = 0; i < 2; i++){
      res = i == 0 ? ex_micras : ex_desv; 
      gsl_interp_init(interp, x, i == 0 ? m->pid->cp : m->pid->cd, size);
      *res = (int)gsl_interp_eval(interp, x, i == 0 ? m->pid->cp : m->pid->cd, (double)v, NULL);
    }
    
    gsl_interp_free(interp);
 
  return true;
}

double difft (TSPEC * ini, TSPEC * fi){

    long enano = (fi->tv_nsec - ini->tv_nsec);
    int esec = (int)(fi->tv_sec - ini->tv_sec);
    return ((double) esec*1000000+(enano/1000)); //micras

}

void prfive(RFIVE * data, int len){

  int i;
  double tturn_acum [len], avg_tturn, absd_tturn, txs_acum [len], avg_txs, absd_txs;
  double * en1[len];
  double * en2[len];
  RESULT statturn, stattxs;
  
  for(i = 0; i < len; i++){
    tturn_acum [i] = data[i].tturn;
    txs_acum [i] = data[i].txsec;
    en1[i] = data[i].e1;
    en2[i] = data[i].e2;
  }
  stats(&statturn, false, tturn_acum, false, false, len, 0, false,0,false);
  stats(&stattxs, false, txs_acum, false, false, len, 0, false,0,false);
  avg_tturn = statturn.res[0];
  absd_tturn = statturn.res[3];
  avg_txs = stattxs.res[0];
  absd_txs = stattxs.res[3];
  printf("\t\t\t AVERAGE\t\tABS_DEV\t\t\n\n");
  printf("secs  x turn:\t\t %.5f\t\t%.5f\n", avg_tturn*0.000001, absd_tturn*0.000001);
  printf("ticks x sec :\t\t %.5f\t\t%.5f\n\n", avg_txs, absd_txs);
  printf("enc: 1, \n");
  tr_enc(en1, len);
  printf("enc: 2, \n");
  tr_enc(en2, len);
}

void tr_enc (double * e[], int mostres){

int k;
double aux[mostres], min_r = 0, max_r = 0, mean, absd;
RESULT stat;

 for(k=0;k<4;k++){
   col_to_row(aux, e, k, mostres);
   stats(&stat, false, aux, false, false, mostres, 0, false,0, true);
   mean = stat.res[0];
   absd = stat.res[3];
   if(k == 2)
     min_r = gsl_stats_min (aux, 1, mostres);
   
   if(k == 3)
     max_r = gsl_stats_max (aux, 1, mostres);
   
   switch (k){
   case 0:
     printf("average: \t\t%.5f\t\t%.5f\n", mean, absd);
     break;;
   case 1:
     printf("t_btwn_ticks: \t\t%.5f\t\t%.5f\n", mean, absd);
     break;;
   case 2:
     printf("range_min: \t\t%.5f\t\t%.5f\n", mean, absd);
     break;;
   case 3:
     printf("range_max: \t\t%.5f\t\t%.5f\n", mean, absd);
     printf("min_rmin: \t\t%.5f\n", min_r);
     printf("max_rmax: \t\t%.5f\n", max_r);
     break;;
   }
 }
 printf("\n");
}


void init_acums (int turns, MOTOR * m){

    int size = (int)(BASE*turns);
    if(m->id == 1){
      if(!mt_enc_is_null(m,1)) {
            if( (acum1 = (double *)calloc(size,sizeof(double))) == NULL)
				printf("Init ac1 fails\n");
			}
      if(!mt_enc_is_null(m,2)) {
            if( (acum3 = (double *)calloc(size,sizeof(double))) == NULL)
				printf("Init ac3 fails\n");
			}
    } else {
      if(!mt_enc_is_null(m,1)) {
            if( (acum2 = (double *)calloc(size,sizeof(double))) == NULL)
				printf("Init ac2 fails\n");
		}
      if(!mt_enc_is_null(m,2)) {
            if( (acum4 = (double *)calloc(size,sizeof(double))) == NULL)
				printf("Init ac4 fails\n");
		}
    }
}

void free_acums(MOTOR * m){
  
  if( m->id == 1 ){
    if(!mt_enc_is_null(m,1))
      free(acum1);
    if(!mt_enc_is_null(m,2))
      free(acum3);
  } else {
    if(!mt_enc_is_null(m,1))
      free(acum2);
    if(!mt_enc_is_null(m,2))
      free(acum4);
  }
}

void reset_acums(int turns, MOTOR * m){


    int size = (int)(BASE*turns);
    if( m->id == 1 ){
      if(!mt_enc_is_null(m,1))
            memset(acum1,0.0,size*sizeof(double));
      if(!mt_enc_is_null(m,2))
            memset(acum3,0.0,size*sizeof(double));
    } else {
      if(!mt_enc_is_null(m,1))
            memset(acum2,0.0,size*sizeof(double));
      if(!mt_enc_is_null(m,2))
            memset(acum4,0.0,size*sizeof(double));
    }
}

int get_in(char *to_print, int type){
    switch(type){
    case 1:
      {
        int ret_int;
        printf("%s", to_print);
        scanf("\n%d",&ret_int);
        fflush(NULL);
        return ret_int;
      }
      break;;
    default:
      {
        char ret;
        printf("%s", to_print);
        scanf("\n%c",&ret);
        fflush(NULL);
        return ret;
      }
      break;;
    }
}



void comp_res(RESULT * res, int mostres, int step, int id){
  
  int i, velo, index;
  double * frame[mostres]; 
  
  printf("\t\t\t\t\t AVERAGE\t\tVARIANCE\t\tSTND_DEV\t\tABS_DEV\t\t\n\n");
  
  for (velo = step; velo <= MAX_VEL; velo += step){
    for (i = 0; i < mostres; i++){
      index = ((((velo/step)-1)*mostres) + i);
      frame[i] = res[index].res;
    }
    comp_vel(velo, mostres, frame, id);
    
    for (i = 0; i < mostres; i++){
      index = ((((velo/step)-1)*mostres) + i);
      frame[i] = res[index].per;
    }
    comp_perc(velo, mostres, frame, id);
    printf("\n");
  }
  printf("---------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
}

void comp_perc(int vel, int mostres, double ** frame, int id){
  
  int k;
  double sum = 0;
  double aux[mostres];
  RESULT stat;
  
  for (k=0;k<PER_SIZE-1;k++){
    col_to_row(aux, frame, k, mostres);
    stats(&stat, false, aux, false, false, mostres, id, false,0, true);
    sum += stat.res[0];
    switch(k){
    case 0:
      printf("vel_%d_0%%-10%%:  \t", vel);
      break;;
    case 1:
      printf("vel_%d_10%%-20%%: \t", vel);
      break;;
    case 2:
      printf("vel_%d_20%%-30%%: \t",vel);
      break;;
    case 3:
      printf("vel_%d_30%%-40%%: \t", vel);
      break;;
    case 4:
      printf("vel_%d_40%%-50%%: \t", vel);
      break;;
    case 5:
      printf("vel_%d_50%%-60%%: \t", vel);
      break;;
    case 6:
      printf("vel_%d_60%%-70%%: \t", vel);
      break;;
    case 7:
      printf("vel_%d_70%%-80%%: \t", vel);
      break;;
    case 8:
      printf("vel_%d_80%%-90%%: \t", vel);
      break;;
    case 9:
      printf("vel_%d_90%%-100%%: \t", vel);
      break;;
    default:
      printf("\t\t>>WEIRD_DATA!!<<\t\t\n");
      break;;
    }
    pr_stats(stat.res, 4);
  }
  printf ("population_average_sum: \t\t%.5f\n", sum);
}

void comp_vel(int vel, int mostres, double ** frame, int id){
  int k;
  double aux[mostres];
  RESULT stat;
  
  for (k=0;k<STATS_SIZE-1;k++){
    col_to_row (aux, frame, k, mostres);
    stats(&stat, false, aux, false, false, mostres, id, false,0, true);
    switch(k){
    case 0:
      printf("vel_%d_average: \t", vel);
      break;;
    case 1:
      printf("vel_%d_variance: \t", vel);
      break;;
    case 2:
      printf("vel_%d_standard deviation: ",vel);
      break;;
    case 3:
      printf("vel_%d_absolute deviation: ", vel);
      break;;
    case 4:
      printf("vel_%d_autocorrelation: ", vel);
      break;;
    case 5:
      printf("vel_%d_waverage: \t", vel);
      break;;
    case 6:
      printf("vel_%d_wvariance: \t", vel);
      break;;
    case 7:
      printf("vel_%d_wstandard deviation: ", vel);
      break;;
    case 8:
      printf("vel_%d_wabsolute deviation: ", vel);
      break;;
    case 9:
      printf("vel_%d_swaverage: \t", vel);
      break;;
    case 10:
      printf("vel_%d_swvariance: \t", vel);
      break;;
    case 11:
      printf("vel_%d_swstandard deviation: ", vel);
      break;;
    case 12:
      printf("vel_%d_swabsolute deviation: ", vel);
      break;;
    case 13:
      printf("vel_%d_upperq:  \t", vel);
      break;;
    case 14:
      printf("vel_%d_lowerq:  \t", vel);
      break;;
    case 15:
      printf("vel_%d_rmax:  \t\t", vel);
      break;;
    case 16:
      printf("vel_%d_rmin:  \t\t", vel);
      break;;
    default:
      printf("\t\t>>WEIRD DATA!!<<\t\t\n");
      break;;
    }
    pr_stats(stat.res, 4);
  }
}

void pr_stats(double * stats, int len){
  
  int i, dig = 0;
  printf("\t\t");
  for(i = 0; i<len; i++){
    dig = floor(log10(abs((int)stats[i]))) + 1;
    if (dig > 8)
      printf("%.5f,\t", stats[i]);
    else if (stats[i] != stats[i]) //is nan?
      printf("%.5f,\t\t\t", stats[i]);
    else
      printf("%.5f,\t\t", stats[i]);
  }
  printf("\n");
}

void col_to_row(double * res, double ** table, int col, int rows){
  
  int k;
  for(k=0; k<rows; k++){
    if ( col == 15 || col == 16 ){
      if ( table[k][col] == 0 )
	res[k] = table[k][col-2];
    } else
      res[k] = table[k][col];
  }
  res[k] = '\0';
}


void presf (RESULT * res, int mostres, int step){
  
  int i, k, velo, index;
  for (velo = step; velo <= MAX_VEL; velo += step){
    printf("\nVELOCIDAD  = %d:\n", velo);
    for (i = 0; i < mostres; i++){
      index = ((((velo/step)-1)*mostres) + i);
      printf("mostra %2d,(%3d): ", i+1, index);
      for (k = 0; k < STATS_SIZE-1; k++)
	printf("%.5f, ", res[index].res[k]);
      printf("\n");
    }
    printf("percent:\n");
    for (i = 0; i < mostres; i++){
      index = ((((velo/step)-1)*mostres) + i);
      for (k = 0; k < 10; k++)
	printf("%.5f, ", res[index].per[k]);
      printf("\n");
    }
  }
}


int cpacum(double * out, int alloc, int id, int encoder){
  
  int i;
  if (id == 1){
    if (encoder == 1){
      for(i=1;(acum1[i]!='\0' && i < alloc); i++)
	out[i] = acum1[i];
    } else {
      for(i=1;(acum3[i]!='\0' && i < alloc); i++)
	out[i] = acum3[i];
    }
  } else {
    if (encoder == 1){
      for(i=1;(acum2[i]!='\0' && i < alloc); i++)
	out[i] = acum2[i];
    } else {
      for(i=1;(acum4[i]!='\0' && i < alloc); i++)
	out[i] = acum4[i];
    }
    
  }
  out[i]='\0';
  return i;
}


int cptable(double * out, int alloc, double * in){
  
  int i;
  
  for(i=0; i < alloc; i++)
    out[i] = in[i];
  return i;
}


int stats (RESULT * out, bool to_print, double * vect, bool wweights, bool capped, int alloc, int id, bool clean, int encoder, bool no_table){

  
  double data[alloc];
  int len;
  
  if (vect == NULL)
    len = cpacum(data, alloc, id, encoder);
  else
    len = cptable(data, alloc, vect);
  
  //len --;
  if(to_print && !no_table)
    prac(len, data);
  
  
  int i, k, ret=1;
  double avrg = avg(len, data);
  
  if(capped){
    for (k=0, i = len-10; (i < len && k < 10); i++, k++)
      data[i] = data[k] = avrg;
  }
  
  if(clean){
    for (i = 0;i < len; i++){
      if(data[i] <= 650)
	data[i] = avrg;
    }
  }
  
  double mean, min, max, wmean, wvariance, wsd, wabs, swmean, swvariance, swsd, swabs, median, upperq, lowerq, /*middleq*/ uq, lq, rmin, rmax;
  min = max = wmean = wvariance = wsd = wabs = rmin = rmax = swmean = swvariance = swsd = swabs = median = upperq = lowerq = /*middleq*/  uq = lq = 0;
  
  mean = out->res[0] = gsl_stats_mean(data, 1, len);
  double variance = out->res[1] = gsl_stats_variance_m(data, 1, len, mean);
  double sd = out->res[2] = gsl_stats_sd_m(data, 1, len, mean);
  
  double abs = out->res[3] = gsl_stats_absdev_m (data, 1, len, mean);
  double autocorr = out->res[4] = gsl_stats_lag1_autocorrelation_m (data, 1, len, mean);
  
  if(wweights){
    double weights[len];
    double sweights[len];
    
    ret=2;
    cal_weight(len, weights, 1 ,3);
    
    wmean = out->res[5] = gsl_stats_wmean (weights, 1, data, 1, len);
    wvariance = out->res[6] = gsl_stats_wvariance_m (weights, 1, data, 1, len, wmean);
    wsd = out->res[7] = gsl_stats_wsd_m (weights, 1,data, 1, len, wmean);
    wabs = out->res[8] = gsl_stats_wabsdev_m (weights,1,data,1,len, wmean);
    gsl_stats_minmax(&min, &max, data, 1, len);
    
    gsl_sort (data,1,len);
    smart_weights(len, sweights, 0.1, max, data,id,out->per);
    
    get_limits(out->per, &uq, &lq, 0.70, max, &rmin, &rmax);
    
    
    if(to_print)
      printf("uq: %.3f, lq: %.3f\n", uq, lq);

    median = gsl_stats_median_from_sorted_data (data,1,len);
    upperq = gsl_stats_quantile_from_sorted_data (data,1,len,0.8);//uq
    lowerq = gsl_stats_quantile_from_sorted_data (data,1,len,0.2);//lq
    
    // Smart weights:
    swmean = out->res[9] = gsl_stats_wmean (sweights, 1, data, 1, len);
    swvariance = out->res[10] = gsl_stats_wvariance_m (sweights, 1, data, 1, len, wmean);
    swsd = out->res[11] = gsl_stats_wsd_m (sweights, 1,data, 1, len, wmean);
    swabs = out->res[12] = gsl_stats_wabsdev_m (sweights,1,data,1,len, wmean);
    
    out->res[13] = upperq;
    out->res[14] = lowerq;
    out->res[15] = rmax;
    out->res[16] = rmin;
  } else {
    for (k=5; k <= 16; k++)
      out->res[k] = 0;
    for (k=0; k < PER_SIZE; k++)
      out->per[k] = 0;
  }
  out->per[PER_SIZE-1] = '\0';
  out->res[STATS_SIZE-1] = '\0';
  
  if(to_print){
    printf("ticks received: %d\nmean: %.5f\nvariance: %.5f\nstandard deviation: %.5f\nmedian: %.5f\nquantil superior: %.5f\nquantil inferior: %.5f\nrmax: %.5f\nrmin: %.5f\n", len, out->res[0], variance, sd, median, upperq, lowerq, rmax, rmin);
    printf("autocorrelation: %.5f\nmin: %.5f\nmax: %.5f\nabsolute_deviation: %.5f\n\nweights: \nwmean: %.5f\nwvariance: %.5f\nwstandard_deviation: %.5f\nwabs_dev: %.5f\n\n", autocorr,min,max,abs, wmean, wvariance, wsd, wabs);
    printf("smart weights: \nswmean: %.5f\nswvariance: %.5f\nswstandard_deviation: %.5f\nswabs_dev: %.5f\n\n",swmean, swvariance, swsd, swabs);
   
  }
  return ret;
  
}

void get_limits(double per[], double * uq, double * lq, double min_pop, double max, double * rmin, double * rmax){
  
  bool found = false;
  int i,k;
  for (i=0; i<10 && !found; i++){
    for(k=i+1; k<10 && !found; k++){
      if(((per[i] + per[k]) >= min_pop))
	found = true;
    }
  }
  k--;i--;

  double uqbase = 0.5 + (min_pop/2);
  double lqbase = 0.5 - (min_pop/2);
  
  if (found){
    double total = per[i] + per[k];
    if (total > min_pop){
      double to_div = total - min_pop;
      *uq = uqbase +(to_div/2);
      *lq = lqbase -(to_div/2);

    } else {
      *uq = uqbase;
      *lq = lqbase;
    }
    *rmin = ((double)i/10) * max;
    *rmax = ((double)(k+1)/10) * max;

  } else {
    *rmin = 0;
    *rmax = 0;
    *uq = uqbase;
    *lq = lqbase;
  }
  
}

void prac (int len, double * vec){
  
  int i=0;
  double val;
  for (; i < len; i++){
    val = vec[i];
    if( ( (i != 0) && ((i%PRAC_TRUNC) == 0) ) )
      printf("%5d\n", (int)val);
    else
      printf("%5d, ", (int)val);
  }
  printf("\n\n");
  
}

void prwcr (int len, double per[]){
  
  int i;
  int aux;
  for (i=0; i < len; i++){
    aux = per[i];
    printf("%d, ", aux);
  }
  printf("\n");
  
}


void prw (int len, double per[]){
  
  int i;
  double aux;
  for (i=0; i < len; i++){
    aux = per[i];
    printf("%.35f, ", aux);
  }
  printf("\n");
  
}


double avg (int len, double * data){
  
  long long sum = 0;
  int i;
  for (i = 10; i <len-10; i++){
    //		printf("volta: %d, sum: %lld\n", i, sum);
    sum+=data[i];
  }
  
  return ((sum/(len-20)));
}


void cal_weight(int len, double * weights, double less, double max){

  int i;
  int head = 10;
  int tail = len - head;
  int first = len/10;
  int last = len - first;
 
  for (i=0; i<len; i++){
    if ((i <= head) || (i >= tail))
      weights[i] = 0;
    else if ((i <= first) || (i >= last) )
      weights[i] = (double)less;
    else
      weights[i] = (double)max;
  }
  
}

void smart_weights(int len, double * weights, double step, double max, double *data, int id, double per []){
  
  /* La idea: contar la poblacion de cada quantil (menos la del quantil anterior), y, sacar el porcentage respeto a la poblacion total (todos los tics), a partir de ahi asignar los      pesos de cada elemento dependiendo del percentage de poblacion del quantil al que corresponda */
  /* Sorted data */
  
  double quantil;
  int res;
  int i;

  for (i=0, quantil=step;quantil<=1;quantil+=step, i++){
    res = no_quantil(len,data,quantil,max,step);
    per[i] = (double)((double)res/len);
  }
  per[i]='\0';
  
  for (i=0; i<len; i++){
    weights[i] = per[(int)(((data[i]/max)-step)*10)];
  }
  
}

int no_quantil (int len, double * data, double perc, double max, double step){

  /* NO-quantil, es decir lo coontrario, dado un porcentage del valor del máximo, cuanta población esta por debajo de ese valor, (Distribuciones?) */
  /* sorted data */
  
  int i;
  double ref = perc * max;
  
  int last = 0;
  double val = data[0];
  double ant = 0;
  double ult = (perc - step) * max;
  
  for (i=1; val < ref; i++) {
    if (val < ant)
      printf("\n\t\t>>WEIRD DATA<<\t\t\n\n");
    if (ult >= val)
      last = i;
    ant = val;
    val = data[i];
  }
  val=data[i];
  last++;
  
  return (i - last);
  
}


