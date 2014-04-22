//#include <unistd.h>
//#include <stdio.h>
//#include <gsl/gsl_math.h>
//#include <gsl/gsl_statistics.h>

#include <lego.h>
#include <gsl/gsl_sort.h>

#define STATS_SIZE	18
#define PER_SIZE	11
#define INT1_KEY	1
#define INT2_KEY	2
//#define BASE		370
#define MAXM		20
#define ENC_RES		4
#define PRAC_TRUNC      25
//MOTOR motor1, motor2;
//int an_fd;

//ENC enc11, enc21, enc12, enc22;

TSPEC t11, t12, t21, t22;

MOTOR * m1, * m2;

double *acum4 = NULL, *acum3 = NULL, *acum2 = NULL, *acum1 = NULL;

/*ENC * e11 = m1->enc1;
ENC * e12 = m1->enc2;
ENC * e21 = m2->enc1;
ENC * e22 = m2->enc2;
*/

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

int get_in(char *, int);
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
void close_acums(int);
//void reinit_acums(int, MOTOR *);
void tr_enc(double * [], int);
void prfive(RFIVE *, int);
double difft (TSPEC *, TSPEC *);

extern void isr_print_1(void){
  if(m1->moving){
    m1->enc1->tics++;
    if ((m1->enc1->tics%100) == 0)
      printf("Hi, I'm the ISR 1\n");
  }
}
extern void isr_print_2(void){
  if(m2->moving){
    m2->enc1->tics++;
    if ((m2->enc1->tics%100) == 0)
      printf("Hi, I'm the ISR 2\n");
  }
}

extern void dbg_isr_11(void){
  //double auxt;
  if(m1->moving){
    clock_gettime(CLK_ID, &t11);
    //auxt = difft(&m1->enc1->tmp, &t11);
    //if(auxt > USXT_MIN) {
    acum1[m1->enc1->tics] = difft(&m1->enc1->tmp, &t11);//e11->tics == 0 ? (double)((e11->tmp.tv_sec * 1000000) + (e11->tmp.tv_nsec/1000)) : (e11->delay); /* NANOS TO MICR0S */
    pthread_mutex_lock(&m1->enc1->mtx);
    m1->enc1->tics++;
    pthread_mutex_unlock(&m1->enc1->mtx);
    clock_gettime(CLK_ID, &m1->enc1->tmp);
    //}

  }
}

extern void dbg_isr_12(void){
  //double auxt;
  if(m1->moving){
    clock_gettime(CLK_ID, &t12);
    // auxt = difft(&m1->enc2->tmp, &t11);
    //if(auxt > USXT_MIN) {
    acum3[m1->enc2->tics] = difft(&m1->enc2->tmp, &t12);//e12->tics == 0 ? (double)((e12->tmp.tv_sec * 1000000) + (e12->tmp.tv_nsec/1000)) : (e12->delay); /* NANOS TO MICR0S */
    pthread_mutex_lock(&m1->enc2->mtx);
    m1->enc2->tics++;
    pthread_mutex_unlock(&m1->enc2->mtx);
    clock_gettime(CLK_ID, &m1->enc2->tmp);
    //}
  }
}

extern void dbg_isr_21(void){
  if(m2->moving){
    clock_gettime(CLK_ID, &t21);
    acum2[m2->enc1->tics] = difft(&m2->enc1->tmp, &t21);//e21->tics == 0 ? (double)((e21->tmp.tv_sec * 1000000) + (e21->tmp.tv_nsec/1000)) : (e21->delay); /* NANOS TO MICR0S */
    pthread_mutex_lock(&m2->enc1->mtx);
    m2->enc1->tics++;
    pthread_mutex_unlock(&m2->enc1->mtx);
    clock_gettime(CLK_ID, &m2->enc1->tmp);
  }
}

extern void dbg_isr_22(void){
  if(m2->moving){
    clock_gettime(CLK_ID, &t22);
    acum4[m2->enc2->tics] = difft(&m2->enc2->tmp, &t22);//e22->tics == 0 ? (double)((e22->tmp.tv_sec * 1000000) + (e22->tmp.tv_nsec/1000)) : (e22->delay); /* NANOS TO MICR0S */
    pthread_mutex_lock(&m2->enc2->mtx);
    m2->enc2->tics++;
    pthread_mutex_unlock(&m2->enc2->mtx);
    clock_gettime(CLK_ID, &m2->enc2->tmp);
  }
}
                                                                                                                                                                                    

double pcoef1[MAX_COEF] = { 5769.92181090883605065755546092987060547, 4479.09869805242851725779473781585693359, 3815.51846175456466880859807133674621582, 3281.67669122572169726481661200523376465,2960.91753850553914162446744740009307861, 2677.93018281985223438823595643043518066, 2443.18133088759122983901761472225189209, 2256.43226464750478044152259826660156250, 2076.36443362455156602663919329643249512, 1880.84519361533693881938233971595764160, 1766.91845025184238693327642977237701416, 1627.52019487895177007885649800300598145, 1510.59062333105975994840264320373535156, 1405.75873144575962214730679988861083984, 1300.50733998499845256446860730648040771, 1199.64103984038956696167588233947753906, 1126.43123210844078130321577191352844238,1077.82681899074850662145763635635375977, 1047.62979268326716919546015560626983643, 1024.72136637642188361496664583683013916, 0.0 };

double dcoef1[MAX_COEF] = { 1280.81678440747532476962078362703323364, 1080.55413534975264155946206301450729370, 962.37714162269867301802150905132293701, 794.70516615830206319515127688646316528, 681.73197952620921569177880883216857910, 617.17903483213251547567779198288917542, 570.02783788164811085152905434370040894, 514.28307792829866684769513085484504700, 485.39651961788968037581071257591247559, 434.73204754488267553824698552489280701, 411.49080190769848286436172202229499817, 396.74984722622548360959626734256744385, 376.29638030608435883550555445253849030, 365.62140796971101508461288176476955414, 343.04282202860895267804153263568878174, 323.94336764183344712364487349987030029, 314.57135151003888040577294304966926575, 287.30168694078963653737446293234825134, 278.15403125283489771391032263636589050, 228.85462293561045044043567031621932983, 0.0 };

double pcoef2[MAX_COEF] = { 5784.08437837837846018373966217041015625, 4445.20846790107862034346908330917358398, 3747.11780970736981544177979230880737305, 3279.89248824115747993346303701400756836,2910.45785170638373529072850942611694336, 2633.61340680736202557454816997051239014, 2381.54762904100789455696940422058105469, 2163.21015912026996375061571598052978516, 1999.84504192275835521286353468894958496, 1844.27244297564470798533875495195388794, 1715.28248146105988780618645250797271729, 1597.04232922647133818827569484710693359, 1481.29561624296366062480956315994262695, 1377.86708383419227175181731581687927246, 1301.24395926704619341762736439704895020, 1248.96952471514123317319899797439575195, 1149.37726195246636962110642343759536743,1117.65528375465146382339298725128173828, 1097.79113254987851178157143294811248779, 1071.86196162545297738688532263040542603, 0.0 };

double dcoef2[MAX_COEF] = { 1129.16526209913718048483133316040039062, 883.12832455475847837078617885708808899, 819.31381796521941396349575370550155640, 742.33985454770515843847533687949180603, 685.03265188058412604732438921928405762, 666.17199380024021593271754682064056396, 627.18803536872428594506345689296722412, 599.68030858976464969600783661007881165, 585.16856491592193378892261534929275513, 571.32514217480252227687742561101913452, 553.57073691453115316107869148254394531, 548.80894970490135165164247155189514160, 537.02872878985044735600240528583526611, 525.58145446398498279449995607137680054, 506.85942229072389864086289890110492706, 487.50514143674456590815680101513862610, 439.71799989441075240392819978296756744, 425.31908384162727543298387899994850159, 411.85678335247609993530204519629478455, 409.68090676785232062684372067451477051, 0.0 };


int main (int argc, char * argv[]) {

  int tst = argc < 2 ? 1 : atoi(argv[1]);
  
  ENC en11, en12, en21, en22;
  //PID p1, p2;
  
  /*e11.pin = M1_ENC1;
    e11.isr = &dbg_isr_11;
    e12.pin = ENULL;
    e12.isr = NULL;
    e21.pin = M2_ENC1;
    e21.isr = &dbg_isr_21;
    e22.pin = ENULL;
    e22.isr = NULL;
    
    
    e11.pin = ENULL;
    e11.isr = NULL;
    e12.pin = M1_ENC2;
    e12.isr = &dbg_isr_12;
    e21.pin = ENULL;
    e21.isr = NULL;
    e22.pin = M2_ENC2;
    e22.isr = &dbg_isr_22;
  */

  /*p1.svel = MAX_COEF;
    cptable(p1.cp, MAX_COEF , pcoef1);
    cptable(p1.cd, MAX_COEF , dcoef1);
    
    p2.svel = MAX_COEF;
    cptable(p2.cp, MAX_COEF , pcoef2);
    cptable(p2.cd, MAX_COEF , dcoef2);
  */
  
  //MOTOR mot1, mot2;

  //MOTOR motor1, motor2;

  //MOTOR * mot1;
  //MOTOR * mot2;
  
  en11.pin = M1_ENC1;
  en11.isr = &dbg_isr_11;
  en12.pin = M1_ENC2;
  en12.isr = &dbg_isr_12;
  en21.pin = M2_ENC1;
  en21.isr = &dbg_isr_21;
  en22.pin = M2_ENC2;
  en22.isr = &dbg_isr_22;

  lego_init();

  int ret = mt_new(m1, &en11, &en12, 1) ? OK : FAIL;
  ret = ret ? mt_new(m2, &en21, &en22, 2) ? OK : FAIL : FAIL;

  ret = ret ? mt_pid_conf(m1, pcoef1, dcoef1) ? OK : FAIL : FAIL;
  ret = ret ? mt_pid_conf(m2, pcoef2, dcoef2) ? OK : FAIL : FAIL;

  //m1=mot1;
  //m2=mot2;
//get_in("Ready??", 0);

struct timespec time;


time.tv_sec = 5;
time.tv_nsec = 0;


 if (ret){
   int ticks, thread_rtn;
   //set_verbose(1);
   switch (tst){
   case 1: //  Test per init_motors() i per les funcions de moviment basiques:
     {
       int vel   = argc < 3 ? 60 : atoi(argv[2]);
       int turns = argc < 4 ? 7 : atoi(argv[3]);
       double gains = argc < 5 ? 0 : atof(argv[4]);
       bool calib = argc < 6 ? false : atoi(argv[5]) != 0 ? true : false;
       
       //set_verbose(2);
       mt_reconf(m1, NULL, NULL); //back to defaults
       mt_reconf(m2, NULL, NULL); //back to defaults
       mt_pid_set_gains(m1->pid, gains ,gains ,gains);
       mt_pid_set_gains(m2->pid, gains ,gains ,gains);
       
       printf("kp: %f, kd: %f, ki: %f, calib: %d\n", gains, gains , gains, calib);
       int mostres = MAX_COEF-1;
       if(calib){
	 mt_calibrate(mostres, 0.8);
	 printf("\n");
	 printf("coefs_punts_1: ");prw(mostres, m1->pid->cp);
	 printf("coefs_punts_2: ");prw(mostres, m2->pid->cp);
	 printf("coef_dev_1:    ");prw(mostres, m1->pid->cd);
	 printf("coef_dev_2:    ");prw(mostres, m2->pid->cd);
	 printf("\n");
       }else
	 printf("\n");
       
       printf("MOTOR 1: %d,%d,%d,%d\n", (int)m1->pinf, (int)m1->pinr, mt_enc_is_null(m1->enc1) ? ENULL : (int)m1->enc1->pin, mt_enc_is_null(m1->enc2) ? ENULL : (int)m1->enc2->pin);
       printf("MOTOR 2: %d,%d,%d,%d\n", (int)m2->pinf, (int)m2->pinr, mt_enc_is_null(m2->enc1) ? ENULL : (int)m2->enc1->pin, mt_enc_is_null(m2->enc2) ? ENULL : (int)m2->enc2->pin);

       if(!mt_move(m1,"fwd", vel))
	 printf("mot_fwd FAIL!");
       
       nanosleep(&time, NULL);
       ticks = mt_stop(m1, true);
       mt_wait_for_stop(m1, 0.8);
       printf("tics received: %d\n", ticks);
       //reset_acums(turns);
       /*	if (pid)
		join_threads();*/
       if(!mt_move(m2, "b", vel))
	 printf("mot_rev FAIL!");
       
       nanosleep(&time, NULL);
       ticks = mt_stop(m2, true);
       mt_wait_for_stop(m2, 0.8);
       printf("tics received: %d\n", ticks);
       //reset_acums(turns);
       /*	if (pid)
		join_threads();*/

       mt_move_t(m1, mt_tticks(m1, turns), "fwd", vel, 0); mt_wait(m1); //moure  un motor en direccio "fwd", al vel% de la velocitat maxima, fins a fer "turns" voltes
       ticks = mt_get_ticks(m1);
       printf("tics received: %d\n", ticks);
     }
     break;;
   case 2: //thread-test: recuperem el valor de pthread create, i el thread deixa els ticks que move_till_ticks_b() recull de mot_stop() al camp tics de la estructura MOTOR (que previament mot_stop()
     {	// ha resetejat, TO_DO: mirar si surt mes a compte un condicional a mot_stop per a que no reseteji el camp)
       
       
       MOTOR * m = argc < 3 ? m1 : atoi(argv[2]) == 1 ? m1 : m2;
       int vel = argc < 4 ? 60 : atoi(argv[3]);
       int turns = argc < 5 ? 7 : atoi(argv[4]);
       double gains = argc < 6 ? 0 : atof(argv[5]);
       double pctr = argc < 7 ? 0 : atof(argv[6]);
       
       printf("pid status :  %d\n", m->pid->svel);
       
       mt_reconf(m, NULL, NULL); //back to defaults
       mt_pid_set_gains(m->pid, gains ,gains ,gains);
       /*m->pid->kp = gains;
	 m->pid->ki = gains;
	 m->pid->kd = gains;*/
       printf("MOTOR: %d,%d,%d,%d\n", (int)m->pinf, (int)m->pinr, mt_enc_is_null(m->enc1) ? ENULL : (int)m->enc1->pin, mt_enc_is_null(m->enc2) ? ENULL : (int)m->enc2->pin);
       
       init_acums(turns, m);
       thread_rtn = mt_move_t(m, mt_tticks(m, turns), "f", vel, pctr);
       while (m->moving == true){
	 printf("moving\n");
	 sleep(1);
       }
       printf("pthread_create says: %d, ticks received: %d, ticks expected: %d\n", thread_rtn, mt_get_ticks(m), mt_tticks(m, turns));
     }
     break;;
   case 3: // interrupcions, aixo de moment estan un pel xungo, pero per comprovar si els delays entre interrupcions son estables a diferents velocitats:
     {
       RESULT out;
       //RESULT out2;
       time.tv_sec = 0;
       time.tv_nsec = 100000000; //70 centesimes de segon ??
       MOTOR * m  = argc < 3 ? m1 : atoi(argv[2]) == 1 ? m1 : atoi(argv[2]) == 2 ? m2 : m1;
       int vel = (argc < 4) ? get_in("Velocity?:",1) : atoi(argv[3]);
       int turns = argc < 5 ? 7 : atoi(argv[4]);
       init_acums(turns, m);
       reset_acums(turns, m);
       mt_wait_for_stop(m,2);
       mt_reset_enc(m);
       thread_rtn = mt_move_t(m, mt_tticks(m, turns), "f", vel, 0);mt_wait(m);
       
       //(RESULT * out, bool to_print, double * vect, bool wweights, bool capped, int alloc, int id, bool clean, int encoder, bool no_table)
       mt_wait_for_stop(m,1);
       int tot = mt_get_ticks(m);
       //close_acums(tot);
       if(mt_enc_count(m) == 1){ //REVISAR CUANDO 0 ENC OK!
	 int encid = mt_enc_is_null(m->enc1) ? 2 : 1;
	 stats(&out,true, NULL, true, true, tot, m->id, true, encid, false);
       } else {
	 stats(&out,true, NULL, true, true, tot, m->id, true, 1, false);
	 printf("\n");
	 stats(&out,true, NULL, true, true, tot, m->id, true, 2, false);
       }
       
     }
     break;;
   case 4:
     {
       int i, velo, index, alloc;
       MOTOR * motor  = argc < 3 ? m1 : atoi(argv[2]) == 1 ? m1 : atoi(argv[2]) == 2 ? m2 : m1;
       int mostres = argc < 4 ? 10 : atoi(argv[3]);
       int step = argc < 5 ? 10 : atoi(argv[4]);
       RESULT rese1[(int)((MAX_VEL/step) * mostres)];// = (RESULT *) malloc(((MAX_VEL/step) * mostres) * sizeof(RESULT));
       RESULT rese2[(int)((MAX_VEL/step) * mostres)];// = (RESULT *) malloc(((MAX_VEL/step) * mostres) * sizeof(RESULT));
       int turns = argc < 6 ? 5 : atoi(argv[5]);

       init_acums(turns, motor);
       reset_acums(turns, motor);
       for (velo = step; velo <= MAX_VEL; velo+=step){
	 for (i = 0; i < mostres; i++){
	   index = ((((velo/step)-1)*mostres) + i);
	   if ((index%50 == 0) && (index != 0))
	     printf("mostres salvades: %d\n", index);
	   mt_wait_for_stop(motor,2);
	   mt_reset_enc(motor);
	   alloc = mt_move_t(motor, mt_tticks(motor, turns), "f", velo, 0);mt_wait(motor);
	   //close_acums(alloc);
	   if(mt_enc_count(motor) == 1){
	     int encid = mt_enc_is_null(motor->enc1) ? 2 : 1;
	     if(encid == 1){
	       stats(&rese1[index], false, NULL, true, true, alloc, motor->id, true, encid, true);
	     }else{
	       stats(&rese2[index], false, NULL, true, true, alloc, motor->id, true, encid, true);
	     }
	   } else {
	     stats(&rese1[index], false, NULL, true, true, alloc, motor->id, true,1, true);
	     stats(&rese2[index], false, NULL, true, true, alloc, motor->id, true,2, true);
	   }
	   reset_acums(turns, motor);
	 }
       }
       printf ("\nMOTOR %d, STEP: %d, MOSTRES %d x step:\n\n\t\t\tENC 1:\n", motor->id, step, mostres);
       if(!mt_enc_is_null(motor->enc1))
	 comp_res(rese1, mostres, step, motor->id);
       printf ("\n\n\t\t\tENC: 2\n");
       if(!mt_enc_is_null(motor->enc2))
	 comp_res(rese2, mostres, step, motor->id);
     }
     break;;
   case 5:
     {//test vel
       TSPEC tini, taux;
       long enano;
       double txturn, tbticks, elapsed, txsec;
       long long int elmicras;
       int esec, index, ticks;
       MOTOR * m  = argc < 3 ? m1 : atoi(argv[2]) == 1 ? m1 : atoi(argv[2]) == 2 ? m2 : m1;
       int vel = (argc < 4) ? get_in("Velocity?:",1) : atoi(argv[3]);
       int step = 5;
       bool to_pr = argc < 5 ? false : atoi(argv[4]) == 1 ? true : false;
       int turns;
       RESULT res[MAXM/step], res2[MAXM/step];
       RFIVE data [MAXM/step];
       
       mt_pid_set_null(m->pid);
       init_acums((int)MAXM, m);
       
       for (turns = step; turns <= MAXM; turns += step){
	 
	 index = ((turns/step)-1);
	 mt_wait_for_stop(m, 0.3);
	 reset_acums((int)MAXM, m);
	 mt_reset_enc(m);
	 clock_gettime(CLK_ID, &tini);
	 mt_move_t(m, mt_tticks(m, turns), "f", vel,0);mt_wait(m);
	 ticks = mt_get_ticks(m);
	 clock_gettime(CLK_ID, &taux);
	 enano = (taux.tv_nsec - tini.tv_nsec);
	 esec = (int)(taux.tv_sec - tini.tv_sec);
	 elapsed = esec+(enano*0.000000001);
	 elmicras = (long long int)((enano/1000) + (long long int)(1000000 * esec));
	 txsec = (ticks / elapsed);
	 txturn = (elmicras / turns);
	 tbticks = (txturn/m->ticsxturn);
	 if(to_pr)
	   printf("voltes: %d, telapsed: %.5lf seg\ntxvolta: %.5lf seg\ntbticks_total: %.5lf micras\nticks/sec: %.5lf\n", turns, elapsed , txturn/1000000, tbticks, txsec);
	 data[index].tturn = txturn;
	 data[index].txsec = txsec;
	 if(mt_enc_count(m) == 1){
	   int encid = mt_enc_is_null(m->enc1) ? 2 : 1;
	   if(encid == 1){
	     stats(&res[index], false, NULL, true, true, ticks, m->id, true, encid, true);
	     if(to_pr)
	       printf("\nenc1,\nmean: %f\ntbticks_e1: %.5f\nrange_min: %f\nrange_max: %f\n", res[index].res[0],(double)(elmicras/m->enc1->tics), res[index].res[14], res[index].res[13]);
	     data[index].e1[0] = res[index].res[0];
	     data[index].e1[1] = (elmicras/m->enc1->tics);
	     data[index].e1[2] = res[index].res[14];
	     data[index].e1[3] = res[index].res[13];
	   }else{
	     stats(&res2[index], false, NULL, true, true, ticks, m->id, true, encid, true);
	     if(to_pr)
	       printf("\nenc2,\nmean: %f\ntbticks_e2: %.5f\nrange_min: %f\nrange_max: %f\n", res2[index].res[0],(double)(elmicras/m->enc2->tics), res2[index].res[14], res2[index].res[13]);
	     data[index].e2[0] = res2[index].res[0];
	     data[index].e2[1] = (elmicras/m->enc2->tics);
	     data[index].e2[2] = res2[index].res[14];
	     data[index].e2[3] = res2[index].res[13];
	     
	   }
	 } else {
	   stats(&res[index], false, NULL, true, true, ((ticks/2)+5), m->id, true,1, true); //AVANS: stats(&res[index], false, NULL, true, true, ticks, m->id, true,1, true);
	   if(to_pr)
	     printf("\nenc1,\nmean: %f\ntbticks_e1: %.5f\nrange_min: %f\nrange_max: %f\n", res[index].res[0], (double)(elmicras/m->enc1->tics), res[index].res[14], res[index].res[13]);
	   data[index].e1[0] = res[index].res[0];
	   data[index].e1[1] = (elmicras/m->enc1->tics);
	   data[index].e1[2] = res[index].res[14];
	   data[index].e1[3] = res[index].res[13];
	   
	   stats(&res2[index], false, NULL, true, true, ((ticks/2)+5), m->id, true,2, true);
	   if(to_pr)
	     printf("\nenc2,\nmean: %f\ntbticks_e2: %.5f\nrange_min: %f\nrange_max: %f\n", res2[index].res[0],(double)(elmicras/m->enc2->tics), res2[index].res[14], res2[index].res[13]);
	   data[index].e2[0] = res2[index].res[0];
	   data[index].e2[1] = (elmicras/m->enc2->tics);
	   data[index].e2[2] = res2[index].res[14];
	   data[index].e2[3] = res2[index].res[13];
	   
	 }
	 if(to_pr)
	   printf("\n------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
	 //free_acums(m);
	 //printf("faig free\n");
       }
       printf("MOTOR: %d, step: %d, vel: %d\n\n", m->id, step, vel);
       prfive(data, MAXM/step);
       free_acums(m);
       
     }
     break;;
   case 6: /* test motor reconf */
     {
       MOTOR * m  = argc < 3 ? m1 : atoi(argv[2]) == 1 ? m1 : atoi(argv[2]) == 2 ? m2 : m1;
       ENC * e1 = m->enc1;
       ENC * e2 = m->enc2;
       int pin1 = m == m1 ? M1_ENC1 : M2_ENC1;
       int pin2 = m == m1 ? M1_ENC2 : M2_ENC2;
       int e2pin, e1pin, vel = 60, turns = 7, ticks;
       ENC e2aux, e1aux;
       
       init_acums(5000, m);
       reset_acums(5000, m);
       printf("tot activat, m_e1: %d, m_e2: %d,\n", e1->pin, e2->pin);
       ticks = mt_move_t(m, mt_tticks(m, turns), "f", vel, 0); mt_wait(m);
       printf("tics totals: %d, esperats: %d, tics e1: %d, tics e2: %d\n", ticks, mt_tticks(m, turns), e1->tics, e2->tics);
       printf("desactivant encoder 2 ... \n");
       mt_wait_for_stop(m,2);
       e2->pin = ENULL;
       mt_reconf(m, NULL, e2); //e1 untouched.
       e2pin = mt_enc_is_null(e2) ? ENULL : e2->pin;
       printf("m_e2 desactivat , m_e1: %d, m_e2: %d,\n", e1->pin, e2pin);
       //get_in("Mira com esta el pin 22 cap d'escombra!", 1);
       
       mt_wait_for_stop(m, 3);
       mt_reset_enc(m);
       ticks = mt_move_t(m, mt_tticks(m, turns), "f", vel, 0); mt_wait(m);
       printf("tics totals: %d, esperats: %d, tics e1: %d, tics e2: %d\n", ticks, mt_tticks(m, turns), e1->tics, mt_enc_is_null(e2) ? 0 : e2->tics);
       printf("reactivant encoder 2 ...\n");
       e2aux.pin = pin2;
       e2aux.isr = m == m1 ? &dbg_isr_12 : &dbg_isr_22;
       mt_reconf(m, NULL, &e2aux); // e1 untouched
       printf("tot activat, m_e1: %d, m_e2: %d,\n", e1->pin, e2->pin);
       //get_in("Mira com esta el pin 22 cap d'escombra!", 1);
       mt_wait_for_stop(m, 3);
       
       mt_reset_enc(m);
       ticks = mt_move_t(m, mt_tticks(m, turns), "f", vel, 0);mt_wait(m);
       printf("tics totals: %d, esperats: %d, tics e1: %d, tics e2: %d\n", ticks, mt_tticks(m, turns), e1->tics, mt_enc_is_null(e2) ? 0 : e2->tics);
       printf("desactivant encoder 1 ... \n");
       e1->pin = ENULL;
       mt_reconf(m, e1, NULL); //e2 untouched.
       e1pin = mt_enc_is_null(e1) ? ENULL : e1->pin;
       printf("m_e1 desactivat , m_e1: %d, m_e2: %d,\n", e1pin, e2->pin);
       mt_wait_for_stop(m, 3);
       //get_in("Mira com esta el pin 27 cap d'escombra!", 1);
       
       mt_reset_enc(m);
       ticks = mt_move_t(m, mt_tticks(m, turns), "f", vel, 0);mt_wait(m);
       printf("tics totals: %d, esperats: %d, tics e1: %d, tics e2: %d\n", ticks, mt_tticks(m, turns), mt_enc_is_null(e1) ? 0 :e1->tics, e2->tics);
       printf("reactivant encoder 1 ...\n");
       e1aux.pin = pin1;
       //e1aux.isr = m == m1 ? &dbg_isr_11 : &dbg_isr_21;
       e1aux.isr = m->id == 1 ? &isr_print_1 : &isr_print_2;
       //m->ticsxturn = 360;
       mt_reconf(m, &e1aux, NULL); // e2 untouched
       printf("tot activat, m_e1: %d, m_e2: %d,\n", e1->pin, e2->pin);
       //get_in("Mira com esta el pin 27 cap d'escombra!", 1);
       mt_wait_for_stop(m, 3);
       
       mt_reset_enc(m);
       ticks = mt_move_t(m, mt_tticks(m, turns), "f", vel, 0);mt_wait(m);
       printf("tics totals: %d, esperats: %d, tics e1: %d, tics e2: %d\n", ticks, mt_tticks(m, turns), e1->tics, e2->tics);
       printf("tornant a defaults ...\n");
       mt_reconf(m, NULL, NULL);
       printf("tot activat, m_e1: %d, m_e2: %d,\n", e1->pin, e2->pin);
       mt_wait_for_stop(m, 3);

       mt_reset_enc(m);
       ticks = mt_move_t(m, mt_tticks(m, turns), "f", vel, 0);mt_wait(m);
       printf("tics totals: %d, esperats: %d, tics e1: %d, tics e2: %d\n", ticks, mt_tticks(m, turns), mt_enc_is_null(e1) ? 0 : e1->tics, e2->tics);
       printf("intentant desactivar els 2 a l'hora ...\n");
       e1->pin = ENULL;
       e2->pin = ENULL;
       mt_reconf(m, e1, e2);
       /* mot_reconf(m,NULL,NULL) es tambien v?lido, pero lo que hace es un reload de las structs globales, que son con las que hacemos pruebas asi pues nada... */
     }
     break;;
   case 7://test calibrate
     {
       MOTOR * m  = argc < 3 ? m1 : atoi(argv[2]) == 1 ? m1 : atoi(argv[2]) == 2 ? m2 : NULL;
       int mostres = argc < 4 ? 10 : atoi(argv[3]), i;
       bool clavat =  argc < 5 ? false : atoi(argv[4]) != 0 ? true : false;
       double twait = argc < 6 ? 1.7 : atof(argv[5]);
       int micras = 0, desv = 0, micras2 = 0, desv2 = 0;
       int step = (MAX_VEL / mostres);
       
       if (m != NULL) { 
	 printf("calibrating MOTOR: %d", m->id);
	 mt_calibrate(mostres, twait);
	 printf("\n");
	 printf("coefs_punts: ");prw(mostres, m->pid->cp);
	 printf("coef_dev:    ");prw(mostres, m->pid->cd);
	 printf("\n");
	 for ( i = clavat ? MIN_VEL : MIN_VEL+(step/2); i <= MAX_VEL; i += step ){
	   if ((i + step > MAX_VEL) && clavat)
	     i = MAX_VEL;
	   else if ((i == MAX_VEL) && !clavat)
	     i = (MAX_VEL - (step/4));
	   
	   mt_get_params(m,i,&micras, &desv);
	   printf("params for %3d vel >> tbticks: %d, desv: %d\n",i,micras, desv);
	   //micras = desv = 0;
	 }
       } else {
	 printf("\ntrying to calibrate BOTH motors\n");
	 mt_calibrate(mostres, twait);
	 /*printf("\n");
	   printf("coefs_punts_1: ");prw(mostres, m1->pid->cp);
	   printf("coefs_punts_2: ");prw(mostres, m2->pid->cp);
	   printf("coef_dev_1:    ");prw(mostres, m1->pid->cd);
	   printf("coef_dev_2:    ");prw(mostres, m2->pid->cd);*/
	 printf("\n");
	 for ( i = clavat ? MIN_VEL : MIN_VEL+(step/2); i <= MAX_VEL; i += step ){
	   if ((i + step > MAX_VEL) && clavat)
	     i = MAX_VEL;
	   else if ((i == MAX_VEL) && !clavat)
	     i = (MAX_VEL - (step/4));
	   
	   mt_get_params(m1,i,&micras, &desv);
	   mt_get_params(m2,i,&micras2,&desv2);
	   printf("MOTOR1: params for %3d vel >> tbticks: %d, desv: %d\n",i,micras, desv);
	   printf("MOTOR2: params for %3d vel >> tbticks: %d, desv: %d\n",i,micras2, desv2);
	   //micras = desv = 0;
	 }
       }
     }
     break;;
   case 8: //get defaults
     {
       int mostres = argc < 3 ? 10 : atoi(argv[2]), i, k;
       double twait = argc < 4 ? 0.7 : atof(argv[3]);
       int iters = argc < 5 ? 5 : atoi(argv[4]);
       int micras1 = 0, desv1 = 0, micras2 = 0, desv2 = 0;
       int step = (MAX_VEL / mostres);
       double sum1[mostres], sum2 [mostres], defp1[mostres], defp2[mostres], defd1[mostres], defd2[mostres];
       /*				printf("MOT1_E1: %d, MOT1_E2: %d, MOT2_E1: %d, MOT2_E2: %d\n", m1->enc1->pin, m1->enc2->pin, m2->enc1->pin, m2->enc2->pin);
					printf("MOTOR 1: pcoef_avans d'entrar"); prw(10, m1->pid->cp);
					printf("MOTOR 1: dcoef_avans d'entrar"); prw(10, m1->pid->cd);
					printf("MOTOR 2: pcoef_avans d'entrar"); prw(10, m2->pid->cp);
					printf("MOTOR 2: dcoef_avans d'entrar"); prw(10, m2->pid->cd);
       */
       for(i = 0; i < iters; i++){
	 mt_calibrate(mostres, twait);
	 if(iters == 1){
	   printf("\n");
	   printf("coefs_punts_1: ");prw(mostres, m1->pid->cp);
	   printf("coefs_punts_2: ");prw(mostres, m2->pid->cp);
	   printf("coef_dev_1:    ");prw(mostres, m1->pid->cd);
	   printf("coef_dev_2:    ");prw(mostres, m2->pid->cd);
	   printf("\n");
	 } else
	   printf("\n");
	 
	 //vv = 95;
	 
	 for (k = 0; k<mostres; k++){
	   defp1[k] += m1->pid->cp[k];
	   defd1[k] += m1->pid->cd[k];
	   defp2[k] += m2->pid->cp[k];
	   defd2[k] += m2->pid->cd[k];
	   sum1[k] += (m1->pid->cp[k] + m2->pid->cp[k])/2;
	   sum2[k] += (m1->pid->cd[k] + m2->pid->cd[k])/2;
	 }
	 
	 for (k = MIN_VEL+(step/2); k < MAX_VEL; k += step){
	   mt_get_params(m1, k, &micras1, &desv1);
	   mt_get_params(m2, k, &micras2, &desv2);
	   printf("params for %2d%% power mot 1 >> tbticks: %d, desv: %d\n",k,micras1, desv1);
	   printf("params for %2d%% power mot 2 >> tbticks: %d, desv: %d\n",k,micras2, desv2);
	   // micras = desv = 0;
	 }
	 
       }
       for (i = 0; i < mostres; i++){
	 defp1[i] = defp1[i]/iters;
	 defd1[i] = defd1[i]/iters;
	 defp2[i] = defp2[i]/iters;
	 defd2[i] = defd2[i]/iters;
	 sum1[i] = sum1[i]/iters;
	 sum2[i] = sum2[i]/iters;
       }
       
       if (iters > 1){
	 printf("\n");
	 printf("DEFAULTS PUNTS_M1:");prw(mostres, defp1);
	 printf("DEFAULTS DESV_M1:");prw(mostres, defd1);
	 printf("DEFAULTS PUNTS_M2:");prw(mostres, defp2);
	 printf("DEFAULTS DESV_M2:");prw(mostres, defd2);
	 printf("DEFAULTS PUNTS:");prw(mostres, sum1);
	 printf("DEFAULTS DESV:");prw(mostres, sum2);
       }
     }
     break;;
   case 9:
     {
       MOTOR * m  = argc < 3 ? m1 : atoi(argv[2]) == 1 ? m1 : atoi(argv[2]) == 2 ? m2 : m1;
       int vel = argc < 4 ? 70 : atoi(argv[3]);
       double ttc = argc < 5 ? TTCDEF : atof(argv[4]);
       double kp = argc < 6 ? 1 : atof(argv[5]);
       double ki = argc < 7 ? 1 : atof(argv[6]);
       double kd = argc < 8 ? 1 : atof(argv[7]);
       bool calib = argc < 9 ? false : atoi(argv[8]) != 0 ? true : false;
       double pctr = argc < 10 ? 0.3 : atof(argv[9]);
       int turns = argc < 11 ? 10 : atoi(argv[10]);
       
       printf ("motor: %d, vel: %d, ttc: %f, kp: %f, ki: %f, kd: %f, calib: %s, turns: %d\n", m->id, vel, ttc, kp, ki, kd, calib ? "true" : "false", turns);
       
       set_verbose(LOG_LVL_DBG);
       mt_reconf(m, NULL, NULL); //back to defaults
       mt_pid_set_gains(m->pid, kp ,ki ,kd);
       m->pid->ttc = ttc;
       
       int mostres = 10;
       
       if(calib){
	 mt_calibrate(mostres, 0.6);
	 printf("\n");
	 printf("coefs_punts_1: ");prw(mostres, m1->pid->cp);
	 printf("coefs_punts_2: ");prw(mostres, m2->pid->cp);
	 printf("coef_dev_1:    ");prw(mostres, m1->pid->cd);
	 printf("coef_dev_2:    ");prw(mostres, m2->pid->cd);
	 printf("\n");
       } else
	 printf("\n");
       
       
       printf("\n\nSTARTING MOVE_T\n\n");
       mt_move_t(m, mt_tticks(m, turns), "f", vel, pctr);mt_wait(m);
       /*printf("\n\nSTARTING MOVE\n\n");
	 move(m, "f", vel);
	 while(get_in("stop?", 1) != 1);
	 mot_stop(m,true);
       */
       
       //while (m->moving)
       //	udelay(1000);
     }
     break;;
   case 10:
     {
       
       int vel = argc < 3 ? 70 : atoi(argv[2]);
       double ttc = argc < 4 ? TTCDEF : atof(argv[3]);
       double kp = argc < 5 ? 1 : atof(argv[4]);
       double ki = argc < 6 ? 1 : atof(argv[5]);
       double kd = argc < 7 ? 1 : atof(argv[6]);
       bool calib = argc < 8 ? false : atoi(argv[7]) != 0 ? true : false;
       
       //            bool hard = argc < 9 ? false : atoi(argv[8]) != 0 ? true : false;
       
       //int turns = argc < 9 ? 10 : atoi(argv[8]);
       
       set_verbose(LOG_LVL_DBG);
       printf ("vel: %d, ttc: %f, kp: %f, ki: %f, kd: %f, calib: \"%s\"\n", vel, ttc, kp, ki, kd, calib ? "true" : "false");
       
       mt_reconf(m1, NULL, NULL); //back to defaults
       mt_reconf(m2, NULL, NULL); //back to defaults
       
       mt_pid_set_gains(m1->pid, kp ,ki ,kd);
       mt_pid_set_gains(m2->pid, kp ,ki ,kd);
       
       m1->pid->ttc = ttc;
       m2->pid->ttc = ttc;
       
       int mostres = 10;
       
       if(calib){
	 mt_calibrate(mostres, 0.6);
	 printf("\n");
	 printf("coefs_punts_1: ");prw(mostres, m1->pid->cp);
	 printf("coefs_punts_2: ");prw(mostres, m2->pid->cp);
	 printf("coef_dev_1:    ");prw(mostres, m1->pid->cd);
	 printf("coef_dev_2:    ");prw(mostres, m2->pid->cd);
	 printf("\n");
       } else
	 printf("\n");
       
       mt_move_sinc("fwd", vel);
       while(get_in("stop?", 1) != 1);
       mt_stop(m1,true);
       mt_stop(m2,true);
     }
     break;;
   case 11:
     {
       set_verbose(LOG_LVL_DBG);
       int vel = argc < 3 ? 70 : atoi(argv[2]);
       double ttc = argc < 4 ? TTCDEF : atof(argv[3]);
       double kp = argc < 5 ? 1 : atof(argv[4]);
       double ki = argc < 6 ? 1 : atof(argv[5]);
       double kd = argc < 7 ? 1 : atof(argv[6]);
       bool calib = argc < 8 ? false : atoi(argv[7]) != 0 ? true : false;
       //          bool hard = argc < 9 ? false : atoi(argv[8]) != 0 ? true : false;
       double pctr = argc < 9 ? 0.3 : atof(argv[8]);
       int turns = argc < 10 ? 10 : atoi(argv[9]);
       
       //			int turns = argc < 9 ? 10 : atoi(argv[8]);
       
       printf ("vel: %d, ttc: %f, kp: %f, ki: %f, kd: %f, calib: \"%s\"\n", vel, ttc, kp, ki, kd, calib ? "true" : "false");
       
       mt_reconf(m1, NULL, NULL); //back to defaults
       mt_reconf(m2, NULL, NULL); //back to defaults
       
       mt_pid_set_gains(m1->pid, kp ,ki ,kd);
       mt_pid_set_gains(m2->pid, kp ,ki ,kd);
       //                pid_setnull(m1->pid);
       //                pid_setnull(m2->pid);
       
       m1->pid->ttc = ttc;
       m2->pid->ttc = ttc;
       
       int mostres = 10;
       
       if(calib){
	 mt_calibrate(mostres, 0.6);
	 printf("\n");
	 printf("coefs_punts_1: ");prw(mostres, m1->pid->cp);
	 printf("coefs_punts_2: ");prw(mostres, m2->pid->cp);
	 printf("coef_dev_1:    ");prw(mostres, m1->pid->cd);
	 printf("coef_dev_2:    ");prw(mostres, m2->pid->cd);
	 printf("\n");
       } else
	 printf("\n");
       
       int res = mt_move_sinc_t("fwd", vel, (turns*720), pctr);mt_wait_all();
       printf("move_sinc-says: \"%s\"\n", res ? "OK" : "FAIL");
       //while(get_in("stop?", 1) != 1);
       mt_stop(m1,true);
       mt_stop(m2,true);
     }
     break;;
   default:
     break;;
   }
 } else{
   printf("ERRORAKO en init de motores\n");
 }
 
 mt_stop(m1, true);
 mt_stop(m2, true);
 //mt_shutdown();
 lego_shutdown();
 
 return ret;
 
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
//	printf("DEBUG: printing txsec"); prac(len, txs_acum);
//	printf("DEBUG: printing stattxs"); prac(STATS_SIZE, stattxs.res);
	avg_tturn = statturn.res[0];
	absd_tturn = statturn.res[3];
	avg_txs = stattxs.res[0];
    absd_txs = stattxs.res[3];
	printf("\t\t\t AVERAGE\t\tABS_DEV\t\t\n\n");
	printf("temps x volta:\t\t %.5f\t\t%.5f\n", avg_tturn*0.000001, absd_tturn*0.000001);
	printf("ticks x segon:\t\t %.5f\t\t%.5f\n\n", avg_txs, absd_txs);
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
				printf("t_entre_ticks: \t\t%.5f\t\t%.5f\n", mean, absd);
				break;;
			case 2:
				printf("range_min: \t\t%.5f\t\t%.5f\n", mean, absd);
				break;;
			case 3:
				printf("range_max: \t\t%.5f\t\t%.5f\n", mean, absd);
				printf("min_rmin: \t\t%.5f\n", min_r);
				printf("min_rmax: \t\t%.5f\n", max_r);
				break;;
		}
	}
	printf("\n");
}


void init_acums (int turns, MOTOR * m){

//    int size = (int)((BASE*turns)+((BASE*turns)*0.2));
    int size = (int)(BASE*turns);
    if(m->id == 1){
        if(!mt_enc_is_null(m->enc1)) {
            if( (acum1 = (double *)calloc(size,sizeof(double))) == NULL)
				printf("Init ac1 fails\n");
			}
        if(!mt_enc_is_null(m->enc2)) {
            if( (acum3 = (double *)calloc(size,sizeof(double))) == NULL)
				printf("Init ac3 fails\n");
			}
    } else {
        if(!mt_enc_is_null(m->enc1)) {
            if( (acum2 = (double *)calloc(size,sizeof(double))) == NULL)
				printf("Init ac2 fails\n");
		}
        if(!mt_enc_is_null(m->enc2)) {
            if( (acum4 = (double *)calloc(size,sizeof(double))) == NULL)
				printf("Init ac4 fails\n");
		}
    }
}

/*
void reinit_acums (int turns, MOTOR * m){

//    int size = (int)((BASE*turns)+((BASE*turns)*0.2));
    int size = (int)(BASE*turns);
    if( m->id == 1 ){
        if(!is_null(m->enc1)) {
            //acum1 = NULL;
			acum1 = (double *)realloc(acum1,size*sizeof(double));
            memset(acum1,0.0,size*sizeof(double));

		}

		 if(!is_null(m->enc2)) {
			//acum3 = NULL;
            acum3 = (double *)realloc(acum3,size*sizeof(double));
            memset(acum3,0.0,size*sizeof(double));
		}

    } else {
        if(!is_null(m->enc1)) {
           //acum2 = NULL;
		   acum2 = (double *)realloc(acum2,size*sizeof(double));
           memset(acum2,0.0,size*sizeof(double));

		}
        if(!is_null(m->enc2)) {
           //acum4 = NULL;
		   acum4 = (double *)realloc(acum4,size*sizeof(double));
           memset(acum4,0.0,size*sizeof(double));
		}
    }
}
*/
void free_acums(MOTOR * m){

    if( m->id == 1 ){
        if(!mt_enc_is_null(m->enc1))
			free(acum1);
        if(!mt_enc_is_null(m->enc2))
			free(acum3);
    } else {
        if(!mt_enc_is_null(m->enc1))
          	free(acum2);
        if(!mt_enc_is_null(m->enc2))
            free(acum4);
    }
}

void reset_acums(int turns, MOTOR * m){

//    int size = (int)((BASE*turns)+((BASE*turns)*0.2));
    int size = (int)(BASE*turns);
    if( m->id == 1 ){
        if(!mt_enc_is_null(m->enc1))
            memset(acum1,0.0,size*sizeof(double));
        if(!mt_enc_is_null(m->enc2))
            memset(acum3,0.0,size*sizeof(double));
    } else {
        if(!mt_enc_is_null(m->enc1))
            memset(acum2,0.0,size*sizeof(double));
        if(!mt_enc_is_null(m->enc2))
            memset(acum4,0.0,size*sizeof(double));
    }
}


/*
void init_acums(int turns){

	int size = ((BASE*turns)+((BASE*turns)*0.2));

	if(!is_null(m1->enc1)){
		acum1 = (double *)realloc(acum1,size*sizeof(double));
//		acum1 = (double *)malloc(size*sizeof(double));
		//acum1[size-1] = '\0';
		printf("m1e1 NO NULL\n");
	}
	if(!is_null(m1->enc2)){
		acum3 = (double *)realloc(acum3,size*sizeof(double));
//		acum3 = (double *)malloc(size*sizeof(double));
		//acum3[size-1] = '\0';
		printf("m1e2 NO NULL\n");
	}
	if(!is_null(m2->enc1)){
		acum2 = (double *)realloc(acum2,size*sizeof(double));
//		acum2 = (double *)malloc(size*sizeof(double));
		//acum2[size-1] = '\0';
		printf("m2e1 NO NULL\n");
	}
	if(!is_null(m2->enc2)){
		acum4 = (double *)realloc(acum4,size*sizeof(double));
//		acum4 = (double *)malloc(size*sizeof(double));
		//acum4[size-1] = '\0';
		printf("m2e2 NO NULL\n");
	}

	printf("Going out!!\n");
}

void reset_acums(int turns){

	int size = ((BASE*turns)+((BASE*turns)*0.2));
    if(!is_null(m1->enc1)){
        memset(acum1,0.0,size*sizeof(double));
		acum1[size-1] = '\0';
	}
    if(!is_null(m1->enc2)){
		memset(acum3,0.0,size*sizeof(double));
		acum3[size-1] = '\0';
	}
    if(!is_null(m2->enc1)){
		memset(acum2,0.0,size*sizeof(double));
		acum2[size-1] = '\0';
	}
    if(!is_null(m2->enc2)){
		memset(acum4,0.0,size*sizeof(double));
		acum4[size-1] = '\0';
	}
}
*/
void close_acums(int ticks){

    //int size = ((BASE*turns)+((BASE*turns)*0.2));
    if(!mt_enc_is_null(m1->enc1))
        acum1[ticks] = '\0';
    if(!mt_enc_is_null(m1->enc2))
        acum3[ticks] = '\0';
    if(!mt_enc_is_null(m2->enc1))
        acum2[ticks] = '\0';
    if(!mt_enc_is_null(m2->enc2)){
        acum4[ticks] = '\0';
    }
}



void comp_res(RESULT * res, int mostres, int step, int id){
  
  int i, velo, index;
  double * frame[mostres]; //= (double **)malloc(mostres * sizeof(double *));
  
  printf("\t\t\t\t\t AVERAGE\t\tVARIANCE\t\tSTND_DEV\t\tABS_DEV\t\t\n\n");
  
  for (velo = step; velo <= MAX_VEL; velo += step){
    for (i = 0; i < mostres; i++){
      index = ((((velo/step)-1)*mostres) + i);
      //			printf("scanning index: %d ===> ", index);
      frame[i] = res[index].res;
    }
    comp_vel(velo, mostres, frame, id);
    
    for (i = 0; i < mostres; i++){
      index = ((((velo/step)-1)*mostres) + i);
      frame[i] = res[index].per;
      /*printf("per k le meto: ");
	prw(PER_SIZE, res[index].per);*/
    }
    comp_perc(velo, mostres, frame, id);
    printf("\n");
  }
  printf("---------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
}

void comp_perc(int vel, int mostres, double ** frame, int id){
  
  int k;
  double sum = 0;
  double aux[mostres];// = (double *)malloc(mostres * sizeof(double));
  RESULT stat;//= (RESULT *)malloc(sizeof(RESULT));// = (double *)malloc(STATS_SIZE * sizeof(double));
  
  for (k=0;k<PER_SIZE-1;k++){
    col_to_row(aux, frame, k, mostres);
    stats(&stat, false, aux, false, false, mostres, id, false,0, true);
    sum += stat.res[0];
    //	printf("stat percent, volta %d: ", k);
    //	prw(mostres, aux);
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
      printf("\t\t>>LIADA PADRE!!<<\t\t\n");
      break;;
    }
    pr_stats(stat.res, 4);
  }
  printf ("population_average_sum: \t\t%.5f\n", sum);
}

void comp_vel(int vel, int mostres, double ** frame, int id){
  int k;
  double aux[mostres];// = (double *)malloc(mostres * sizeof(double));
  RESULT stat;// = (RESULT *)malloc(sizeof(RESULT));// = (double *)malloc(STATS_SIZE * sizeof(double));
  
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
      printf("\t\t>>LIADA PADRE!!<<\t\t\n");
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


int get_in(char *to_print, int type){
  switch(type){
  case 1:;;
    int ret_int;
    printf("%s", to_print);
    scanf("\n%d",&ret_int);
    fflush(NULL);
    return ret_int;
  default:;;
    char ret;
    printf("%s", to_print);
    scanf("\n%c",&ret);
    fflush(NULL);
    return ret;
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
  //printf("la i ar fina: %d", i);
  return i;
}


int cptable(double * out, int alloc, double * in){
  
  int i;
  
  for(i=0; i < alloc; i++)
    out[i] = in[i];
  return i;
}


int stats (RESULT * out, bool to_print, double * vect, bool wweights, bool capped, int alloc, int id, bool clean, int encoder, bool no_table){
  
  //	printf("printing ACUM1 (inside stats), ALLOC = %d: \n", alloc);
  //	prac(alloc, acum1);
  
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
  
  /* la varian?a i desviaci? tipica surt molt gran */
  //	printf("DEBUG: ");
  //	prw(STATS_SIZE-1,out->res);
  //	printf("El que treu la funcio de la mitja: %f\n", gsl_stats_mean(data, 1, len-1));
  mean = out->res[0] = gsl_stats_mean(data, 1, len);
  //	printf("MEAAAAN: %f\n",out->res[0]);
  double variance = out->res[1] = gsl_stats_variance_m(data, 1, len, mean);
  double sd = out->res[2] = gsl_stats_sd_m(data, 1, len, mean);
  
  double abs = out->res[3] = gsl_stats_absdev_m (data, 1, len, mean);
  double autocorr = out->res[4] = gsl_stats_lag1_autocorrelation_m (data, 1, len, mean);
  //double perc[PER_SIZE];
  
  /* calcul dels valors amb pesos, "descartem" el 10% dels tics del final i del principi */
  
  if(wweights){
    double weights[len];// = malloc(len * sizeof(double));
    double sweights[len];// = malloc(len * sizeof(double));
    
    ret=2;
    cal_weight(len, weights, 1 ,3);
    
    wmean = out->res[5] = gsl_stats_wmean (weights, 1, data, 1, len);
    wvariance = out->res[6] = gsl_stats_wvariance_m (weights, 1, data, 1, len, wmean);
    wsd = out->res[7] = gsl_stats_wsd_m (weights, 1,data, 1, len, wmean);
    wabs = out->res[8] = gsl_stats_wabsdev_m (weights,1,data,1,len, wmean);
    gsl_stats_minmax(&min, &max, data, 1, len);
    
    gsl_sort (data,1,len);
    smart_weights(len, sweights, 0.1, max, data,id,out->per);
    
    //cp_loc(out->per, id);
    get_limits(out->per, &uq, &lq, 0.70, max, &rmin, &rmax);
    //prw(PER_SIZE-1,perc);
    
    if(to_print){
      printf("uq: %.3f, lq: %.3f\n", uq, lq);
      //			prw(10,per);
    }
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
  //cptable(out->per, PER_SIZE-1, perc);
  //printf("LEN: %d\n", cptable(out->per, PER_SIZE-1, perc));
  
  if(to_print){
    printf("ticks received: %d\nmean: %.5f\nvariance: %.5f\nstandard deviation: %.5f\nmedian: %.5f\nquantil superior: %.5f\nquantil inferior: %.5f\nrmax: %.5f\nrmin: %.5f\n", len, out->res[0], variance, sd, median, upperq, lowerq, rmax, rmin);
    printf("autocorrelation: %.5f\nmin: %.5f\nmax:%.5f\nabsolute_deviation: %.5f\n\nweights: \nwmean: %.5f\nwvariance: %.5f\nwstandard_deviation: %.5f\nwabs_dev: %.5f\n\n", autocorr,min,max,abs, wmean, wvariance, wsd, wabs);
    printf("smart weights: \nswmean: %.5f\nswvariance: %.5f\nswstandard_deviation: %.5f\nswabs_dev: %.5f\n\n",swmean, swvariance, swsd, swabs);
    prw(PER_SIZE-1,out->per);
  }
  //prw(10,per);
  return ret;
  
}

void get_limits(double per[], double * uq, double * lq, double min_pop, double max, double * rmin, double * rmax){
  
  bool found = false;
  int i,k;
  //prw(PER_SIZE-1, per);
  for (i=0; i<10 && !found; i++){
    for(k=i+1; k<10 && !found; k++){
      if(((per[i] + per[k]) >= min_pop))
	found = true;
    }
  }
  k--;i--;
  /*printf("DEBUG: i->%d, k->%d\n", i,k);
    prw(10,per);*/
  double uqbase = 0.5 + (min_pop/2);
  double lqbase = 0.5 - (min_pop/2);
  
  if (found){
    double total = per[i] + per[k];
    if (total > min_pop){
      double to_div = total - min_pop;
      //printf("DEBUG: entro aqui!\n");
      // *uq = (per[k] > (min_pop/2)) ? (per[i] < (min_pop/2)) ? (uqbase + ((per[k]-(min_pop/2)) - ((min_pop/2) - per[i]))) : (uqbase + (per[k]-(min_pop/2))) : uqbase;
      // *lq = (per[i] > (min_pop/2)) ? (per[k] < (min_pop/2)) ? (lqbase - ((per[i]-(min_pop/2)) - ((min_pop/2) - per[k]))) : (lqbase - (per[i]-(min_pop/2))) : lqbase;
      *uq = uqbase +(to_div/2);
      *lq = lqbase -(to_div/2);
      //printf("valores: uq: %f, lq: %f\n", *uq, *lq);
    } else {
      *uq = uqbase;
      *lq = lqbase;
    }
    *rmin = ((double)i/10) * max;
    *rmax = ((double)(k+1)/10) * max;
    //printf("valores: rmin: %f, rmax: %f, i: %d, k: %d, max: %f\n", *rmin, *rmax, i, k, max);
  } else {
    *rmin = 0;
    *rmax = 0;
    *uq = uqbase;
    *lq = lqbase;
  }
  
  //double range_min,range_max;
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
  
  /* La idea: contar la poblacion de cada quantil (menos la del quantil anterior), y, sacar el porcentage respeto a la poblacion total (todos los tics), a partir de ahi assignar los pesos 
     de cada elemento dependiendo del percentage de poblacion del quantil al que corresponda */
  //	int tam = PER_SIZE;
  double quantil;// perc[tam];
  int res;
  int i;

  for (i=0, quantil=step;quantil<=1;quantil+=step, i++){
    res = no_quantil(len,data,quantil,max,step);
    per[i] = (double)((double)res/len);
  }
  per[i]='\0';
  //	cp_per(perc, tam, id);
  //	prw(tam, perc);
  
  for (i=0; i<len; i++){
    weights[i] = per[(int)(((data[i]/max)-step)*10)];
  }
  
}

/*void cp_per (double * per, int tam, int id){
  int i;

  if (id == 1){
		for (i=0;i<tam;i++)
			pop_perc1[i] = per[i];
		pop_perc1[i]='\0';

	} else {
		for (i=0;i<tam;i++)
			pop_perc2[i] = per[i];
		pop_perc2[i]='\0';

	}

}
*/
int no_quantil (int len, double * data, double perc, double max, double step){

  /* NO-quantil, es decir lo coontrario, dado un porcentage del valor del m?ximo, cuanta poblaci?n esta por debajo de ese valor, (Distribuciones?) */
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
  
  //	printf("no-quantil says: act: %d - last: %d = %d | ant = %.5f, ref = %.5f | data content: %.5f\n", i, last, i-last, ult, ref, val);
  return (i - last);
  
}


