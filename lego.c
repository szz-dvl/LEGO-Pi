#define MAIN_FILE
#include "lego_shared.h"
#include "lego_motor.h"
#include "lego_analog.h"
#include "lego_digital.h"

//INIT status;

static void handl_alrm(void);
static void terminate(int);

bool lego_set_verbose (int lvl) {
  
  bool ret = true;
  if(status.mt)
    ret = mt_set_verbose(lvl) ;
  if(status.dg)
    ret = dg_set_verbose(lvl) && ret ; 
  if(status.ag)
    ret = ag_set_verbose(lvl) && ret ;

  return ret;
}

void unexportall(){
  
  system("/usr/local/bin/gpio unexportall");
  
}

/*
void udelay (int us) {
  
  TSPEC tini, tfi;
  clock_gettime(CLK_ID, &tini);
  clock_gettime(CLK_ID, &tfi);
  
  while( difft(&tini, &tfi) < us)
    clock_gettime(CLK_ID, &tfi);
  
}


double difft (TSPEC * ini, TSPEC * fi){

  long enano = (fi->tv_nsec - ini->tv_nsec);
  int esec = (int)(fi->tv_sec - ini->tv_sec);
  return ((double) esec*1000000+(enano/1000)); //microseconds

}
*/

void setup_sighandlers(void){

  int i;
  for (i = 0; i < 64; i++) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    
    if( i != 17 && i != 26) { //avoid sigchld and sigalrm
      sa.sa_handler = (void *) terminate;
      sigaction(i, &sa, NULL);
    } else if ( i == 26 ){
      sa.sa_handler = (void *) handl_alrm;
      sigaction(i, &sa, NULL);
    }
  }
}

static void handl_alrm(void) {
  printf("ALARM!!\n");
}


static void terminate(int signum) {
  
  printf("LEGO-Pi: Entering terminate, received signal = %d\n", signum);
  if(status.mt)
    mt_shutdown();
  if(status.ag)
    ag_shutdown();
  if(status.dg)
    dg_shutdown();
  unexportall();
  exit(EXIT_FAILURE);

}

bool lego_init (int dg_retires, int ag_avg) {
  
  status.wpi = wiringPiSetupGpio() == 0;
  status.mt = mt_init();
  status.ag = ag_init(ag_avg);
  status.dg = dg_init(dg_retires);
  
  return (status.dg && status.ag && status.mt);
}

void lego_shutdown (void) {
  
  if(status.mt)
    mt_shutdown();
  if(status.ag)
    ag_shutdown();
  if(status.dg)
    dg_shutdown();

  exit(EXIT_SUCCESS);

}
