//#include "lego.h"
#include "lego_shared.h"
#include "lego_motor.h"
#include "lego_analog.h"
#include "lego_digital.h"

//#include <lego/lego_i2c.h>

#define PRINT            1
#define QUIET            0
#define LOG_LVL_DBG      2
#define LOG_LVL_ADV      1
#define LOG_LVL_FATAL    0

static int pr_criticals = PRINT;
static int pr_debug = QUIET;

static void LEGO_shutdown (void);
static void handl_alrm(void);
static void terminate(void);

void fatal (char *fmt, ...) {

  va_list ap;
  
  va_start(ap, fmt);
  vfprintf(stderr, fmt, ap);
  va_end(ap);

  // Play that funky music white boy
  LEGO_shutdown();
  exit(EXIT_FAILURE);
}

void not_critical (char* fmt, ...) {

  if (pr_criticals == QUIET)
    return;

  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);
  
}

void debug (char* fmt, ...) {
  
  if (pr_debug == QUIET)
    return;
  
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);

}

void set_verbose (int lvl) {
  
  if (lvl == LOG_LVL_FATAL) {
    pr_criticals = QUIET;
    pr_debug = QUIET;
    i2c_set_loglvl(LOG_QUIET);
  } else if (lvl == LOG_LVL_ADV) {
    pr_criticals = PRINT;
    pr_debug = QUIET;
    i2c_set_loglvl(LOG_QUIET);
  } else if (lvl == LOG_LVL_DBG) {
    pr_criticals = PRINT;
    pr_debug = PRINT;
    i2c_set_loglvl(LOG_PRINT);
  } else 
    printf("Log level \"%d\" out of bounds\n", lvl);
    
}

static void LEGO_shutdown () {

  mot_clear();
  analog_shutdown();
  exit(EXIT_FAILURE);
}

void unexportall(){
  
  system("/usr/local/bin/gpio unexportall");
  
}

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

static void handl_alrm(void) {}


static void terminate(void) {
  
  mot_clear();
  analog_shutdown();  
  exit(EXIT_SUCCESS);

}

void lego_init () {
  
  wiringPiSetupGpio();
  init_motors();
  analog_setup();

}

void lego_shutdown (void) {
  
  mot_clear();
  analog_shutdown();
  exit(EXIT_SUCCESS);

}
