#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <stdbool.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>

#define FAIL		-1
#define LOG_LVL_DBG      2
#define LOG_LVL_ADV      1
#define LOG_LVL_FATAL    0

#define DELAY_US(t)      nanosleep((TSPEC*)&(TSPEC){0, t*1000}, NULL) 

#ifndef SHAREFILE_INCLUDED
#define SHAREFILE_INCLUDED

struct init_struct {

  bool wpi;
  bool mt;
  bool ag;
  bool dg;

};

typedef struct init_struct INIT;


#ifdef  MAIN_FILE

INIT status = {false, false, false, false};

#else

extern INIT status;

#endif
#endif

typedef struct timespec TSPEC;

extern double difft (TSPEC *, TSPEC *);

void unexportall();
void setup_sighandlers(void);

