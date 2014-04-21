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

#define OK		 1
#define FAIL		-1
#define WEIRD		-2

typedef struct timespec TSPEC;

extern void udelay(int);
extern double difft (TSPEC *, TSPEC *);
//extern void lego_shutdown (void);

void unexportall();
void fatal (char *fmt, ...);
void not_critical (char *fmt, ...);
void debug (char *fmt, ...);
void setup_sighandlers(void);

///extern void lego_init (void);
//extern void lego_shutdown (void);
//extern void set_verbose (int);
