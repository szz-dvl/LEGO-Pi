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
#define DIFFT(ti,tf)     (((*tf).tv_nsec - (*ti).tv_nsec)/1000 + ((*tf).tv_sec - (*ti).tv_sec)*1000000)

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


void unexportall();
void setup_sighandlers(void);

