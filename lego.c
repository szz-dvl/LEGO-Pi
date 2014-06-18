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

#define MAIN_FILE
#include "lego_shared.h"
#include "lego_motor.h"
#include "lego_analog.h"
#include "lego_digital.h"

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

void setup_sighandlers(void){

  int i;
  for (i = 0; i < 64; i++) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    
    if( i != 17 && i != 26 && i != 28) { //avoid sigchld, sigalrm and sigvtalrm, sivtalarm received in some situations, to investigate...
      sa.sa_handler = (void *) terminate;
      sigaction(i, &sa, NULL);
    } else if ( i == 26 ){ //may be modified in next versions
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
