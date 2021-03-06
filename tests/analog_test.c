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

#include <lego/lego_analog.h>
#include <string.h>
#include <math.h>
#include <getopt.h>

static int tst = 0;
static int port = 0;
static int times = 6;
static bool log_dbg = false;
static bool tset = false;

static void print_usage(const char *prog)
{
	printf("Usage: %s -t<test_num> [-pid]\n", prog);
	puts("  -t --test      test number to perform [1-6]           {mandatory}\n"
	     "  -p --port      analog port [0-3]                      {0}\n"
	     "  -i --times     iterations, only apply to some tests   {6}\n"
	     "  -d --dbg       Set the library in debug mode [no arg] {disabled}\n");
	exit(EXIT_FAILURE);
}


   
static void parse_opts(int argc, char *argv[])
{
  while (1) {
    static const struct option lopts[] = {
      { "test",    1, 0, 't' },
      { "times" ,  1, 0, 'i' },
      { "port",    1, 0, 'p' },
      { "dbg" ,    0, 0, 'd' },
      {0, 0, 0, 0},
    };
      
    int c;
    
    c = getopt_long(argc, argv, "t:i:p:d", lopts, NULL);
      
    if (c == -1)
      break;
    
    switch (c) {
    case 't':
      tst = atoi(optarg);
      break;
    case 'i':
      times = atoi(optarg);
      tset = true;
      break;
    case 'p':
      port = atoi(optarg);
      break;
    case 'd':
      log_dbg = true;
      break;
    default:
      print_usage(argv[0]);
      break;
    }
  }
}

//usefull for testing
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
  

int main(int argc, char * argv[]) {
  
  parse_opts(argc, argv);
  
  ag_init(10);
  ag_set_verbose(log_dbg ? LOG_LVL_DBG : LOG_LVL_ADV);
  
  switch(tst) {
  case 0:
    printf("%s: At least the test number is needed!\n", argv[0]);
    ag_shutdown();
    print_usage(argv[0]);
  case 1: //LEGO Push + LEGO Light sensor test
    {
      int i;
      ANDVC push, l2, l3;       
      double lres2 = 0, lres3 = 0, psh_val = 0;
      char straux[] = {'\0'};

      ag_new(&push, 0, PUSH);
      ag_new(&l2,1, LIGHT);
      ag_new(&l3,3, LIGHT);
      
      for(i=0; i<times; i++){
	while(!ag_psh_is_pushed(&push, &psh_val));
	
	sprintf(straux, "Setting ligh %s\n", ag_lgt_get_ledstate(&l2) ? "OFF" : "ON");
	printf("%s",straux);
	if (ag_lgt_get_ledstate(&l2))
	  ag_lgt_set_led(&l2, false);
	else  
	  ag_lgt_set_led(&l2, true);
	
	if (ag_lgt_get_ledstate(&l3))
	  ag_lgt_set_led(&l3, false);
	else  
	  ag_lgt_set_led(&l3, true);
        
	DELAY_US(200000);
	lres3 = ag_read_volt(&l3);
	lres2 = ag_read_volt(&l2);
	printf("PUSH_VAL: %.2f, LIGHT_1 says: %.2f, LIGHT_3 says: %.2f, VDIFF = %.2f\n", psh_val, lres2, lres3, fabs(lres2-lres3));
      }
    }
    break;;
  case 2: //LEGO Light sensor extra light led test
    {
      char straux [] = {'\0'};
 
      ANDVC light;
      ag_new(&light,port, LIGHT);


      while ( get_in("continue...? ", 1) != 0 ) {

	sprintf(straux, "Setting ligh %s on port %d ", ag_lgt_get_ledstate(&light) ? "OFF" : "ON", light.port);
      
	get_in(straux, 1);

	if (ag_lgt_get_ledstate(&light))
	  ag_lgt_set_led(&light, false);
	else  
	  ag_lgt_set_led(&light, true);
	
      }

    }
    break;;
  case 3: //LEGO Sound sensor test
    {
      int i;
      ANDVC snd;
      ag_new(&snd,port, SOUND);
      
      for (i=0; i<times; i++) {
	
	printf("SOUND says: %d dB, read_int = %d, read_volt = %f\n", ag_snd_get_db(&snd), ag_read_int(&snd), ag_read_volt(&snd));
	sleep(1);
	
      }

    }
    break;;
  case 4: //HT_Gyroscope Test
    {
      ANDVC gyro;
      bool error;
      
      ag_new(&gyro,port,HT_GYRO);

      if(tset)
	ag_gyro_cal(&gyro, times);
      
      while (1) {
	
	printf("GYRO says: %d, read_volt = %f, read_int = %d\n", ag_gyro_get_val(&gyro, &error), ag_read_volt(&gyro), ag_read_int(&gyro));
        DELAY_US(200000);
	
      }

    }
    break;;
  case 5: //HT_Gyroscope try to get the value for stationary robot
    {
      
      uint64_t acum = 0;
      int partial, i;
      ANDVC gyro;
      
      ag_new(&gyro,port,HT_GYRO);
      
      for(i=0; i<times; i++) {
	
	partial = ag_read_int(&gyro);
	acum += (uint64_t) partial;
      
      }
      
      printf("average reading for %d samples: %d\n", times, (int)((double)acum/times));
    }
    break;;
      case 6: //AG_OTHER Test;
    {
      
      int i;
      ANDVC lother;
      bool to_set = false;
      
      ag_new(&lother,port,AG_OTHER);
      
      for(i=0; i<times; i++) {
	printf("Delivering yellow wire %s V\n", to_set ? "3.3" : "0");
	ag_oth_set_y(&lother,to_set);
	to_set = !to_set;
	DELAY_US(10000);
	printf("UNKNOWN SAYS: volt = %.2f, int = %d\n", ag_read_volt(&lother), ag_read_int(&lother));
	if(!get_in("Continue?", 1))
	  break;
      }
      
    }
    break;;
  default:
    break;;
  }
  ag_shutdown();
  return (EXIT_SUCCESS);
}

