#include <lego/lego_analog.h>
//#include <lego.h>
#include <string.h>
#include <math.h>

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

  ag_init(10);
  //set_bverbose(LOG_LVL_DBG);

  int tst = argc < 2 ? 5 : atoi(argv[1]);
  
  switch(tst) {
  case 1:
    {
      int times = argc < 3 ? 6 : atoi(argv[2]), i;
      ANDVC push, l2, l3;       
      double lres2 = 0, lres3 = 0, psh_val = 0;
      char straux[] = {'\0'};

      ag_new(&push, 0, PUSH);
      ag_new(&l2,1, LIGHT);
      ag_new(&l3,3, LIGHT);
      
      for(i=0; i<times; i++){
	while(!ag_psh_is_pushed(&push, &psh_val));
	
	sprintf(straux, "Setting ligh %s on port %d\n", ag_lgt_get_ledstate(&l2) ? "OFF" : "ON", l2.port);
	//get_in(straux, 1);
	printf("%s",straux);
	if (ag_lgt_get_ledstate(&l2))
	  ag_lgt_set_led(&l2, false);
	else  
	  ag_lgt_set_led(&l2, true);
	
	//strcpy(straux, "");
	sprintf(straux, "Setting ligh %s on port %d\n", ag_lgt_get_ledstate(&l3) ? "OFF" : "ON", l3.port);
	//get_in(straux, 1);
	if (ag_lgt_get_ledstate(&l3))
	  ag_lgt_set_led(&l3, false);
	else  
	  ag_lgt_set_led(&l3, true);
        
	DELAY_US(200000);
	lres3 = ag_read_volt(&l3);
	lres2 = ag_read_volt(&l2);
	printf("PUSH_VAL: %.2f, LIGHT_2 says: %.2f, LIGHT_3 says: %.2f, VDIFF = %.2f\n", psh_val, lres2, lres3, fabs(lres2-lres3));
	//while(ag_psh_is_pushed(&push, &psh_val));
      }
    }
    break;;
  case 2:
    {
      int lport = argc < 3 ? 3 : atoi(argv[2]);
      char straux [] = {'\0'};
 
      ANDVC light;
      ag_new(&light,lport, LIGHT);


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
  case 3:
    {
      
      int port = argc < 3 ? 2 : atoi(argv[2]);
      ANDVC snd;
      ag_new(&snd,port, SOUND);
      
      while (1) {
	
	printf("SOUND says: %d dB, read_int = %d, read_volt = %f\n", ag_snd_get_db(&snd), ag_read_int(&snd), ag_read_volt(&snd));
	sleep(1);
	
      }

    }
    break;;
  case 4:
    {
      
      int port = argc < 3 ? 1 : atoi(argv[2]);
      ANDVC gyro;
      bool error;
      
      ag_new(&gyro,port,HT_GYRO);

      ag_gyro_cal(&gyro);
      
      while (1) {
	
	printf("GYRO says: %d, read_volt = %f, read_int = %d\n", ag_gyro_get_val(&gyro, &error), ag_read_volt(&gyro), ag_read_int(&gyro));
	sleep(1);
	
      }

    }
    break;;
  case 5: //try to get the value for stationary robot
    {
      
      int port = argc < 3 ? 1 : atoi(argv[2]), i;
      int times = argc < 4 ? 1 : atoi(argv[3]);
      uint64_t acum = 0;
      int partial;
      ANDVC gyro;
      //bool error;
      
      ag_new(&gyro,port,HT_GYRO);

      ag_gyro_cal(&gyro);
      
      for(i=0; i<times; i++) {
	
	partial = ag_read_int(&gyro);
	acum += (uint64_t) partial;
	//printf("reading %d: %d\n", i, partial);
	//printf("GYRO says: %d, read_volt = %f, read_int = %d\n", ag_gyro_get_val(&gyro, &error), ag_read_volt(&gyro), ag_read_int(&gyro));
      }
      
      printf("average reading for %d samples: %d\n", times, (int)((double)acum/times));
    }
    break;;
      case 6: //try to get the value for stationary robot
    {
      
      int port = argc < 3 ? 1 : atoi(argv[2]), i;
      int times = argc < 4 ? 1 : atoi(argv[3]);
      ANDVC lother;
      bool to_set = false;
      //bool error;
      
      ag_new(&lother,port,AG_OTHER);
      
      for(i=0; i<times; i++) {
	ag_oth_set_y(&lother,to_set);
	to_set = !to_set;
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

