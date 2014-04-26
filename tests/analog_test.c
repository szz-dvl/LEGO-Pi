#include <lego/lego_analog.h>
#include <string.h>

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

  ag_init();

  int tst = argc < 2 ? 5 : atoi(argv[1]);
  
  switch(tst) {
  case 1:
    {
      int times = argc < 3 ? 6 : atoi(argv[2]), i;
      ANDVC push, l1, l2, l3;       
      double lres1, lres2, lres3, psh_val;
      //set_verbose(LOG_LVL_ADV);
      char straux[] = {'\0'};

      get_in("sa pagao?\n", 1);
      ag_new(&push, 3, PUSH);
      ag_new(&l1,0, LIGHT);
      ag_new(&l2,1, LIGHT);
      ag_new(&l3,2, LIGHT);
      
      for(i=0; i<times; i++){
	while(!ag_psh_is_pushed(&push, &psh_val));
	
	sprintf(straux, "Setting ligh %s on port %d\n", ag_lgt_get_ledstate(&l1) ? "OFF" : "ON", l1.port);
	get_in(straux, 1);
	if (ag_lgt_get_ledstate(&l1))
	  ag_lgt_set_led(&l1, false);
	else  
	  ag_lgt_set_led(&l1, true);
	
	//strcpy(straux, "");
	sprintf(straux, "Setting ligh %s on port %d\n", ag_lgt_get_ledstate(&l2) ? "OFF" : "ON", l2.port);
	get_in(straux, 1);
	if (ag_lgt_get_ledstate(&l2))
	  ag_lgt_set_led(&l2, false);
	else  
	  ag_lgt_set_led(&l2, true);
	
	//strcpy(straux, "");
	sprintf(straux, "Setting ligh %s on port %d\n", ag_lgt_get_ledstate(&l3) ? "OFF" : "ON", l3.port);
	get_in(straux, 1);
	if (ag_lgt_get_ledstate(&l3))
	  ag_lgt_set_led(&l3, false);
	else  
	  ag_lgt_set_led(&l3, true);
	
	sleep(1);
	
	lres1 = ag_read_volt(&l1);
	lres2 = ag_read_volt(&l2);
	lres3 = ag_read_volt(&l3);
	printf("PUSH_VAL: %.2f, LIGHT_1 says: %.2f, LIGHT_2 says: %.2f, LIGHT_3 says: %.2f\n", psh_val, lres1, lres2, lres3);
	
      }
    }
    break;;
  case 2:
    {
      int lport = argc < 3 ? 3 : atoi(argv[2]);
      char straux [] = {'\0'};
 
      ANDVC light;
      ag_new(&light,lport, LIGHT);


      while ( get_in("continue...?", 1) != 0 ) {

	sprintf(straux, "Setting ligh %s on port %d\n", ag_lgt_get_ledstate(&light) ? "OFF" : "ON", light.port);
      
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
      ag_new(&gyro,port,HT_GYRO);
  
      ag_gyro_cal(&gyro);
      
      while (1) {
	
	printf("GYRO says: %d, read_volt = %f\n", ag_gyro_get_val(&gyro), ag_read_volt(&gyro));
	sleep(1);
	
      }

    }
    break;;
  default:
    break;;
  }
  ag_shutdown();
  return (EXIT_SUCCESS);
}

