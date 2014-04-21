#include <lego/lego_analog.h>

int main(int argc, char * argv[]) {

  int times = argc < 2 ? 5 : atoi(argv[1]), i;
  ANDVC push, l1, l2, l3;       
  double lres1, lres2, lres3;
  //set_verbose(LOG_LVL_ADV);
  
  new_analog(&push, 3, PUSH);
  new_analog(&l1,0, LIGHT);
  new_analog(&l2,1, LIGHT);
  new_analog(&l3,2, LIGHT);
  
  for(i=0; i<times; i++){
    while(!analog_pushed(&push));
    
    if (l1.l_on) {
      printf("entro l1 light on\n");
      analog_light_off(&l1);
    } else { 
      analog_light_on(&l1);
      printf("entro l1 light off\n");
    }
    if (l2.l_on)
      analog_light_off(&l2);
    else
      analog_light_on(&l2);
    
    if (l3.l_on)
      analog_light_off(&l3);
    else
      analog_light_on(&l3);
    
    sleep(1);
    
    lres1 = analog_read_voltage(&l1);
    lres2 = analog_read_voltage(&l2);
    lres3 = analog_read_voltage(&l3);
    printf("LIGHT_1 says: %.2f, LIGHT_2 says: %.2f, LIGHT_3 says: %.2f\n", lres1, lres2, lres3);
    
  }

  return (EXIT_SUCCESS);
}

