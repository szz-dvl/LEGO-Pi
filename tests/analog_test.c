#include <lego/lego_analog.h>

int main(int argc, char * argv[]) {

  ag_init();
  
  int times = argc < 2 ? 5 : atoi(argv[1]), i;
  ANDVC push, l1, l2, l3;       
  double lres1, lres2, lres3, psh_val;
  //set_verbose(LOG_LVL_ADV);
  
  ag_new(&push, 2, PUSH);
  ag_new(&l1,0, LIGHT);
  ag_new(&l2,1, LIGHT);
  ag_new(&l3,3, LIGHT);
  
  for(i=0; i<times; i++){
    while(!ag_psh_is_pushed(&push, &psh_val));

    if (ag_lgt_get_ledstate(&l1))
      ag_lgt_set_led(&l1, false);
    else  
      ag_lgt_set_led(&l1, true);
    
    if (ag_lgt_get_ledstate(&l2))
      ag_lgt_set_led(&l2, false);
    else  
      ag_lgt_set_led(&l2, true);
    
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

  ag_shutdown();
  return (EXIT_SUCCESS);
}

