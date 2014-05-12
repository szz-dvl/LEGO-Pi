#include <lego.h>


#define US_PORT  1
#define LF_PORT  0
#define LB_PORT  3
#define SND_PORT 1
#define PSH_PORT 2
#define RMT_PORT 1
#define LMT_PORT 0
#define MY_FWD   BWD
#define MY_BWD   FWD 

MOTOR * mtr, * mtl;
ANDVC lfront, lback, snd, push;
DGDVC us;

bool doInit () {
  
  lego_init();
  
    if((mtl = mt_new(NULL, NULL, LMT_PORT)) == NULL) {
      printf("Error initializing left motor.\n");
      return false;
    }
    
    if((mtr = mt_new(NULL, NULL, RMT_PORT)) == NULL) {
      printf("Error initializing right motor.\n");
      return false;
    }
    
    if(!ag_new(&push, PSH_PORT, PUSH)) {
      printf("Error initializing push sensor.\n");
      return false;
    } 

    if(!ag_new(&lback, LB_PORT, LIGHT)) {
      printf("Error initializing back light sensor.\n");
      return false;
    } 

    if(!ag_new(&lfront, LF_PORT, LIGHT)) {
      printf("Error initializing front light sensor.\n");
      return false;
    } 

    if(!ag_new(&snd, SND_PORT, SOUND)) {
      printf("Error initializing sound sensor.\n");
      return false;
    } 
    
    if(!dg_new(&us, LEGO_US, US_PORT)) {
      printf("Error initializing ultrasonic sensor.\n");
      return false;
    } 
  
    return true;
}

void stop_all () {
  
  mt_stop(mtl, true);
  mt_stop(mtr, true);

}

bool conditions_compliant () {

  uint8_t dist;
  double pval;
  bool pushed;

  dg_us_get_dist(&us, &dist, 0);
  pushed = ag_psh_is_pushed(&push, &pval);

  return ((dist < 20) && (pval != -1) && !pushed);

}

bool no_path_found () {

  uint8_t dist;
  //double pval;
  //bool pushed;

  dg_us_get_dist(&us, &dist, 0);
  //pushed = ag_push_is_pushed(&push, &pval);
  printf("looking for path, dist = %u\n", dist);
  return (dist < 20);

}

void rotate_robot (int vel, bool left) {

  if (left){
    mt_move (mtl, MY_BWD, vel);
    mt_move (mtr, MY_FWD, vel);
  } else {
    mt_move (mtl, MY_FWD, vel);
    mt_move (mtr, MY_BWD, vel);
  }
}


void turn_robot (int vel, bool left) {

  if (left)
    mt_move (mtr, MY_FWD, vel);
  else 
    mt_move (mtl, MY_FWD, vel);

}


void move_but_think_stupid_robot (int vel) {
  
  printf("entering move.\n");
  mt_move_sinc(MY_FWD, vel);
  
  while (conditions_compliant());

  stop_all();

} 

void look_for_another_path_nasty_machine (bool rotate, int vel) {

  int flight, blight;
  
  printf("entering path finding.\n");
  flight = ag_read_int(&lfront);
  blight = ag_read_int(&lback);

  if(rotate)
    rotate_robot(vel, flight > blight);
  else 
    turn_robot(vel, flight > blight);

  while (no_path_found());

  stop_all();

}

int main (int argc, char * argv[]) {


  if(!doInit()) {
    
    printf("Error Initializing Stuff.\n");
    exit(EXIT_FAILURE);

  } else { //aki fiesta

    int pcount = 0;
    int vel = argc < 2 ? 70 : atoi(argv[1]);
    int limit = argc < 3 ? 50 : atoi(argv[2]);
    bool rotate = argc < 4 ? true : atoi(argv[3]) != 0 ? true : false;

    while (pcount < limit) {
      move_but_think_stupid_robot(vel);
      look_for_another_path_nasty_machine(rotate, vel/2);
      pcount ++;
    
    }

    exit(EXIT_SUCCESS);
  }




} 

