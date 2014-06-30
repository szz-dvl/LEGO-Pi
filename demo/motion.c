#include <math.h>
#include <getopt.h>
#include <lego.h>


#define US_PORT   0
#define PSH_PORT  2
#define RMT_PORT  1
#define LMT_PORT  0
#define MY_FWD    BWD
#define MY_BWD    FWD 
#define SAFE_DST  45
#define INI_DEG   35
#define UDELAY(t) DELAY_US(t)

#define WHEEL_RADIUS 2.5 //cm
#define MOTOR_AXIS   18  //cm


MOTOR * mtr, * mtl;
ANDVC push;
DGDVC us;

static int vel = 90;
static int dist = SAFE_DST;
static bool sincro = false;
static bool rotate = false;
static int enc = 3;
static int txt = 720;
static int deg_step = 35;
static bool left = false;

static void print_usage(const char *prog)
{
	printf("Usage: %s -t<test_num> [-pid]\n", prog);
	puts("  -v --vel       velocity, def = 90.\n"
	     "  -s --sincro    enable sincro.\n"
	     "  -d --dist      distance to change dir.\n"
	     "  -r --rotate    rotate instead of turning.\n"
	     "  -e --enc       encoder lines enabled per motor.[1-2], <3> both.\n"
	     "  -l --left      turn to the left.\n"
	     "  -p --dstep     degrees step to look for path.\n");
	exit(EXIT_FAILURE);
}


   
static void parse_opts(int argc, char *argv[])
{
  while (1) {
    static const struct option lopts[] = {
      { "vel",    1, 0, 'v'},
      { "dist" ,  1, 0, 'd'},
      { "sincro", 0, 0, 's'},
      { "rotate", 0, 0, 'r'},
      { "enc",    1, 0, 'e'},
      { "left",   0, 0, 'l'},
      { "dstep",  1, 0, 'p'},
      {0, 0, 0, 0},
    };
      
    int c;
    
    c = getopt_long(argc, argv, "v:e:d:p:srl", lopts, NULL);
      
    if (c == -1)
      break;
    
    switch (c) {
    case 'v':
      vel = atoi(optarg);
      break; 
    case 's':
      sincro = true;
      break;
    case 'd':
      dist = atoi(optarg);
      break;
    case 'r':
      rotate = true;
      break;
    case 'e':
      enc = atoi(optarg);
      txt = enc == 3 ? 720 : 360;
    case 'p':
      deg_step = atoi(optarg);
      break;
    case 'l':
      left = true;
      break;
    default:
      print_usage(argv[0]);
      break;
    }
  }
}


bool doInit () {
  
  lego_init(5,10);

  if((mtl = mt_new((enc == 3 || enc == 1) ? NULL : ECNULL, (enc == 3 || enc == 2) ? NULL : ECNULL, LMT_PORT)) == NULL) {
    printf("Error initializing left motor.\n");
    return false;
  }
  
  if((mtr = mt_new((enc == 3 || enc == 1) ? NULL : ECNULL, (enc == 3 || enc == 2) ? NULL : ECNULL , RMT_PORT)) == NULL) {
    printf("Error initializing right motor.\n");
    return false;
  }
    
  if(!ag_new(&push, PSH_PORT, PUSH)) {
    printf("Error initializing push sensor.\n");
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

void move_all (dir d, int vel) {
  
  if (!sincro){
    mt_move(mtl, d, vel);
    mt_move(mtr, d, vel);
  } else 
    mt_move_sinc(d, vel);

}


void turn_degrees (int vel, int degrees){

  double cmtogo = rotate ? (2 * M_PI * ((double)MOTOR_AXIS/2)) : (2 * M_PI * (double)MOTOR_AXIS);

  int limit = (int)( ( ( cmtogo / (2*M_PI*WHEEL_RADIUS) ) * txt) / (360/degrees) ) , first;

  if (!rotate){
    mt_move_t (left ? mtr : mtl, limit, MY_FWD, vel, 0);
    mt_wait(left ? mtr : mtl);
    mt_wait_for_stop(left ? mtr : mtl, 1);
  } else {
    mt_move_t (left ? mtr : mtl, limit, MY_FWD, vel, 0);
    mt_move_t (left ? mtl : mtr, limit, MY_BWD, vel, 0);
    first = mt_wait_all();
    mt_wait_for_stop(first == LMT_PORT ? mtr : mtl, 1);
  }
}


void move_but_think_stupid_robot (int vel) {
  
  uint8_t dst = 255;
  
  move_all(MY_FWD, vel);
  
  do{
    dg_us_get_dist(&us, &dst, 0);
    UDELAY(50000);
  }
  while (dst > dist);

  stop_all();
  mt_wait_for_stop(mtr, 1);

} 

void look_for_another_path_nasty_machine (bool rotate, int vel) {


  bool okgo = false;
  uint8_t dst = 0;
  int degr = INI_DEG;

  while (!okgo){
    
    turn_degrees(vel, degr);
    
    dg_us_get_dist(&us, &dst, 0);
    
    if(dst > dist){
      okgo = true;
      if (degr >= 180)
	left = !left;
    } else
      degr += deg_step;
        
  }
}


int main (int argc, char * argv[]) {


  parse_opts(argc, argv);

  if(!doInit()) {
    
    printf("Error Initializing Stuff.\n");
    exit(EXIT_FAILURE);

    
  } else { //aki fiesta

    double pval;

    while (!ag_psh_is_pushed(&push, &pval));
    UDELAY(750000);

    while (!ag_psh_is_pushed(&push, &pval)) {
      move_but_think_stupid_robot(vel);
      UDELAY(750000);
      look_for_another_path_nasty_machine(rotate, vel);
      UDELAY(750000);
    }

    stop_all();
    lego_shutdown();
    exit(EXIT_SUCCESS);
  }

} 

