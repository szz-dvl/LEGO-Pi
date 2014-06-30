#include <lego.h>
#include <math.h>
#include <getopt.h>

#define RMT_PORT  1
#define LMT_PORT  0
#define MY_FWD    BWD
#define MY_BWD    FWD 
#define PSH_PORT  2

#define UDELAY(t) DELAY_US(t)


#define WHEEL_RADIUS 2.5 //cm
#define MOTOR_AXIS   18  //cm

MOTOR * mtr, * mtl;
ANDVC push;

static int dist = 100;
static int vel = 90;
static double posCtrl = 0;
static bool sinc = false;
static int enc = 3;
static int txt = 720;
static bool rotate = false; 

static void print_usage(const char *prog)
{
	printf("Usage: %s -t<test_num> [-pid]\n", prog);
	puts("  -d --dist      distance for each side of the square [cm]\n"
	     "  -v --vel       velocity [0-200]\n"
	     "  -p --posctrl   position control [0-1]\n"
	     "  -s --sinc      sincro \"on\"\n"
	     "  -e --enc       encoder lines active per motor [1,2] or <3> both\n"
	     "  -r --rotate    rotate robot = true, turn robot = false\n");

	exit(EXIT_FAILURE);
}


   
static void parse_opts(int argc, char *argv[])
{
  while (1) {
    static const struct option lopts[] = {
      { "dist",    1, 0, 'd'},
      { "vel",     1, 0, 'v'},
      { "posctrl", 1, 0, 'p'},
      { "sinc",    0, 0, 's'},
      { "rotate",  0, 0, 'r'},
      { "enc",     1, 0, 'e'},
      {0, 0, 0, 0},
    };
      
    int c;
    
    c = getopt_long(argc, argv, "d:v:p:e:sr", lopts, NULL);
      
    if (c == -1)
      break;
    
    switch (c) {
    case 'v':
      vel = atoi(optarg);
      break; 
    case 'd':
      dist = atoi(optarg);
      break;
    case 'p':
      posCtrl = atof(optarg);
      break;
    case 's':
      sinc = true;
      break;
    case 'r':
      rotate = true;
      break;
    case 'e':
      enc = atoi(optarg);
      txt = enc == 3 ? 720 : 360;
      break;
    default:
      print_usage(argv[0]);
      break;
    }
  }
}



bool doInit () {
  
  lego_init(1,1); //parameters will be ignored
  
  
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

    return true;
}

void stop_all () {
  
  mt_stop(mtl, true);
  mt_stop(mtr, true);

}

void move_all_t (dir d, int vel, int ticks, double posCtrl) {
  
  mt_move_t (mtl, ticks, d, vel, posCtrl);
  mt_move_t (mtr, ticks, d, vel, posCtrl);
  
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

void turn_degrees (int vel, int degrees){

  double cmtogo = rotate ? (2 * M_PI * ((double)MOTOR_AXIS/2)) : (2 * M_PI * (double)MOTOR_AXIS);

  int limit = (int)( ( ( cmtogo / (2*M_PI*WHEEL_RADIUS) ) * txt) / (360/degrees) ) , first;

  //printf("turn ticks limit = %d, trayectoria = %.2f\n", limit, (cmtogo/(360/degrees)) );
 
  if (!rotate){
    mt_move_t (mtr, limit, MY_FWD, vel, 0);
    mt_wait(mtr);
    mt_wait_for_stop(mtr, 2);
  } else {
    mt_move_t (mtr, limit, MY_FWD, vel, 0);
    mt_move_t (mtl, limit, MY_BWD, vel, 0);
    first = mt_wait_all();
    mt_wait_for_stop(first == LMT_PORT ? mtr : mtl, 2);
  }
}


int main (int argc, char * argv[]) {

  parse_opts(argc,argv);
  
  //printf("posCtrl is: %f\n", posCtrl);
  
  double turns, pval;
  int side, ticks, first;

  if(!doInit()) {
    
    printf("Error Initializing Stuff.\n");
    exit(EXIT_FAILURE);

  } else {    //cuadrado...

    while(!ag_psh_is_pushed(&push, &pval));

    UDELAY(750000);
    
    //compute turns for the desired distance, 
    turns = dist/(2*M_PI*WHEEL_RADIUS);
    
    //ticks for the desired turns
    ticks = turns * txt;
    //printf("ticks to go: %d, turns = %.2f, dist x turn = %.5f\n", ticks, turns, 2*M_PI*WHEEL_RADIUS);
    
    //draw square.
    for( side = 0; side < 4; side++ ){
      if(sinc)
	mt_move_sinc_t(MY_FWD, vel, ticks, posCtrl);
      else
	move_all_t(MY_FWD, vel, ticks, posCtrl);
    
      first = mt_wait_all();
      stop_all();
      mt_wait_for_stop(first == LMT_PORT ? mtr : mtl, 2);

      turn_degrees(vel, 90);
    }
    
    
  }
  
  lego_shutdown();
  exit(EXIT_SUCCESS);
  
} 

