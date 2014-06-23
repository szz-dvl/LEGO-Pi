#include <lego.h>
#include <string.h>
#include <math.h>
#include <getopt.h>
#include <gsl/gsl_sort.h>


#define LR_PORT   1
#define LL_PORT   3
#define PSH_PORT  2
#define RMT_PORT  1
#define LMT_PORT  0
#define MY_FWD    BWD
#define MY_BWD    FWD 

#define UDELAY(t) DELAY_US(t)


#define WMAX 1750

#define IS_WHITE(x)   (x <= WMAX) //x >= WMIN &&
#define IS_BLACK(x)   (!IS_WHITE(x)) //(x >= BMIN && x <= BMAX)

#define TDELAY   90000

MOTOR * mtr, * mtl;
ANDVC lright, lleft, push;

static bool mode = false; //follow line by default 
static int calib = 1000;  //1000 samples per calibration
static bool white = true; //calibrate whitestatic bool log_dbg = false;
static int vel = 90;

static void print_usage(const char *prog)
{
	printf("Usage: %s -t<test_num> [-pid]\n", prog);
	puts("  -m --mode      true = follow line, false = calibrate\n"
	     "  -s --samples   samples per calibration\n"
	     "  -c --color     color to calibrate black = true, white = false\n"
	     "  -v --vel       velocity, def = 90\n");
	exit(EXIT_FAILURE);
}


   
static void parse_opts(int argc, char *argv[])
{
  while (1) {
    static const struct option lopts[] = {
      { "samples",    1, 0, 's'},
      { "mode" ,      0, 0, 'm'},
      { "color",      0, 0, 'c'},
      { "vel",        1, 0, 'v'},
      {0, 0, 0, 0},
    };
      
    int c;
    
    c = getopt_long(argc, argv, "s:v:mc", lopts, NULL);
      
    if (c == -1)
      break;
    
    switch (c) {
    case 'v':
      vel = atoi(optarg);
      break; 
    case 's':
      calib = atoi(optarg);
      break;
    case 'm':
      mode = true;
      break;
    case 'c':
      white = false;
      break;
    default:
      print_usage(argv[0]);
      break;
    }
  }
}



bool doInit () {
  
  lego_init(5,10);
  
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

    if(!ag_new(&lright, LR_PORT, LIGHT)) {
      printf("Error initializing right light sensor.\n");
      return false;
    } 

    if(!ag_new(&lleft, LL_PORT, LIGHT)) {
      printf("Error initializing left light sensor.\n");
      return false;
    } 
      
    return true;
}

void stop_all () {
  
  mt_stop(mtl, true);
  mt_stop(mtr, true);

}

void move_all (dir d, int vel) {
  
  mt_move(mtl, d, vel);
  mt_move(mtr, d, vel);
  
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



int main (int argc, char * argv[]) {

  parse_opts(argc,argv);

  if(!doInit()) {
    
    printf("Error Initializing Stuff.\n");
    exit(EXIT_FAILURE);

  } else { //o calibrar colores o seguir la linea

    //bool rotate = true;
    double pval;
    bool stop = true;
    int left, right;

    while(!ag_psh_is_pushed(&push, &pval));

    UDELAY(750000);

    ag_lgt_set_led(&lright, true);
    ag_lgt_set_led(&lleft, true);

    if (mode) { //calibrar colores
      int i = 0;
      double acumr [calib], minr, maxr, upr, lowr;
      double acuml [calib], minl, maxl, upl, lowl;

      for (;i<calib;i++) {
	acumr[i] = (double)ag_read_int(&lright);
	acuml[i] = (double)ag_read_int(&lleft);
	UDELAY(100);
      } 
      
      gsl_stats_minmax(&minr, &maxr, acumr, 1, calib);
      gsl_stats_minmax(&minl, &maxl, acuml, 1, calib);
      
      gsl_sort (acumr,1,calib);
      gsl_sort (acuml,1,calib);
      
      upl = gsl_stats_quantile_from_sorted_data (acuml,1,calib,0.95);//uq
      lowl = gsl_stats_quantile_from_sorted_data (acuml,1,calib,0.05);//lq
    
      upr = gsl_stats_quantile_from_sorted_data (acumr,1,calib,0.95);//uq
      lowr = gsl_stats_quantile_from_sorted_data (acumr,1,calib,0.05);//lq
      
      for (i = 0 ; i < 2; i++){
	printf("COLOR: %s, SENSOR: %s\n", white ? "WHITE" : "BLACK", i == 0 ? "LEFT" : "RIGHT");
	printf("min_v: %d, max_v: %d\n", i == 0 ? (int)minl : (int)minr, i == 0 ? (int)maxl : (int)maxr);
	printf("low_q: %d, up_q :%d\n", i == 0 ? (int)lowl : (int)lowr, i == 0 ? (int)upl : (int)upr);
	printf("\n");
	
      }
      
    } else { //seguir linea    
 
      while(!ag_psh_is_pushed(&push, &pval)) {
	
	if (stop){
	  stop = false;
	  move_all(MY_FWD,vel);
	}
	
	right = ag_read_int(&lright);
	left = ag_read_int(&lleft);
	
	if(! IS_WHITE(right) ||  ! IS_WHITE(left)){
	  
	  stop_all();
	  stop = true;
	  
	  if (IS_BLACK(left)){
	    
	    rotate_robot(vel, true);	  
	    while(IS_BLACK(ag_read_int(&lleft)));
	    UDELAY(TDELAY);
	    stop_all();
	    
	  } else if (IS_BLACK(right)) {
	    
	    rotate_robot(vel, false);	  
	    while(IS_BLACK(ag_read_int(&lright)));
	    UDELAY(TDELAY);
	    stop_all();
	  }
	    
	   	  
	}
     
     
      }
    }

  }
  lego_shutdown();
  exit(EXIT_SUCCESS);
  
} 

