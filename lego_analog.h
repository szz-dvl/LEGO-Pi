#include "lego_shared.h"
#include <wiringPiSPI.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define CS	 	0
#define SPI_CLK		1500000
#define TO_READ		0x08
#define LEN		3
#define MAX_VAL		1023  //10 bits de resolució (tot-hi que eren 12...sospito de INL...)
#define VREF		5     //Voltatge de referencia
#define PUSH            1
#define LIGHT           2
#define MAX_PORT        3
#define MIN_PORT        0


enum lports { L_PORT0 = 15, L_PORT1 = 30, L_PORT2 = 14, L_PORT3 = 31 };

extern int an_fd;

struct analog_device {
   
  int type;
  int port;
  int lpin;

};

typedef struct analog_device ANDVC;

extern void analog_setup(void);
extern int new_analog (ANDVC*, int, int);
extern int analog_light_on (ANDVC*);
extern int analog_light_off (ANDVC*);
extern double analog_read_voltage (ANDVC *);
extern int analog_read_int (ANDVC *);
extern bool analog_pushed (ANDVC *);
extern void analog_shutdown (void);
