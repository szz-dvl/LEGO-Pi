#include "lego_shared.h"
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define CS	 	0
#define SPI_CLK		1000000
#define LEN		4
#define MAX_VAL		4095  //10 bits de resolució (tot-hi que eren 12...sospito de INL...)
#define VREF		5     //Voltatge de referencia
#define MAX_PORT        3
#define MIN_PORT        0
#define MAX_DB          100
#define HT_GYRO_DEF     2378

typedef enum {

  PUSH,
  LIGHT,
  SOUND,
  HT_GYRO,
  AG_OTHER

} agType;


enum lports { L_PORT0 = 3, L_PORT1 = 28, L_PORT2 = 2, L_PORT3 = 29 }; //al final 30 -> 28 / 31 -> 29

struct analog_device {
   
  agType type;
  int port;

};

typedef struct analog_device ANDVC;


extern bool   ag_init(int times);
extern bool   ag_set_verbose(int lvl);
extern bool   ag_new (ANDVC* dvc, int port, agType type);
extern bool   ag_lgt_set_led (ANDVC* dvc, bool on);
extern int    ag_lgt_get_ledstate (ANDVC* dvc);
extern bool   ag_psh_is_pushed (ANDVC * dvc, double * volt);
extern int    ag_snd_get_db (ANDVC * dvc);
extern bool   ag_gyro_cal (ANDVC * dvc);
extern int    ag_gyro_get_val (ANDVC * dvc, bool * error);
extern double ag_read_volt (ANDVC * dvc);
extern bool   ag_oth_set_y (ANDVC* dvc, bool high);
extern int    ag_read_int (ANDVC * dvc);
extern void   ag_shutdown ();
