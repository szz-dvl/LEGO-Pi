#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "../lego_i2c.h"

#define DVC_ADDR     0x02 //de moment vale pa toos
#define FREQ         9600 //baudios/Hz
#define SDA_1          27
#define SCL_1          17
#define SDA_2          14
#define SCL_2          15


#define CMD_REG        0x41
#define COL_BASE_DATA  3
#define COL_REGS       14
#define COL_MAX_LEN    8
#define IRS_BASE_DATA  3
#define IRS_REGS       16
#define IRS_MAX_LEN    8
#define US_BASE_DIST   12
#define US_MAX_LEN     7
#define US_REGS        20
#define AC_BASE_DATA   3
#define AC_REGS        9
#define AC_MAX_LEN     8
#define COM_BASE_DATA  3
#define COM_REGS       8
#define COM_MAX_LEN    8

#define RAW_LEN        2
#define DWAIT          4000
#define IWAIT          3000
#define CWAIT          19000

/*typedef enum { //a pendre pel cacas

  US_PROD_VERS = 0x00,
  US_PROD_ID = 0x08,
  US_SENSOR_TYPE = 0x10,
  US_FACTORY_ZERO = 0x11,
  US_FACTORY_SCALE_FACTOR = 0x12,
  US_FACTORY_SCALE_DIV = 0x13,
  US_MEASUREMENT_UNITS = 0x14,
  US_MEASUREMENT_INTERV = 0x40,
  US_CMD_STATE = 0x41,
  US_DIST_0 = 0x42,
  US_DIST_1 = 0x43,
  US_DIST_2 = 0x44,
  US_DIST_3 = 0x45,
  US_DIST_4 = 0x46,
  US_DIST_5 = 0x47,
  US_DIST_6 = 0x48,
  US_DIST_7 = 0x49

  } rdRegs;
*/
enum cmdSetters {
 
  US_SET_MEASUREMENT_INTERV = 0x40,
  US_SET_ACT_ZERO = 0x50,
  US_SET_ACT_SC_FACTOR = 0x51,
  US_SET_ACT_SC_DIV = 0x52

};

struct sensor_setter_cmd {

  uint8_t reg;
  int delay;

};
typedef struct sensor_setter_cmd SCMD;

struct sensor_command {

  uint8_t reg;
  uint8_t val;
  int delay;

};
typedef struct sensor_command CMD;

struct sensor_msg {
  
  uint8_t base;
  int len;
  int delay;
  char * def;

};
typedef struct sensor_msg MSG;

SCMD scmds [] = {{US_SET_MEASUREMENT_INTERV, CWAIT}, {US_SET_ACT_ZERO, CWAIT}, {US_SET_ACT_SC_FACTOR, CWAIT}, {US_SET_ACT_SC_DIV, CWAIT}};

              //US off cmd             US single shot         US continuous mes. (def)   US even capture cmd    US warm reset           COL calibrate white   COM calibrate
CMD cmds [] = {{CMD_REG, 0x00, CWAIT},{CMD_REG, 0x01, CWAIT},{CMD_REG, 0x02, CWAIT},    {CMD_REG, 0x03, CWAIT},{CMD_REG, 0x04, CWAIT}, {CMD_REG, 0x43, 0},   {CMD_REG, 0x43, 0},      //COM Measurement mode
 {CMD_REG, 0x00, 0}};

MSG cregs [] = {{0x01, 7, 0,"Version number: "}, {0x08, 8, 0,"Manufacturer: "}, {0x10, 8, 0,"Sensor Type: "}, {0x42, 1, 0,"Color number: "}, {0x43, 1, 0,"Red reading: "}, {0x44, 1, 0,"Green reading: "}, {0x45, 1, 0,"Blue reading: "},  {0x46, RAW_LEN, 0,"Raw Red reading: "}, {0x48, RAW_LEN, 0,"Raw Green reading: "}, {0x4a, RAW_LEN, 0, "Raw Blue reading: "}, {0x4c, 1, 0,"Color Index: "}, {0x4d, 1, 0,"Normalized Red: "}, {0x4e, 1, 0,"Normalized Green: "},  {0x4f, 1, 0,"Normalized Blue: "}};

MSG usregs [] = {{0x00, 5, IWAIT, "Product Version: "}, {0x08, 5, IWAIT, "Product ID: "}, {0x10, 6, IWAIT, "Sensor Type: "}, {0x11, 1, IWAIT,"Factory Zero: "}, {0x12, 1, IWAIT,"Factory Scale Factor: "}, {0x13, 1, IWAIT,"Factory Scale Divisor: "}, {0x14, 7, IWAIT, "Measurement units: "},  {0x40, 1, IWAIT, "Measurement interval: "}, {0x41, 1, IWAIT, "Command state: "}, {0x50, 1, IWAIT, "Actual Zero: "},{0x51, 1, IWAIT, "Actual Scale Factor: "},{0x52, 1, IWAIT, "Actual Scale Divisor: "},{0x42, 1, DWAIT,"Distance 0: "}, {0x43, 1, DWAIT, "Distance 1: "},{0x44, 1, DWAIT, "Distance 2: "},{0x45, 1, DWAIT,"Distance 3: "},{0x46, 1, DWAIT,"Distance 4: "},{0x47, 1, DWAIT,"Distance 5: "},{0x48, 1, DWAIT, "Distance 6: "},{0x49, 1, DWAIT, "Distance 7: "}};

MSG irsregs[] = {{0x01, 7, 0,"Version number: "}, {0x08, 8, 0,"Manufacturer: "}, {0x10, 8, 0,"Sensor Type: "}, {0x42, 1, 0,"DC Signal Direction: "}, {0x43, 1, 0,"DC Strength 1 / Direction 1: "}, {0x44, 1, 0,"DC Strength 2 / Direction 3: "}, {0x45, 1, 0,"DC Strength 3 / Direction 5: "}, {0x46, 1, 0,"DC Strength 4 / Direction 7: "}, {0x47, 1, 0,"DC Strength 5 / Direction 9: "},{0x48, 1, 0,"DC mean: "}, {0x49, 1, 0,"AC Direction : "}, {0x4a, 1, 0,"AC Strength 1: "}, {0x4b, 1, 0,"AC Strength 2: "}, {0x4c, 1, 0,"AC Strength 3: "}, {0x4d, 1, 0,"AC Strength 4: "}, {0x4e, 1, 0,"AC Strength 5: "}};

MSG acregs[] = {{0x01, 7, 0,"Version number: "}, {0x08, 8, 0,"Manufacturer: "}, {0x10, 8, 0,"Sensor Type: "}, {0x42, 1, 0,"X_Upper: "}, {0x43, 1, 0,"Y_Upper: "}, {0x44, 1, 0,"Z_Upper: "}, {0x45, 1, 0,"X_Lower: "}, {0x46, 1, 0,"Y_Lower: "}, {0x47, 1, 0,"Z_Lower: "}};

MSG comregs[] = {{0x01, 7, 0,"Version number: "}, {0x08, 8, 0,"Manufacturer: "}, {0x10, 8, 0,"Sensor Type: "}, {0x41, 1, 0,"Command state: "},{0x42, 1, 0,"Two degree heading: "}, {0x43, 1, 0,"One ddegree adder: "}, {0x44, 1, 0,"Heading Low: "}, {0x45, 1, 0,"Heading High: "}};

int main (int argc, char * argv[]){
  
  int tst = argc < 2 ? 1 : atoi(argv[1]);

  i2c_init(LOG_PRINT);

  switch(tst){
  case 0: //Explore regs in a given range for a device attached to a given port 
    {
      int port = argc < 3 ? 1 : atoi(argv[2]);
      uint8_t range_ini = argc < 4 ? 0x00 : (uint8_t)atoi(argv[3]);
      uint8_t range_fi = argc < 5 ? 0xff : (uint8_t)atoi(argv[4]);
      int retries = argc < 6 ? 10 : atoi(argv[5]), retry = 0;

      int sda = port == 1 ? SDA_1 : SDA_2;
      int scl = port == 1 ? SCL_1 : SCL_2;
      
      uint8_t data_out[] = {range_ini};
      uint16_t data_in[1];
      bool ret;

      I2C_DVC dvc;
      
      i2c_new_device(&dvc, DVC_ADDR, FREQ, sda, scl, GPPUD_PULLDOWN, GPPUD_PULLUP);

      i2c_set_loglvl(LOG_QUIET);
      
      while(data_out[0] < range_fi) {
	  if (!(ret = i2c_transfer(&dvc, data_out , 1, true, data_in, 1, false, 10000))){
	    printf("NACK sending: 0x%02x\n", data_out[0]);
	    retry ++;
	    sleep(1);
	  }
	  if (retry > retries) {
	    if(data_out[0] <= range_fi-1)
	      printf("switching to REG: 0x%02x\n", data_out[0] +1);
	    data_out[0] += 1; 
	    retry = 0;
	  } else if (ret) {
	    printf("sended: 0x%02x, received: 0x%04x / %u / %c in %d retries\n", data_out[0], data_in[0], data_in[0], data_in[0], retry);
	    data_out[0] += 1; 
	    retry = 0;
	  }
      }
      i2c_shutdown(&dvc);
    }
    break;;
  case 1: //Send command to a device plugged into a given port
    {
      
      int port = argc < 3 ? 1 : atoi(argv[2]);
      int index = argc < 4 ? 2 : atoi(argv[3]);
            
      uint8_t cmd, reg;
      int delay;

      int sda = port == 1 ? SDA_1 : SDA_2;
      int scl = port == 1 ? SCL_1 : SCL_2;
      
      I2C_DVC dvc;
           
      if(argc < 5){
        reg = cmds[index].reg;
	cmd = cmds[index].val;
	delay = cmds[index].delay;
      } else {
	reg = scmds[index].reg;
	cmd = (uint8_t)atoi(argv[4]);
	delay = scmds[index].delay;
      }

      uint8_t data_aux[] = {reg,cmd};

      i2c_new_device(&dvc, DVC_ADDR, FREQ, sda, scl, GPPUD_PULLDOWN, GPPUD_PULLUP);

      i2c_write(&dvc, data_aux, 2, delay);
      
      i2c_shutdown(&dvc);

    }
    break;;
  case 2: //Ultrasonic stuff
    {

      int port = argc < 3 ? 1 : atoi(argv[2]);
      int i, k;
      uint8_t cmd_ss[2];
      uint16_t data_in [] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00};
      uint8_t data_aux[1];
      
      int sda = port == 1 ? SDA_1 : SDA_2;
      int scl = port == 1 ? SCL_1 : SCL_2;
            
      I2C_DVC us;

      i2c_new_device(&us, DVC_ADDR, FREQ, sda, scl, GPPUD_PULLDOWN, GPPUD_PULLUP);
      
      for (i=0; i<US_REGS; i++){ 
	data_aux[0] = usregs[i].base;
	if (i < US_BASE_DIST)
	  i2c_transfer(&us, data_aux, 1, true, data_in, usregs[i].len, false, usregs[i].delay);
	else {
	  cmd_ss [0] = cmds[1].reg;
	  cmd_ss [1] = cmds[1].val;
	  i2c_write(&us, cmd_ss, 2, cmds[1].delay);
	  i2c_transfer(&us, data_aux, 1, true, data_in, usregs[i].len, false, usregs[i].delay);
	}
	if(i == 0 || i == 1 || i == 2 || i == 6 ){
	  printf("%s", usregs[i].def);
	  for (k=0; k<usregs[i].len; k++)
	    printf("%c",data_in[k]);
	  printf("\n");
	} else if (i < US_BASE_DIST)
	  printf("%s0x%02x\n", usregs[i].def, data_in[0]);
	else
	  printf("%s%u\n", usregs[i].def, data_in[0]);

	//sleep(1);
      }
      i2c_shutdown(&us);
    }
    break;;
  case 3://color sensor HiTechnic
    {
      int port = argc < 3 ? 2 : atoi(argv[2]);
      uint8_t data_aux [1];
      uint16_t data_in [1];
      char str_aux [COL_MAX_LEN+1];
      
      int sda = port == 1 ? SDA_1 : SDA_2;
      int scl = port == 1 ? SCL_1 : SCL_2;
      
      I2C_DVC col;

      i2c_new_device(&col, DVC_ADDR, FREQ, sda, scl, GPPUD_PULLUP, GPPUD_PULLUP);	
      
      int i, k;
      uint16_t auxraw = 0;
      
      for (i=0; i < COL_REGS; i++){
	for (k=0; k < cregs[i].len; k++){
	  data_aux[0] = cregs[i].base + k;
	  
	  if(i2c_transfer(&col, data_aux, 1, false, data_in, 1, false, cregs[i].delay)) {
	    
	    if(i < COL_BASE_DATA)
	      str_aux[k] = (char) data_in[0];
	    
	    else if(i>6 && i<10) {
	      if((cregs[i].base + k)%2 == 0)
		auxraw = (data_in[0] & 0xff);
	      else {
		auxraw <<= BYTE_LEN;
		auxraw |= (data_in[0] & 0xff);
	      }
	      //printf("INDEX: %d,AUXRAW_0x%02x: 0x%04x, data_in: 0x%04x\n",i, cregs[i].base + k, auxraw, data_in[0]);
	    }
	    
	    //sleep(1);
	  } else 
	    printf("NAK sending >> %s0x%02x\n", cregs[i].def, cregs[i].base + k);  
	  
	}
	if(i < COL_BASE_DATA) {
	  str_aux[k] = '\0';
	  printf("%s%s\n", cregs[i].def, str_aux);
	} else if(i>6 && i<10) 
	  printf("%s0x%04x / %u\n", cregs[i].def, auxraw, auxraw);
	else if(i == 10){
	  printf("%s0x%02x / ", cregs[i].def, data_in[0]);
	  for (k=BYTE_LEN-3; k >= 0; k--) {
	    if (data_in[0] & (0x01 << k))
	      printf("1");
	    else 
	      printf("0");
	  }
	  printf("\n");
	}else
	  printf("%s0x%02x / %u\n", cregs[i].def, data_in[0], data_in[0]);
      }
      
    
    i2c_shutdown(&col);
  }
  break;;  
 case 4://IR seeker HiTechnic
   {
     int port = argc < 3 ? 2 : atoi(argv[2]);
     uint8_t data_aux [1];
     uint16_t data_in [1];
     char str_aux [IRS_MAX_LEN+1];
     
     int sda = port == 1 ? SDA_1 : SDA_2;
     int scl = port == 1 ? SCL_1 : SCL_2;
     
     I2C_DVC irs;
     
     i2c_new_device(&irs, DVC_ADDR, FREQ, sda, scl, GPPUD_PULLUP, GPPUD_PULLUP);	
     
     int i, k;

     for (i=0; i < IRS_REGS; i++){
	for (k=0; k < irsregs[i].len; k++){
	  data_aux[0] = irsregs[i].base + k;
	  
	  if(i2c_transfer(&irs, data_aux, 1, false, data_in, 1, false, irsregs[i].delay)) {
	    
	    if(i < IRS_BASE_DATA)
	      str_aux[k] = (char) data_in[0];
	        
	  } else 
	    printf("NAK sending >> %s0x%02x\n", irsregs[i].def, irsregs[i].base + k);  
	  
	}// la k
	if(i < IRS_BASE_DATA) {
	  str_aux[k] = '\0';
	  printf("%s%s\n", irsregs[i].def, str_aux);
	} else
	  printf("%s0x%02x / %u\n", irsregs[i].def, data_in[0], data_in[0]);
	
     }//la i
     i2c_shutdown(&irs);
   }
   break;; 
  case 5: //Hitechnic accelerometer
    {
      int port = argc < 3 ? 2 : atoi(argv[2]);
      uint8_t data_aux [1];
      uint16_t data_in [1];
      char str_aux [AC_MAX_LEN+1];
      
      int sda = port == 1 ? SDA_1 : SDA_2;
      int scl = port == 1 ? SCL_1 : SCL_2;
      
      int x = 0, y = 0 ,z = 0;
      //uint16_t x_aux = 0, y_aux = 0, z_aux = 0;
 
      I2C_DVC acc;
      
      i2c_new_device(&acc, DVC_ADDR, FREQ, sda, scl, GPPUD_PULLUP, GPPUD_PULLUP);	
      
      int i, k;
      
      while (1){
      
      for (i=0; i < AC_REGS; i++){
	for (k=0; k < acregs[i].len; k++){
	  data_aux[0] = acregs[i].base + k;
	  
	  if(i2c_transfer(&acc, data_aux, 1, false, data_in, 1, false, acregs[i].delay)) {
	    
	    if(i < AC_BASE_DATA)
	      str_aux[k] = (char) data_in[0];
	    else {
	      switch(acregs[i].base + k){
	      case 0x42:
		{
		  x = data_in[0] & 0x80 ? (data_in[0] - 256) << 2 : data_in[0] << 2;
		  //printf("Xupper: %d, data: %u\n", x, data_in[0]);
		}
		break;;
	      case 0x43:
		{
		  y = data_in[0] & 0x80 ? (data_in[0] - 256) << 2 : data_in[0] << 2;
		  //printf("Yupper: %d, data: %u\n", y, data_in[0]);
		}
		break;;
	      case 0x44:
		{
		  z = data_in[0] & 0x80 ? (data_in[0] - 256) << 2 : data_in[0] << 2;
		  //printf("Zupper: %d, data: %u\n", z, data_in[0]);
		}
		break;;
	      case 0x45:
		{
		  x |= data_in[0] & 0x03;
		  //printf("X all: %d, data: %u\n", x, data_in[0]);
		}
		break;;
	      case 0x46:
		{
		  y |= data_in[0] & 0x03;
		  //printf("Y all: %d, data: %u\n", y, data_in[0]);
		}
		break;;
	      case 0x47:
		{
		  z |= data_in[0] & 0x03;
		  //printf("X all: %d, data: %u\n", z, data_in[0]);
		}
		break;;
	      default:
		break;;
	      }

	    }
	  } else 
	    printf("NAK sending >> %s0x%02x\n", acregs[i].def, acregs[i].base + k);  
	  
	}// la k
	if(i < AC_BASE_DATA) {
	  str_aux[k] = '\0';
	  printf("%s%s\n", acregs[i].def, str_aux);
	} 	
      }//la i
      
      printf("X: %d\n", x);
      printf("Y: %d\n", y);
      printf("Z: %d\n", z);
      sleep(1);
      
      }//while
      i2c_shutdown(&acc);
    }
    break;; 
  case 6://Compass sensor HiTechnic
    {
      int port = argc < 3 ? 2 : atoi(argv[2]);
      uint8_t data_aux [1];
      uint16_t data_in [1];
      char str_aux [COM_MAX_LEN+1];
      
      int sda = port == 1 ? SDA_1 : SDA_2;
      int scl = port == 1 ? SCL_1 : SCL_2;
      
      int dsigned = 0;
      uint16_t dunsigned = 0;

      I2C_DVC com;
     
      i2c_new_device(&com, DVC_ADDR, FREQ, sda, scl, GPPUD_PULLUP, GPPUD_PULLUP);	
      
      int i, k;
     
      for (i=0; i < COM_REGS; i++){
	for (k=0; k < comregs[i].len; k++){
	  data_aux[0] = comregs[i].base + k;
	  
	  if(i2c_transfer(&com, data_aux, 1, false, data_in, 1, false, comregs[i].delay)) {
	    
	    if(i < COM_BASE_DATA)
	      str_aux[k] = (char) data_in[0];
	    else {
	      switch(comregs[i].base + k){
	      case 0x42:
		{
		  dsigned = (int) data_in[0]*2;//(data_in[0]*2) > 127 ? (data_in[0] - 256) << 1 : data_in[0] << 1;
		  //printf("dsigned 0x42: %d, data: %u\n", dsigned, data_in[0]);
		}
		break;;
	      case 0x43:
		{
		  dsigned += data_in[0];// & 0x80 ? (data_in[0] - 256) << 2 : data_in[0] << 2;
		  //printf("dsigned 0x43: %d, data: %u\n", dsigned, data_in[0]);
		}
		break;;
	      case 0x44:
		{
		  dunsigned = data_in[0]; //& 0x80 ? (data_in[0] - 256) << 2 : data_in[0] << 2;
		  //printf("dunsigned 0x44: %u,  data: %u\n", dunsigned, data_in[0]);
		}
		break;;
	      case 0x45:
		{
		  dunsigned |= data_in[0] << BYTE_LEN;
		  //printf("dunsigned 0x45: %u,  data: %u\n", dunsigned, data_in[0]);
		}
		break;;
	      default:
		break;;
	      }
	    }
	    
	  } else 
	    printf("NAK sending >> %s0x%02x\n", comregs[i].def, comregs[i].base + k);  
	  
	}// la k
	if(i < COM_BASE_DATA){
	  str_aux[k] = '\0';
	  printf("%s%s\n", comregs[i].def, str_aux);
	} else if(comregs[i].base == 0x41)
	  printf("%s0x%02x / %u\n", comregs[i].def, data_in[0], data_in[0]);
	
      }//la i
      
      printf("Degree signed: %d\n", dsigned);
      printf("Degree unsigned: %u\n", dunsigned);
      
      i2c_shutdown(&com);
    }
    break;;
  case 7: //raspidillu
    {
      I2C_DVC col;
      
      uint8_t data_aux[] = {0x43};
      uint16_t data_in[3];
 
      i2c_new_device(&col, DVC_ADDR, FREQ, SDA_2, SCL_2, GPPUD_PULLUP, GPPUD_PULLUP);
      
      if(i2c_transfer(&col, data_aux, 1, false, data_in, 3, false, 0))
      
	printf("Received: 0x%02x / %u, 0x%02x / %u, 0x%02x / %u \n", data_in[0], data_in[0], data_in[1], data_in[1], data_in[2], data_in[2]);
      
      else 

	printf("NACK!\n");

      i2c_shutdown(&col);

    }
    break;;
  default:
    break;;
  }
  
  return (EXIT_SUCCESS);
  
}
