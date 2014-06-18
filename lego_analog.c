/*
* This file is part of LEGO-Pi.
*
* Copyright (Copyplease) szz-dvl.
*
*
* License
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Affero General Public License as published
* by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Affero General Public License for more details at
* <http://www.gnu.org/licenses/agpl-3.0-standalone.html>
*/

#include "lego_analog.h"

INIT            status;

static int      an_fd = 0;

static uint8_t  port_busy = 0;

static bool     lpin_state [] = {false, false, false, false};
static int      ypin_port  [] = {L_PORT0, L_PORT1, L_PORT2, L_PORT3};

static int      act_gyro = HT_GYRO_DEF;

static uint8_t  spiMode = 3; 
static uint8_t  spiBPW = 8 ;
static char    *spiDev0 = "/dev/spidev0.0" ;
static char    *spiDev1 = "/dev/spidev0.1" ;
static int      spiAvg = 10 ;
static int      ag_log_lvl = LOG_LVL_ADV ;

static int      SPI_receive (int, uint8_t [], int);
static double   analog_read_voltage (ANDVC * dvc);
static int      analog_read_int (ANDVC * dvc);
static uint16_t res_inv (uint8_t data[]);
static void     not_critical (char *fmt, ...);
static void     debug (char *fmt, ...);

static uint16_t res_inv (uint8_t data[]){

  int i;
  uint16_t res = 0;
  uint8_t * pt = data;
  uint8_t dt;
  
  
    dt = (pt[3] >> 2);
    
    for(i=0; i<6; i++) {
      res |= (dt & 0x01);
      dt >>= 1;
      res *= 2;
    }
    
    dt = pt[2] & 0x3F;
    
    for(i=0; i<5; i++) {
      res |= (dt & 0x01);
      dt >>= 1;
      res *= 2;
    }
    
    return res |= (dt & 0x01) ;
}


static int SPI_receive (int fd, uint8_t resp[], int chann) {

  struct spi_ioc_transfer spi;
  int retval , i;
  uint8_t send [] = {0x00};
  uint16_t res, resinv;
  uint8_t next;

  send[0] |= (1 << 7) ;	                 // start bit		
  send[0] |= (1 << 6) ;                  // !differential, single-ended		
                                         // D2 is a don't care
  send[0] |= ((chann >> 1) & 0x01) << 4 ; 
  send[0] |= ((chann >> 0) & 0x01) << 3 ;
  
  
  //printf("SENDING %d: 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x \n", chann, resp[0], resp[1], resp[2], resp[3], resp[4], resp[5]);

  spi.tx_buf = (unsigned long)send ;
  spi.rx_buf = (unsigned long)resp ;
  spi.len = (__u32) LEN ;
  spi.delay_usecs = 0 ;
  spi.speed_hz = (__u32) SPI_CLK ;
  spi.bits_per_word = spiBPW ;
  spi.cs_change = 0 ;
  spi.pad = 0 ;

  retval = ioctl (fd, SPI_IOC_MESSAGE(1), &spi) ;
  
  
  //response treantemt
  if(retval){
    res = ((resp[0] & 0x01) << 11) | resp[1] << 3 | resp[2] >> 5;
    resinv = res_inv(resp);
    if(res != resinv){          //try to shift the result 1 bit to the right, experiments shows that it happens sometimes.

      next = resp[0] & 0x01;
      for (i=0; i<LEN-1; i++){
	if(next){
	  next = resp[i+1] & 0x01;
	  resp[i+1] = (resp[i+1] >> 1) | 0x80;
	} else {
	  next = resp[i+1] & 0x01;
	  resp[i+1] >>= 1;
	}
      }
      resp[0]>>=1;
      
      res = ((resp[0] & 0x01) << 11) | resp[1] << 3 | resp[2] >> 5;
      resinv = res_inv(resp);
      
      return res == resinv ? res : -2;
    } else
      return res;
  }

  return retval ;
}

static double analog_read_voltage (ANDVC * dvc) { /* READ analog value (float), VOLTATGE */

  uint8_t data [] = { 0x00, 0x00, 0x00, 0x00 } ;
  int retval;
  uint32_t avg = 0;
  int i = 0;
  
  while (i < spiAvg) {
    
    if ((retval = SPI_receive(an_fd, data, dvc->port)) < 0)
      debug("analog_read_voltage: Error comunicating to SPI device, %s\n", retval == -1 ? "ioctl failed." : "incoherent response.");
    else { 
      avg += retval;
      i++;
    }
  }
  
  return ((((double)avg/spiAvg)/MAX_VAL) * VREF);
}




static int analog_read_int (ANDVC * dvc) { /* READ analog value (integer), max avalue = 4095 */

  uint8_t data [] = { 0x00, 0x00, 0x00, 0x00 } ;
  int retval;
  uint32_t avg = 0;
  int i = 0;
  
  while (i < spiAvg) {
    
    if ((retval = SPI_receive(an_fd, data, dvc->port)) < 0)
      debug("analog_read_voltage: Error comunicating to SPI device, %s\n", retval == -1 ? "ioctl failed." : "incoherent response.");
    else { 
      avg += retval;
      i++;
    }
  }
  
  return ((double)avg/spiAvg);
}


static int SPI_init (int cs, int speed){

  int fd ;

  if ((fd = open (cs == 0 ? spiDev0 : spiDev1, O_RDWR)) < 0)
    return FAIL ;

  if (ioctl (fd, SPI_IOC_WR_MODE, &spiMode) < 0) return FAIL ;
  if (ioctl (fd, SPI_IOC_RD_MODE, &spiMode) < 0) return FAIL ;

  if (ioctl (fd, SPI_IOC_WR_BITS_PER_WORD, &spiBPW) < 0) return FAIL ;
  if (ioctl (fd, SPI_IOC_RD_BITS_PER_WORD, &spiBPW) < 0) return FAIL ;

  if (ioctl (fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) return FAIL ;
  if (ioctl (fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) return FAIL ;

  return fd ;
}

extern bool ag_set_verbose(int lvl){

  if(status.ag){
    
    if (lvl >=0 && lvl <= LOG_LVL_DBG){
      ag_log_lvl = lvl;
      return true;
    } else {
      printf("ag_set_verbose: Log level \"%d\" out of bounds\n", lvl);
      return false;
    }
  } else {
    not_critical("ag_set_verbose: Analog interface not initialised.\n");
    return false;
  }
}

extern bool ag_init(int avgc) {

  spiAvg = avgc; 
  int i;

  if(!status.wpi)
    status.wpi = wiringPiSetupGpio() == 0;
  
  for (i = 0; i <= MAX_PORT; i++)
    pinMode(ypin_port[i], OUTPUT);    //set all the pins wired to yellow wire to OUTPUT

  if (!(an_fd = SPI_init(CS, SPI_CLK))) {
    not_critical("analog_setup: Error setting up SPI interface");
    perror("");
    status.ag = false;
  } else
    status.ag = true;
  
  return status.ag;
}

extern bool ag_new ( ANDVC* dvc, int port, agType type ) {

  if(status.ag) {
    
    int ret = true;

    if (port < MIN_PORT || port > MAX_PORT){
      not_critical("ag_new: port must be between %d - %d\n", MIN_PORT, MAX_PORT);
      ret = false;
    } else if (!(port_busy & 1 << port)) {
      dvc->port = port;
      port_busy |= 1 << port;
    } else {
      not_critical("ag_new: Analog port %d busy\n", port);
      ret = false;
    }
    
    if(ret) {
      if( type > AG_OTHER ) {
	not_critical("ag_new: Unrecognized device type\n");
	ret = false;
      } else {
	dvc->type = type;
	if (type == LIGHT)
	  digitalWrite(ypin_port[dvc->port],LOW);
       else if(type == SOUND)
	  digitalWrite(ypin_port[dvc->port], HIGH);
      }
    }
      
    return ret;

  } else {
    not_critical("ag_new: Analog interface not initialised.\n");
    return false;
  }
}

extern bool ag_lgt_set_led (ANDVC* dvc, bool on){
  
  if(status.ag) {
    if (dvc->type != LIGHT) {
      not_critical("ag_lgt_light_on: Device type must be %d (LIGHT)\n", LIGHT);
      return false;
    } else if (on) { 
      if (!lpin_state[dvc->port]) {
	digitalWrite(ypin_port[dvc->port], HIGH); 
	lpin_state[dvc->port] =  true;
      }
    } else {
      if (lpin_state[dvc->port]) {
	digitalWrite(ypin_port[dvc->port], LOW); 
	lpin_state[dvc->port] =  false;
      }
    }
    return true;
  } else {
    not_critical("ag_lgt_light_on: Analog interface not initialised.\n");
    return false;
  }

}

extern bool ag_oth_set_y (ANDVC* dvc, bool high){
  
  if(status.ag) {
    if (dvc->type != AG_OTHER) {
      not_critical("ag_oth_set_y: Device type must be %d (AG_OTHER)\n", AG_OTHER);
      return false;
    } else if (high) { 
      if (!lpin_state[dvc->port]) {
	digitalWrite(ypin_port[dvc->port], HIGH); 
	lpin_state[dvc->port] =  true;
      }
    } else {
      if (lpin_state[dvc->port]) {
	digitalWrite(ypin_port[dvc->port], LOW); 
	lpin_state[dvc->port] =  false;
      }
    }
    return true;
  } else {
    not_critical("ag_oth_set_y: Analog interface not initialised.\n");
    return false;
  }

}

extern int ag_lgt_get_ledstate (ANDVC* dvc){
  
  if(status.ag) {
    if (dvc->type != LIGHT) {
      not_critical("ag_lgt_get_ledstate: Device type must be %d (LIGHT)\n", LIGHT);
      return FAIL;
    } else
      return lpin_state[dvc->port];
  } else {
    not_critical("ag_lgt_get_ledstate: Analog interface not initialised.\n");
    return FAIL;
  }

}

extern bool ag_psh_is_pushed (ANDVC * dvc, double * volt) {

  if(status.ag) {
    if (dvc->type != PUSH) {
      not_critical("ag_psh_is_pushed: Device type must be %d (PUSH)\n", PUSH);
      *volt = -1;
      return false;
    }
    else {  
      *volt = analog_read_voltage(dvc);
      return *volt < (double)(VREF/2) ;
    }
  } else {
    not_critical("ag_psh_is_pushed: Analog interface not initialised.\n");
    return false;
  }

}

extern int ag_snd_get_db (ANDVC * dvc) { 

  if(status.ag) {
    
    if(dvc->type == SOUND) {
      
      int val = analog_read_int(dvc);
	
      return( ((double)(MAX_VAL-val)/MAX_VAL) * MAX_DB );

    } else {

      not_critical("ag_snd_get_db: Device type must be %d (SOUND)\n", SOUND);
      return FAIL;

    }  

  } else {
    
    not_critical("ag_new: Analog interface not initialised.\n");
    return FAIL;
    
  }

}

extern int ag_gyro_get_val (ANDVC * dvc, bool * error) { //Positives values for clockwise rotation, negatives for anticlockwise, the higher the value returned is the faster we turn. 

  *error = false; 
  if(status.ag) {
    
    if(dvc->type == HT_GYRO) {

      int val = analog_read_int(dvc);
      return val - act_gyro;

    } else {

      not_critical("ag_gyro_get_val: Device type must be %d (HT_GYRO)\n", HT_GYRO);
      *error = true;
      return FAIL;

    }  

  } else {
    
    not_critical("ag_gyro_get_val: Analog interface not initialised.\n");
    *error = true;
    return FAIL;
    
  }

}

extern bool ag_gyro_cal (ANDVC * dvc, int times) { //Make sure the robot is stationary before call this one!

  if(status.ag) {
    
    if(dvc->type == HT_GYRO) {
      
      int acum = 0, val;
      int tback = times == 0 ? 5 : times;
      int i = tback;
      
      while (i > 0) {
	
	val = analog_read_int(dvc);
	acum += val;
	i--;
	val = 0;
	DELAY_US(50);
      
      }
      
      act_gyro = (acum / tback);

      return true;

    } else {

      not_critical("ag_gyro_get_val: Device type must be %d (HT_GYRO)\n", HT_GYRO);
      return false;

    }  

  } else {
    
    not_critical("ag_gyro_get_val: Analog interface not initialised.\n");
    return false;
    
  }

}

extern int ag_read_int (ANDVC * dvc){
  if(status.ag) 
    return (analog_read_int(dvc));
  else {
    not_critical("ag_new: Analog interface not initialised.\n");
    return FAIL;
  }
}

extern double ag_read_volt (ANDVC * dvc){
  if (status.ag) 
    return (analog_read_voltage(dvc));
  else {
    not_critical("ag_new: Analog interface not initialised.\n");
    return FAIL;
  }

}

  
extern void ag_shutdown () {
  
  int i;

  if(!status.mt)
    unexportall();
  
  status.ag = false;
  
  for (i = 0; i <= MAX_PORT; i++)
    digitalWrite(ypin_port[i], LOW);
  
  if(an_fd > 0)
    close(an_fd);
}


static void not_critical (char* fmt, ...) {

  if (ag_log_lvl < LOG_LVL_ADV)
    return;

  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);
  
}

static void debug (char* fmt, ...) {
  
  if (ag_log_lvl < LOG_LVL_DBG)
    return;
  
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);

}




