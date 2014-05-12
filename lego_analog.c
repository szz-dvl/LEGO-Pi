#include "lego_analog.h"

//#define NEVER_HAPPENS   4

INIT status;

static int an_fd = 0;

static uint8_t port_busy = 0;

static bool lpin_state [] = {false, false, false, false};
static int  ypin_port  [] = {L_PORT0, L_PORT1, L_PORT2, L_PORT3};

static int act_gyro = HT_GYRO_DEF;
static int last_gyro = 0;

static int SPIreceive (int, uint8_t [], int);
static double analog_read_voltage (ANDVC * dvc);
static int analog_read_int (ANDVC * dvc);

static int SPIreceive (int fd, uint8_t resp[], int chann) {

  struct spi_ioc_transfer spi;
  int retval ;

  
  //FROM: http://code.google.com/p/webiopi/source/browse/trunk/python/webiopi/devices/analog/mcp3x0x.py
  resp[0] |= 1 ;
  resp[1] |= (1 << 7) ;	//! differential mode, single-ended...
  resp[1] |= ((chann >> 2) & 0x01) << 6 ;
  resp[1] |= ((chann >> 1) & 0x01) << 5 ;
  resp[1] |= ((chann >> 0) & 0x01) << 4 ;
  
  spi.tx_buf = (unsigned long)resp ;
  spi.rx_buf = (unsigned long)resp ;
  spi.len = (__u32) LEN ;
  spi.delay_usecs = 0 ;
  spi.speed_hz = (__u32) SPI_CLK ;
  spi.bits_per_word = 8 ;
  spi.cs_change = 0 ;
  spi.pad = 0 ;

  retval = ioctl (fd, SPI_IOC_MESSAGE(1), &spi) ;

  //printf("resp[0] = %u, resp[1] = %u, resp[2] = %u\n", resp[0], resp[1], resp[2]);
  return retval ;
}

static double analog_read_voltage (ANDVC * dvc) { /* READ analog value (float), VOLTATGE */

  uint8_t data [LEN] = { 0x00, 0x00, 0x00 } ;
  int ioctl, retval = true, res;
  
  if (!(ioctl = SPIreceive(an_fd, data, dvc->port))) {
    
   not_critical("analog_read_voltage: Error comunicating to SPI device\n");
   retval = FAIL;
   
  }
  
  if(retval != FAIL) {
    //printf("entro aki? 0x%02x, 0x%02x, 0x%02x", data[0], data[1], data[2]);
    res = data[2];
    res |= ((data[1] & 0x07) << 8);
    //printf (" res = %d\n", res);
    return ((double) ((double)res/(double)MAX_VAL) * VREF);
    
  } else
    return retval;
  
}

static int analog_read_int (ANDVC * dvc) { /* READ analog value (integer), max avalue = 1023 */

  uint8_t data [LEN] = { 0x00, 0x00, 0x00 } ;
  int ioctl, retval = true, res;
    
  if (!(ioctl = SPIreceive(an_fd, data, dvc->port))) {
    
    not_critical("analog_read_int: Error comunicating to SPI device\n") ;
    retval = FAIL ;   
  }
  
  if(retval != FAIL) {
    res = data[2];
    res |= ((data[1] & 0x07) << 8);
    //printf("data readed: data[2] = %u, data[1] = %u, data[0] = %u,  retval = %d\n", data[2], data[1], data[0], retval);
    return res;
    
  } else
    return retval;
  
}


extern bool ag_init() {

  int i;

  if(!status.wpi)
    status.wpi = wiringPiSetupGpio() == 0;
  
  for (i = 0; i <= MAX_PORT; i++)
    pinMode(ypin_port[i], OUTPUT);    //set all the pins wired to yellow wire to OUTPUT

  if (!(an_fd = wiringPiSPISetup(CS, SPI_CLK))) {
    not_critical("analog_setup: Error setting up SPI interface, ERRNO: %d\n", errno);
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
	else if(type  == SOUND)
	  digitalWrite(ypin_port[dvc->port], HIGH); //REVISAR!won't change, for SOUND sensor we need 3.3V in the yellow wire (to get dB), dBA not accessible due to the lack of pins
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

extern bool ag_lgt_get_ledstate (ANDVC* dvc){
  
  if(status.ag) {
    if (dvc->type != LIGHT) {
      not_critical("ag_lgt_get_ledstate: Device type must be %d (LIGHT)\n", LIGHT);
      return false;
    } else
      return lpin_state[dvc->port];
  } else {
    not_critical("ag_lgt_get_ledstate: Analog interface not initialised.\n");
    return false;
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
      //printf("Voltage is: %f\n", *volt);
      if(*volt < (double)(VREF/2))
	return true;
      else
	return false;
    }
  } else {
    not_critical("ag_psh_is_pushed: Analog interface not initialised.\n");
    return false;
  }

}

extern int ag_snd_get_db (ANDVC * dvc) { 

  if(status.ag) {
    
    if(dvc->type == SOUND) {
      
      int val;
      
      if( (val = analog_read_int(dvc)) != FAIL) {
	
	return( ((double)(MAX_VAL-val)/MAX_VAL) * MAX_DB );
	
      } else
	return FAIL;

    } else {

      not_critical("ag_snd_get_db: Device type must be %d (SOUND)\n", SOUND);
      return false;

    }  

  } else {
    
    not_critical("ag_new: Analog interface not initialised.\n");
    return false;
    
  }

}

extern int ag_gyro_get_val (ANDVC * dvc) { //Positives values for clockwise rotation, negatives for anticlockwise, the higher the value returned is the faster we turn. 

  if(status.ag) {
    
    if(dvc->type == HT_GYRO) {
      
      int val;
      
      if( (val = analog_read_int(dvc)) != FAIL)
	last_gyro = val - act_gyro;
	
	return last_gyro;

    } else {

      not_critical("ag_gyro_get_val: Device type must be %d (HT_GYRO)\n", HT_GYRO);
      return false;

    }  

  } else {
    
    not_critical("ag_gyro_get_val: Analog interface not initialised.\n");
    return false;
    
  }

}

extern bool ag_gyro_cal (ANDVC * dvc) { //Make sure the robot is stationary before call this one!

  if(status.ag) {
    
    if(dvc->type == HT_GYRO) {
      
      int acum = 0, val;
      int i = 5;
      
      while (i > 0) {
	
	if( (val = analog_read_int(dvc)) != FAIL){
	  acum += val;
	  i--;
	  val = 0;
	} else
	  val = 0;
	
	DELAY_US(50);
      }
      
      act_gyro = (acum / 5);

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
    return false;
  }
}

extern double ag_read_volt (ANDVC * dvc){
  if (status.ag) 
    return (analog_read_voltage(dvc));
  else {
    not_critical("ag_new: Analog interface not initialised.\n");
    return false;
  }

}

  
extern void ag_shutdown () {
  
  if(!status.mt)
    unexportall();
  
  status.ag = false;
  
  if(an_fd > 0)
    close(an_fd);
}





