//#include <lego/lego_i2c.h>
#include "lego_digital.h"

INIT              status;

static uint8_t    port_busy = 0;

static I2C_DVC    ** ptable; //to shutdown every device initialized 

static int        retries;

static int       dg_log_lvl;

char * us_titles [] = {
  "Product Version: ",
  "Product ID: ", 
  "Sensor Type: ", 
  "Factory Zero: ",
  "Factory Scale Factor: ",
  "Factory Scale Divisor: ",
  "Measurement Units: ", 
  "Measurement Interval: "
};

char * ht_titles [] = {
  "Product Version: ",
  "Manufacturer: ", 
  "Sensor Type: "
};

//Commands                                                         
CMD cmds [] = {
  {CMD_REG, 0x00, US_CWAIT, 1}, //LEGO US off cmd
  {CMD_REG, 0x01, US_CWAIT, 1}, //LEGO US single shot mode
  {CMD_REG, 0x02, US_CWAIT, 1}, //LEGO US continuous measurement mode (def)
  {CMD_REG, 0x03, US_CWAIT, 1}, //LEGO US even capture mode
  {CMD_REG, 0x04, US_CWAIT, 1}, //LEGO US warm reset cmd
  {CMD_REG, 0x43, 0, -1},       //Hitechnic Color calibrate white (v1)
  {CMD_REG, 0x00, 200, 2},      //Hitechnic Color active mode (v2) [Use ambient light cancellation] (def)
  {CMD_REG, 0x01, 200, 2},      //Hitechnic Color passive mode (v2) [Disable ambient light cancellation]
  {CMD_REG, 0x03, 200, 2},      //Hitechnic Color Raw mode (v2)
  {CMD_REG, 0x35, 0, 2},        //Hitechnic Color 50Hz cancellation mode (v2)
  {CMD_REG, 0x36, 0, 2},        //Hitechnic Color 60Hz cancellation mode (v2)
  {CMD_REG, 0x00, 0, 1},        //Hitechnic Compass Measurement mode
  {CMD_REG, 0x43, 0, 1},        //Hitechnic Compass Calibrate mode
  {CMD_REG, 0x00, 0, 2},        //Hitechnic IRSeeker AC DSP mode [1200Hz] (v2)
  {CMD_REG, 0x01, 0, 2}         //Hitechnic IRSeeker AC DSP mode [600Hz] (v2)
  
};

//Messages 
MSG regs [] = {
  //Hitechnic generic
  {0x01, 7, HT_DLY, 1},        //Hitechnic version number 
  {0x08, 8, HT_DLY, 1},        //Hitechnic manufacturer 
  {0x10, 8, HT_DLY, 1},        //Hitechnic sensor type
  {0x41, 1, HT_DLY, 1},        //Hitechnic "Generic" command state.

  //Hitechnic Color 
  {0x42, 1, HT_DLY, 1},        //Hitechnic Color col. number 
  {0x43, 1, HT_DLY, 1},        //Hitechnic Color red reading 
  {0x44, 1, HT_DLY, 1},        //Hitechnic Color green reading 
  {0x45, 1, HT_DLY, 1},        //Hitechnic Color blue reading 

  //Hitechnic Color V1 only
  {0x46, RAW_LEN, HT_DLY, 1},  //Hitechnic Color Raw red reading (v1)   [10 bits] 
  {0x48, RAW_LEN, HT_DLY, 1},  //Hitechnic Color Raw green reading (v1) [10 bits]
  {0x4a, RAW_LEN, HT_DLY, 1},  //Hitechnic Color Raw blue reading (v1)  [10 bits]
  {0x4c, 1, HT_DLY, 1},        //Hitechnic Color col. index (v1)
  {0x4d, 1, HT_DLY, 1},        //Hitechnic Color Normalised red reading (v1)
  {0x4e, 1, HT_DLY, 1},        //Hitechnic Color Normalised green reading (v1)
  {0x4f, 1, HT_DLY, 1},        //Hitechnic Color Normalised blue reading (v1) 

  //Hitechnic Color V2 only [just guessing here...]
  {0x46, 1, HT_DLY, 2},        //Hitechnic Color white reading (v2)
  {0x47, 1, HT_DLY, 2},        //Hitechnic Color col. index (v2)
  {0x48, 1, HT_DLY, 2},        //Hitechnic Color Normalised red reading (v2)
  {0x49, 1, HT_DLY, 2},        //Hitechnic Color Normalised green reading (v2)
  {0x4a, 1, HT_DLY, 2},        //Hitechnic Color Normalised blue reading (v2)
  {0x42, RAW_LEN, HT_DLY, 2},  //Hitechnic Color Raw red reading (v2)   [16 bits] 
  {0x44, RAW_LEN, HT_DLY, 2},  //Hitechnic Color Raw green reading (v2) [16 bits] 
  {0x46, RAW_LEN, HT_DLY, 2},  //Hitechnic Color Raw blue reading (v2)  [16 bits] 
  {0x48, RAW_LEN, HT_DLY, 2},  //Hitechnic Color Raw white reading (v2) [16 bits]

  //Hitechnic IRSeeker
  {0x42, 1, HT_DLY, 1},        //Hitechnic IRSeeker DC signal direction 
  {0x43, 1, HT_DLY, 1},        //Hitechnic IRSeeker DC signal strength 1 / Direction 1   
  {0x44, 1, HT_DLY, 1},        //Hitechnic IRSeeker DC signal strength 2 / Direction 3
  {0x45, 1, HT_DLY, 1},        //Hitechnic IRSeeker DC signal strength 3 / Direction 5
  {0x46, 1, HT_DLY, 1},        //Hitechnic IRSeeker DC signal strength 4 / Direction 7
  {0x47, 1, HT_DLY, 1},        //Hitechnic IRSeeker DC signal strength 5 / Direction 9
  {0x48, 1, HT_DLY, 2},        //Hitechnic IRSeeker DC signal strength average (v2)
  {0x49, 1, HT_DLY, 2},        //Hitechnic IRSeeker AC signal direction (v2)
  {0x4a, 1, HT_DLY, 2},        //Hitechnic IRSeeker AC signal strength 1 / Direction 1 (v2)
  {0x4b, 1, HT_DLY, 2},        //Hitechnic IRSeeker AC signal strength 2 / Direction 3 (v2)
  {0x4c, 1, HT_DLY, 2},        //Hitechnic IRSeeker AC signal strength 3 / Direction 5 (v2)
  {0x4d, 1, HT_DLY, 2},        //Hitechnic IRSeeker AC signal strength 4 / Direction 7 (v2)
  {0x4e, 1, HT_DLY, 2},        //Hitechnic IRSeeker AC signal strength 5 / Direction 9 (v2)

  //Hitechnic Accel.
  {0x42, 1, HT_DLY, 1},        //Hitechnic Accelerometer "X" upper byte [signed]
  {0x43, 1, HT_DLY, 1},        //Hitechnic Accelerometer "Y" upper byte [signed]
  {0x44, 1, HT_DLY, 1},        //Hitechnic Accelerometer "Z" upper byte [signed]
  {0x45, 1, HT_DLY, 1},        //Hitechnic Accelerometer "X" lower byte [2 bits]
  {0x46, 1, HT_DLY, 1},        //Hitechnic Accelerometer "Y" lower byte [2 bits]
  {0x47, 1, HT_DLY, 1},        //Hitechnic Accelerometer "Z" lower byte [2 bits]
  
  //Hitechnic Compass
  {0x42, 1, HT_DLY, 1},        //Hitechnic Compass two degree heading
  {0x43, 1, HT_DLY, 1},        //Hitechnic Compass one degree adder
  {0x44, 2, HT_DLY, 1},        //Hitechnic Compass heading [16 bits]

  //LEGO Ultrasonic
  {0x00, 5, US_IWAIT, 1},      //LEGO Ultrasonic product version 
  {0x08, 5, US_IWAIT, 1},      //LEGO Ultrasonic product ID
  {0x10, 6, US_IWAIT, 1},      //LEGO Ultrasonic sensor type
  {0x11, 1, US_IWAIT, 1},      //LEGO Ultrasonic factory zero
  {0x12, 1, US_IWAIT, 1},      //LEGO Ultrasonic factory scale factor
  {0x13, 1, US_IWAIT, 1},      //LEGO Ultrasonic factory scale divisor
  {0x14, 7, US_IWAIT, 1},      //LEGO Ultrasonic measurement units
  {0x40, 1, US_IWAIT, 1},      //LEGO Ultrasonic measurement interval
  {0x41, 1, US_IWAIT, 1},      //LEGO Ultrasonic command state
  {0x42, 1, US_DWAIT, 1},      //LEGO Ultrasonic distance 0
  {0x43, 1, US_DWAIT, 1},      //LEGO Ultrasonic distance 1
  {0x44, 1, US_DWAIT, 1},      //LEGO Ultrasonic distance 2
  {0x45, 1, US_DWAIT, 1},      //LEGO Ultrasonic distance 3
  {0x46, 1, US_DWAIT, 1},      //LEGO Ultrasonic distance 4
  {0x47, 1, US_DWAIT, 1},      //LEGO Ultrasonic distance 5
  {0x48, 1, US_DWAIT, 1},      //LEGO Ultrasonic distance 6
  {0x49, 1, US_DWAIT, 1}       //LEGO Ultrasonic distance 7
};


static void check_device(DGDVC * dev);
static void get_version (DGDVC * dev);
static bool match_cmd (dgType type, cmdIdx cmd); 
static bool match_cmd_ver (int vers, cmdIdx cmd);
static bool send_message (DGDVC * dev, msgIdx msg, uint16_t * reply);
static void raw_data_to_str (uint16_t raw [], char out[], int len);
static bool send_cmd (DGDVC * dev, cmdIdx cmd);
static void not_critical (char *fmt, ...);
static void debug (char *fmt, ...);

extern bool dg_set_verbose(int lvl) {

  if(status.dg){

    if (lvl >=0 && lvl <= LOG_LVL_DBG){
      dg_log_lvl = lvl;
      i2c_set_loglvl(lvl < LOG_LVL_DBG ? LOG_QUIET : LOG_PRINT);
      return true;
    } else {
      printf("dg_set_verbose: Log level \"%d\" out of bounds\n", lvl);
      return false;
    }

  } else {
    not_critical("dg_set_verbose: Digital interface not initialised\n");
    return false;
  }
}

extern bool dg_init(int retry){
  if (retry < 0) {
    not_critical("dg_init: Setting retries to 0.\n");
    retries = 0;
  } else
    retries = retry;
  
  ptable = calloc(MAX_PORT_DG+1, sizeof(I2C_DVC *));
  status.dg = i2c_init(LOG_QUIET);
  
  return status.dg;
}

extern bool dg_new (DGDVC * dev, dgType type, int port) {

  if(status.dg) {

    if (type == DG_OTHER) {
      not_critical("dg_new: To initialise unknown devices use dg_new_unknown.\n");
      return false;
    } else if (type > DG_OTHER && type <= HT_IRS){
      if(port < MIN_PORT_DG || port > MAX_PORT_DG){
	not_critical("dg_new: Port must be between %d and %d.\n", MIN_PORT_DG, MAX_PORT_DG);
	return false;
      } else {
	dev->type = type;
	dev->port = port;
	dev->vers = 0;
	dev->dvc = (I2C_DVC *)malloc(sizeof(I2C_DVC));
	
	if(!i2c_new_device(dev->dvc, LEGO_ADDR, LEGO_FREQ, port == 0 ? SDA_0 : SDA_1, port == 0 ? SCL_0 : SCL_1, type == LEGO_US ? GPPUD_PULLDOWN : GPPUD_PULLUP, GPPUD_PULLUP)){
	  not_critical("dg_new: Digital port %d busy.\n", port);
	  return false;
	} else
	  port_busy |= (1 << port);
	
	check_device(dev);
	
	if (dev->vers == 0) {
	  not_critical("dg_new: Device does not match the demanded device type.\n");
	  port_busy &= ~(1 << port);
	  i2c_shutdown(dev->dvc);
	  return false ;
	} else {
	  ptable[port] = dev->dvc;
	  if(dev->type == HT_COLOR && dev->vers > 1)
	    send_cmd(dev, HTCS_ACT_MD);
	  return true;
	}
      }
      
    } else {

      not_critical("dg_new: Unrecognized device type\n", port);
      return false;
    }
  } else {
    
    not_critical("dg_new: Digital interface not initialised\n");
    return false;

  }
}
 
extern bool dg_new_unknown (DGDVC * dev, uint8_t addr, int freq, int port) {
  
  if(status.dg) {
  
    if(port < MIN_PORT_DG || port > MAX_PORT_DG){
  
      not_critical("dg_new_unknown: Port must be between %d and %d.\n", MIN_PORT_DG, MAX_PORT_DG);
      return false;
      
    } else {
      dev->type = DG_OTHER;
      dev->port = port;
      dev->vers = 0;
      dev->dvc = (I2C_DVC *)malloc(sizeof(I2C_DVC));
      if(!i2c_new_device(dev->dvc, addr, freq, port == 0 ? SDA_0 : SDA_1, port == 0 ? SCL_0 : SCL_1, GPPUD_PULLUP, GPPUD_PULLUP)){
	not_critical("dg_new_unknown: Digital port %d busy.\n", port);
	return false;
      } else { 
	port_busy |= (1 << port);
	ptable[port] = dev->dvc;
      }
    }
    return true;
    
  } else {
    
    not_critical("dg_new_unknown: Digital interface not initialised\n");
    return false;
    
  }

}

extern void dg_shutdown () {

  if (port_busy & 0x01)
    i2c_shutdown(ptable[0]);
  
  if (port_busy & 0x02)
    i2c_shutdown(ptable[1]);

  status.dg = false;

}
 
static void check_device(DGDVC * dev) {

  
  int i;
  char straux[IN_MAX_LEN + 1];
  int retry = retries; 
  bool ret = false;
  uint8_t data_aux[1];
  uint16_t data_in[IN_MAX_LEN];

  if (IS_HITECH(dev->type)) {
    for(i=0; i<regs[HT_SEN_TYPE].len; i++) {
      data_aux[0] = regs[HT_SEN_TYPE].base + i;
      ret = false;
      while (!ret && retry >= 0){
	if(!(ret = i2c_transfer(dev->dvc, data_aux, 1, false, data_in, 1, false, regs[HT_SEN_TYPE].delay))){
	  retry --;
	  debug("I2C transaction failed, REG = 0x%02x, retry = %d\n", regs[HT_SEN_TYPE].base + i, retries - retry); //futur debug.
	  DELAY_US(DEF_DELAY);
	} 
	  
      }
      if (retry < 0 && !ret) {
	not_critical("check_device: i2c transaction failed, REG = 0x%02x\n", regs[HT_SEN_TYPE].base + i);
	return;
      } else {
	straux[i] = (char)data_in[0];
      }
    }
    
    straux[i] = '\0';
    
  } else { //Ultrasonic
    data_aux[0] = regs[US_SEN_TYPE].base;
    while (!ret && retry >= 0){
      if(!(ret = i2c_transfer(dev->dvc, data_aux, 1, true, data_in, regs[US_SEN_TYPE].len, false, regs[US_SEN_TYPE].delay))){
	retry --;
	debug("I2C transaction failed, REG = 0x%02x, retry = %d\n", regs[US_SEN_TYPE].base, retries - retry); //futur debug.
	DELAY_US(DEF_DELAY);
      }
    }
    if (retry < 0 && !ret) {
      not_critical("check_device: i2c transaction failed, REG = 0x%02x.\n", regs[US_SEN_TYPE].base);
      return;
    } else {
      
      for(i=0; i<regs[US_SEN_TYPE].len; i++)
	straux[i] = (char)data_in[i];  
    }
    
  }
  
  char * aux;
  switch (dev->type){
  case LEGO_US:
    {
      if (strstr(straux, "Sonar") != NULL)
	get_version(dev);
      else
	not_critical("check_device: Sensor type check failed. %s\n", straux);
    }
    break;;
  case HT_COMPASS:
    {
      if (strstr(straux, "Compass") != NULL)
	get_version(dev);
      else
	not_critical("check_device: Sensor type check failed. %s\n", straux);
    }
    break;;
  case HT_ACCEL:
    {
      if (strstr(straux, "Accel") != NULL)
	get_version(dev);
      else
	not_critical("check_device: Sensor type check failed. %s\n", straux);
    }
    break;;
  case HT_COLOR:
    {
      if ((aux = strstr(straux, "Color")) != NULL) //Must work for v1 and v2
	get_version(dev);
      else {
	not_critical("check_device: Sensor type check failed. %s\n", straux);
      }
    }
    break;;
  case HT_IRS:
    {
      if (strstr(straux, "IR") != NULL) //May fail for v2..
	get_version(dev);
      else
	not_critical("check_device: Sensor type check failed. %s\n", straux);
    }
    break;;
  default:
    not_critical("check_device: Device type not recognised.\n");
    break;;
  }

}

static void get_version (DGDVC * dev) {
  
  int retry = retries; 
  bool ret = false;
  uint8_t data_aux[1];
  uint16_t data_in[IN_MAX_LEN];
  char vaux[2];

  if (IS_HITECH(dev->type)) {
      data_aux[0] = regs[HT_VER_NUM].base + 1;
      ret = false;
      while (!ret && retry >= 0){
	if(!(ret = i2c_transfer(dev->dvc, data_aux, 1, false, data_in, 1, false, regs[HT_VER_NUM].delay))){
	  retry --;
	  debug("I2C transaction failed, REG = 0x%02x, retry = %d\n", regs[HT_VER_NUM].base + 1, retries - retry); 
	  DELAY_US(DEF_DELAY);
	}
      }
      if (retry < 0 && !ret) {
	not_critical("get_version: i2c transaction failed, REG = 0x%02x.\n", regs[HT_VER_NUM].base + 1);
	return;
      } else {
	vaux[0] = (char)data_in[0];
	vaux[1] = '\0';
	dev->vers = atoi(vaux);
	//printf("DEBUG: vaux = %s, data_in = %u, vers = %d\n", vaux, data_in[0], dev->vers);
      }
    
  } else { //Ultrasonic
    data_aux[0] = regs[US_PR_VER].base;
    while (!ret && retry >= 0){
      if(!(ret = i2c_transfer(dev->dvc, data_aux, 1, true, data_in, regs[US_PR_VER].len, false, regs[US_PR_VER].delay))){
	retry --;
        debug("I2C transaction failed, REG = 0x%02x, retry = %d\n", regs[US_PR_VER].base, retries - retry); 
	DELAY_US(DEF_DELAY);
      }
    }
    if (retry < 0 && !ret) {
      not_critical("get_version: i2c transaction failed, REG = 0x%02x.\n", regs[US_PR_VER].base);
      return;
    } else {
      vaux[0] = (char)data_in[1];
      vaux[1] = '\0';
      dev->vers = atoi(vaux);
    }
  }
  
}

extern bool dg_send_cmd (DGDVC * dev, cmdIdx cmd) {

  if(status.dg)
    
    return (send_cmd(dev, cmd));
    
  else {
    
    not_critical("dg_send_cmd: Digital interface not initialised\n");
    return false;
    
  }

}


static bool send_cmd (DGDVC * dev, cmdIdx cmd) {
  
    bool ret = false;
    int retry = retries;
    uint8_t data_aux[] = {cmds[cmd].reg, cmds[cmd].val};
  
    if (match_cmd(dev->type, cmd)){
      if (match_cmd_ver(dev->vers, cmd)){
	
	while (!ret && retry >= 0) {
	  if(!(ret = i2c_write(dev->dvc, data_aux, 2, cmds[cmd].delay))){
	    retry --;
	    debug("I2C transaction failed, CMD = {0x%02x, 0x%02x} retry = %d\n", cmds[cmd].reg , cmds[cmd].val, retries - retry); //futur debug.
	    DELAY_US(DEF_DELAY);
	  }
	}
	if(retry < 0 && !ret){
	  not_critical("send_cmd: i2c transaction failed, REG = 0x%02x, CMD = 0x%02x.\n", cmds[cmd].reg, cmds[cmd].val);
	  return false;
	}
      } else {
	not_critical("send_cmd: Unsupported sensor version.\n");
	return false;
      }
    } else {
      not_critical("send_cmd: Sensor type doesn't match command.\n");
      return false;
    }

    return true;
    
}

static bool match_cmd (dgType type, cmdIdx cmd) {

  switch (type){
  case LEGO_US:
    return IS_US_CMD(cmd);
  case HT_COMPASS:
    return IS_HTCM_CMD(cmd);
  case HT_COLOR:
    return IS_HTCS_CMD(cmd);
  case HT_IRS:
    return IS_HTIS_CMD(cmd);
  default:
    return false;
  }

}

static bool match_cmd_ver (int vers, cmdIdx cmd) {

  if(abs(cmds[cmd].minvrs) == vers && cmds[cmd].minvrs < 0)
    return true;
  else if (vers >= cmds[cmd].minvrs && cmds[cmd].minvrs > 0)
    return true;
  else 
    return false;

}

static bool send_message (DGDVC * dev, msgIdx msg, uint16_t * reply) {

  bool ret = false;
  uint8_t data_aux[1];
  int retry = retries;
  
  if (dev->vers >= regs[msg].minvrs){
    if IS_HITECH(dev->type) {
	uint16_t data_in[1];
	int i;
	for(i=0; i<regs[msg].len; i++){
	  data_aux[0] = regs[msg].base + i;
	  ret = false;
	  while (!ret && retry >= 0){
	    if(!(ret = i2c_transfer(dev->dvc, data_aux, 1, false, data_in, 1, false, regs[msg].delay))){
	      retry --;
	      debug("I2C transaction failed, REG = 0x%02x retry = %d\n", regs[msg].base + i, retries - retry); //futur debug.
	      DELAY_US(DEF_DELAY);
	    }
	  }
	  if(retry < 0 && ! ret){
	    not_critical("send_message: i2c transaction failed, REG = 0x%02x.\n", regs[msg].base + i);
	    return false;
	  } else
	    reply[i] = data_in[0];
	}
      } else { //Ultrasonic
      data_aux[0] = regs[msg].base; 
      while (!ret && retry >= 0){
	if(!(ret = i2c_transfer(dev->dvc, data_aux, 1, true, reply, regs[msg].len, false, regs[msg].delay))){
	  retry --;
	  debug("I2C transaction failed, REG = 0x%02x retry = %d\n", regs[msg].base, retries - retry); //futur debug.
	  DELAY_US(DEF_DELAY);
	}
      }
      if(retry < 0 && !ret){
	not_critical("send_message: i2c transaction failed, REG = 0x%02x.\n", regs[msg].base);
	return false;
      }
    }
    
  } else {
    not_critical("send_message: Unsupported sensor version.\n");
    return false;
  }
  
  return true;
}

extern bool dg_col_get_rgb (DGDVC * dvc, uint8_t * red, uint8_t * green, uint8_t * blue) {

  if(status.dg) {
    
    bool ret = false;
    
    if (dvc->type == HT_COLOR) {
      
      uint16_t taux[1];
      
      //if(dvc->vers == 2)
      //send_cmd(dvc, HTCS_ACT_MD);
    
      if ((ret = send_message(dvc, HTCS_RED, taux)))
	*red = (uint8_t)taux[0];
      else 
	not_critical("dg_col_get_rgb: Red reading failed.\n");
      
      if(send_message(dvc, HTCS_GREEN, taux))
	*green = (uint8_t)taux[0];
      else {
	not_critical("dg_col_get_rgb: Green reading failed.\n");
	ret = false;
      }
      
      if(send_message(dvc, HTCS_BLUE, taux))
	*blue = (uint8_t)taux[0];
      else {
	not_critical("dg_col_get_rgb: Blue reading failed.\n");
	ret = false;
      }
      
    } else {
      not_critical("dg_col_get_rgb: Device type must be HT_COLOR [%d]\n", HT_COLOR);
      return false;
    }
  
    return ret;
  
  } else {
    
    not_critical("dg_col_get_rgb: Digital interface not initialised\n");
    return false;
    
  }

}

extern bool dg_col_get_norm (DGDVC * dvc, uint8_t * red, uint8_t * green, uint8_t * blue) {

  if(status.dg) {
  
    bool ret = false;

    if(dvc->type == HT_COLOR) {
      
      uint16_t taux[1];
    
      if (dvc->vers == 1) {
	
	if ((ret = send_message(dvc, HTCS_NRM_RED, taux)))
	  *red = (uint8_t)taux[0];
	else 
	  not_critical("dg_col_get_norm: Red reading failed.\n");
	
	if(send_message(dvc, HTCS_NRM_GREEN, taux))
	  *green = (uint8_t)taux[0];
	else {
	  not_critical("dg_col_get_norm: Green reading failed.\n");
	  ret = false;
	}
	
	if(send_message(dvc, HTCS_NRM_BLUE, taux))
	  *blue = (uint8_t)taux[0];
	else {
	  not_critical("dg_col_get_norm: Blue reading failed.\n");
	  ret = false;
	}      
	
      } else if(dvc->vers == 2) {
	
	//send_cmd(dvc, HTCS_ACT_MD);
	
	if ((ret = send_message(dvc, HTCS2_NRM_RED, taux)))
	  *red = (uint8_t)taux[0];
	else 
	  not_critical("dg_col_get_norm: Red reading failed.\n");
	
	if(send_message(dvc, HTCS2_NRM_GREEN, taux))
	  *green = (uint8_t)taux[0];
	else {
	  not_critical("dg_col_get_norm: Green reading failed.\n");
	  ret = false;
	}
	
	if(send_message(dvc, HTCS2_NRM_BLUE, taux))
	  *blue = (uint8_t)taux[0];
	else {
	  not_critical("dg_col_get_norm: Blue reading failed.\n");
	  ret = false;
	}
	
      }
      
    } else {
    
      not_critical("dg_col_get_norm: Device type must be HT_COLOR [%d]\n", HT_COLOR);
      return false; 
    }
  
    return ret;
    
  } else {
    
    not_critical("dg_col_get_norm: Digital interface not initialised\n");
    return false;
    
  }

} 

extern bool dg_col_get_index (DGDVC * dvc, uint8_t * idx){

  if(status.dg) {

  
    if(dvc->type == HT_COLOR) {
    
      uint16_t taux[1];
    
      if (dvc->vers == 1) {
	
	if(send_message(dvc, HTCS_COL_IDX, taux))
	  *idx = (uint8_t)taux[0];
	else {
	  not_critical("dg_col_get_index: Color index reading failed.\n");
	  return false;
	}      
	
      } else if(dvc->vers == 2) {
	
	//send_cmd(dvc, HTCS_ACT_MD);
	
	if(send_message(dvc, HTCS2_COL_IDX, taux))
	  *idx = (uint8_t)taux[0];
	else {
	  not_critical("dg_col_get_index: Color index reading failed.\n");
	  return false;
	}
	
      }
    } else {
      
      not_critical("dg_col_get_index: Device type must be HT_COLOR [%d]\n", HT_COLOR);
      return false; 
    }
 
    return true;
  } else {
    
    not_critical("dg_col_get_index: Digital interface not initialised\n");
    return false;
    
  }

}
  
extern bool dg_col_get_number (DGDVC * dvc, uint8_t * num){
  
  if(status.dg) {
 
    if(dvc->type == HT_COLOR) {
    
      uint16_t taux[1];
    
      //if (dvc->vers == 2) 
      //send_cmd(dvc, HTCS_ACT_MD);
    
      if(send_message(dvc, HTCS_COL_NUM, taux))
	*num = (uint8_t)taux[0];
      else {
	not_critical("dg_col_get_number: Color number reading failed.\n");
	return false;
      }
      
    } else {
      
      not_critical("dg_col_get_number: Device type must be HT_COLOR [%d]\n", HT_COLOR);
      return false; 
    }
    
    return true;

  } else {
    
    not_critical("dg_col_get_number: Digital interface not initialised\n");
    return false;
    
  }

}

extern bool dg_col_get_white (DGDVC * dvc, uint16_t * white, bool raw, bool passive){

  if(status.dg) {
  
    if(dvc->type == HT_COLOR) {
    
      uint16_t taux[raw ? RAW_LEN : 1];
      
      if (dvc->vers == 2) { 
	if(!raw) {
	  //send_cmd(dvc, HTCS_ACT_MD);
	  
	  if(send_message(dvc, HTCS2_WHITE, taux))
	    *white = taux[0];
	  else {
	    not_critical("dg_col_get_white: White reading failed.\n");
	    return false;
	  }
	} else {
	  if(passive){
	    if(!send_cmd(dvc, HTCS_PAS_MD)){
	      not_critical("dg_col_get_white: Enable passive mode failed.\n");
	      goto back_to_def;
	    }
	  }
	  
	  if(!send_cmd(dvc, HTCS_RAW_MD)){
	    not_critical("dg_col_get_white: Enable raw mode failed.\n");
	    goto back_to_def;
	  }
	  
	  if(send_message(dvc, HTCS2_RAW_WHITE, taux)){
	    *white = taux[0] << BYTE_LEN;
	    *white |= taux[1] & 0xff;
	  } else {
	    not_critical("dg_col_get_white: Raw white reading failed.\n");
	    goto back_to_def;
	  }
	  
	  if (!send_cmd(dvc, HTCS_ACT_MD)){
	    not_critical("dg_col_get_white: Reenabling active mode failed.\n");
	    goto back_to_def; //insisto...
	  }
	}
	
      } else {
	
	not_critical("dg_col_get_white: Unsupported sensor version.\n");
	return false;
      }
      
    } else {
      
      not_critical("dg_col_get_white: Device type must be HT_COLOR [%d].\n", HT_COLOR);
      return false; 
    }
    
    return true;
    
  } else {
    
    not_critical("dg_col_get_white: Digital interface not initialised\n");
    return false;
    
  }


 back_to_def:
  if (!send_cmd(dvc, HTCS_ACT_MD))
    not_critical("dg_col_get_white: Reenabling active mode failed.\n");
  return false;

}
  
extern bool dg_col_get_raw (DGDVC * dvc, uint16_t * red, uint16_t * green, uint16_t * blue, bool passive) {

  if(status.dg) {
  
    bool ret = false;

    if(dvc->type == HT_COLOR) {
      
      uint16_t taux[RAW_LEN];
      
      if (dvc->vers == 1) {
	
	if(passive)
	  not_critical("dg_col_get_raw: Ignorign passive mode, unsupported sensor version.\n");
	
	if ((ret = send_message(dvc, HTCS_RAW_RED, taux))) {
	  *red = taux[0] << BYTE_LEN;
	  *red |= taux[1] & 0xff; 
	}
	else 
	  not_critical("dg_col_get_raw: Red reading failed.\n");
	
	if(send_message(dvc, HTCS_RAW_GREEN, taux)) {
	  *green = taux[0] << BYTE_LEN;
	  *green |= taux[1] & 0xff; 
	} else {
	  not_critical("dg_col_get_raw: Green reading failed.\n");
	  ret = false;
	}
	
	if(send_message(dvc, HTCS_RAW_BLUE, taux)) {
	  *blue = taux[0] << BYTE_LEN;
	  *blue |= taux[1] & 0xff; 
	} else {
	  not_critical("dg_col_get_raw: Blue reading failed.\n");
	  ret = false;
	}      
	
      } else if(dvc->vers == 2) {
	
	if(passive){
	  if(!send_cmd(dvc, HTCS_PAS_MD)){
	    not_critical("dg_col_get_raw: Enable passive mode failed.\n");
	    goto back_to_def;
	  }
	}
	
	if(!send_cmd(dvc, HTCS_RAW_MD)){
	  not_critical("dg_col_get_raw: Enable raw mode failed.\n");
	  goto back_to_def;
	}
	
	if ((ret = send_message(dvc, HTCS2_RAW_RED, taux))) {
	  *red = taux[0] << BYTE_LEN;
	  *red |= taux[1] & 0xff; 
	}
	else 
	  not_critical("dg_col_get_raw: Red reading failed.\n");
	
	if(send_message(dvc, HTCS2_RAW_GREEN, taux)) {
	  *green = taux[0] << BYTE_LEN;
	  *green |= taux[1] & 0xff; 
	} else {
	  not_critical("dg_col_get_raw: Green reading failed.\n");
	  ret = false;
	}
	
	if(send_message(dvc, HTCS2_RAW_BLUE, taux)) {
	  *blue = taux[0] << BYTE_LEN;
	  *blue |= taux[1] & 0xff; 
	} else {
	  not_critical("dg_col_get_raw: Blue reading failed.\n");
	  ret = false;
	}      
	
	if(!send_cmd(dvc, HTCS_ACT_MD)){
	  not_critical("dg_col_get_raw: Reenable active mode failed.\n");
	  goto back_to_def; //insisto...
	}
      }
      
    } else {
      
      not_critical("dg_col_get_raw: Device type must be HT_COLOR [%d]\n", HT_COLOR);
      return false; 
    }
  
    return ret;
    
  } else {
    
    not_critical("dg_col_get_raw: Digital interface not initialised\n");
    return false;
    
  }


 back_to_def:
  if (!send_cmd(dvc, HTCS_ACT_MD))
    not_critical("dg_col_get_white: Reenabling active mode failed.\n");
  return false;
  
} 

extern bool dg_irs_get_dir (DGDVC * dvc, uint8_t * dir, bool dc) {

  if (status.dg) {
    
    if (dvc->type == HT_IRS) {
    
      uint16_t taux[1];
      
      if(dc) {
	
	if(send_message(dvc, HTIS_DC_DIR, taux))
	  *dir = (uint8_t)taux[0];
	else {
	  not_critical("dg_irs_get_dir: DC Direction reading failed.\n");
	  return false;
	}
	
      } else {
	
	if(send_message(dvc, HTIS_AC_DIR, taux))
	  *dir = (uint8_t)taux[0];
	else {
	  not_critical("dg_irs_get_dir: AC Direction reading failed.\n");
	  return false;
	}
	
	
      }
      
    } else {
      
      not_critical ("dg_irs_get_dir: Device type must be HT_IRS [%d]\n", HT_IRS);
      return false;
      
    }

    return true;
    
  } else {
    
    not_critical("dg_irs_get_dir: Digital interface not initialised\n");
    return false;
    
  }


}

extern bool dg_irs_get_str (DGDVC * dvc, uint8_t * str, int num, bool dc) {

  if(status.dg) {
  
    if (dvc->type == HT_IRS) {
    
      uint16_t taux[1];
      
      if(num < 1 || num > MAX_IRS_STR) {
	not_critical("dg_irs_get_str: Strength number out of bounds.\n");
	return false;
      }
      
      if(dc) {
	
	if(send_message(dvc, HTIS_DCSTR_1 + (num - 1), taux))
	  *str = (uint8_t)taux[0];
	else {
	  not_critical("dg_irs_get_str: DC strength %d reading failed.\n", num);
	  return false;
	}
	
      } else {
	
	if(send_message(dvc, HTIS_ACSTR_1 + (num - 1), taux))
	  *str = (uint8_t)taux[0];
	else {
	  not_critical("dg_irs_get_str: AC strength %d reading failed.\n", num);
	  return false;
	}
	
      }
      
    } else {
      
      not_critical ("dg_irs_get_str: Device type must be HT_IRS [%d]\n", HT_IRS);
      return false;
      
    }
    
    return true;
    
  } else {
    
    not_critical("dg_irs_get_str: Digital interface not initialised\n");
    return false;
    
  }


}

extern bool dg_irs_get_dcavg (DGDVC * dvc, uint8_t * avg) {

  if(status.dg) {

    if (dvc->type == HT_IRS) {
    
      uint16_t taux[1];
      
      if(send_message(dvc, HTIS_DC_AVG, taux))
	*avg = (uint8_t)taux[0];
      else {
	not_critical("dg_irs_get_dcavg: DC Average reading failed.\n");
	return false;
      }
      
    } else {
      
      not_critical ("dg_irs_get_dir: Device type must be HT_IRS [%d]\n", HT_IRS);
      return false;
      
    }
    
    return true;
    
  } else {
    
    not_critical("dg_irs_get_dir: Digital interface not initialised\n");
    return false;
    
  }

  
}

extern bool dg_irs_get_allstr (DGDVC * dvc, uint8_t strt [], bool dc) {

  if(status.dg) {
    
    bool ret = true;
    
    if (dvc->type == HT_IRS) {
      
      uint16_t taux[1];
      int i;
      //strt = (uint8_t *)malloc(5*sizeof(uint8_t));
      
      if(dc) {
	
	for (i=0; i<MAX_IRS_STR; i++){
	  if(send_message(dvc, HTIS_DCSTR_1 + i, taux))
	    strt[i] = (uint8_t)taux[0];
	  else {
	    not_critical("dg_irs_get_allstr: DC strength %d reading failed.\n", i);
	    ret = false;
	  }
	}
      } else {
	
	for(i=0; i<MAX_IRS_STR; i++){
	  if(send_message(dvc, HTIS_ACSTR_1 + i, taux))
	    strt[i] = (uint8_t)taux[0];
	  else {
	    not_critical("dg_irs_get_allstr: AC strength %d reading failed.\n", i);
	    ret = false;
	  }
	}
      }
      
    } else {
      
      not_critical ("dg_irs_get_allstr: Device type must be HT_IRS [%d]\n", HT_IRS);
      return false;
      
    }
    
    return ret;
    
  } else {
    
    not_critical("dg_irs_get_allstr: Digital interface not initialised\n");
    return false;
    
  }
  
}

extern bool dg_get_state (DGDVC * dvc, uint8_t * state){

  if(status.dg) {
  
    if(dvc->type != DG_OTHER){

      uint16_t taux[1];
      cmdIdx cmdaux;
      
      if(dvc->type == LEGO_US)
	cmdaux = US_STATE;
      else
	cmdaux = HT_STATE;

      if(send_message(dvc, cmdaux, taux))
	*state = (uint8_t)taux[0];
      else {
	char * str = dvc->type == LEGO_US ? "LEGO_US" : dvc->type == HT_COMPASS ? "HT_COMPASS" : dvc->type == HT_ACCEL ? "HT_ACCEL" : dvc->type == HT_COLOR ? "HT_COLOR" : dvc->type == HT_IRS ? "HT_IRS" : ""; 
	not_critical("dg_get_state: State reading failed for %s.\n", str);
	return false;
      }

    } else {
      not_critical("dg_get_state: No unknown devices allowed.\n");
      return false;
    }
    
    return true;

  } else {
    
    not_critical("dg_get_state: Digital interface not initialised\n");
    return false;
    
  }

}

extern bool dg_us_get_dist (DGDVC * dvc, uint8_t * dist, int num) {

  if(status.dg) {
  
    if (dvc->type == LEGO_US) {
    
      uint16_t taux[1];

      if(num < 0 || num > MAX_US_DIST) {
	not_critical("dg_us_get_dist: Distance number out of bounds.\n");
	return false;
      }

      if(num != 0){
	
	if(!send_cmd(dvc, US_SIN_SHOT)){
	  not_critical("dg_us_get_dist: Error setting single shot mode\n");
	  goto back_to_def; //vaya a ser que...
	}
	
      }
      
      if(send_message(dvc, US_DIST_0 + num, taux))
	*dist = (uint8_t)taux[0];
      else {
	not_critical("dg_us_get_dist: Distance %d reading failed.\n", num);
	if(num != 0)
	  goto back_to_def;
	return false;
      }
      
      
    } else {
      
      not_critical ("dg_us_get_dist: Device type must be LEGO_US [%d]\n", LEGO_US);
      return false;
      
    }
    
    if(num !=0 ){
      if(!send_cmd(dvc, US_CONT_MES)){
	not_critical("dg_us_get_dist: Error setting back continious measurement mode.\n");
	goto back_to_def; //insisto...
      }
    }
    
    return true;
    
  } else {
    
    not_critical("dg_us_get_dist: Digital interface not initialised\n");
    return false;
    
  }

 back_to_def:
  if(!send_cmd(dvc, US_CONT_MES))
    not_critical("dg_us_get_dist: Error setting back continious measurement mode.\n");
  return false;

}

extern bool dg_us_get_alldist (DGDVC * dvc, uint8_t tdist []) {

  if(status.dg) {
    
    bool ret = true;
    
    if (dvc->type == LEGO_US) {
      
      uint16_t taux[1];
      int i;
      
      if(!send_cmd(dvc, US_SIN_SHOT)){
	not_critical("dg_us_get_alldist: Error setting single shot mode\n");
	goto back_to_def; //vaya a ser que...
      }
      
      for (i=0; i<=MAX_US_DIST; i++) {
	if(send_message(dvc, US_DIST_0 + i, taux))
	  tdist[i] = (uint8_t)taux[0];
	else {
	  not_critical("dg_us_get_alldist: Distance %d reading failed.\n", i);
	  ret = false;
	}
      }
      
    } else {
      
      not_critical ("dg_us_get_alldist: Device type must be LEGO_US [%d]\n", LEGO_US);
      return false;
      
    }
    
    if(!send_cmd(dvc, US_CONT_MES)){
      not_critical("dg_us_get_alldist: Error setting back continious measurement mode.\n");
      goto back_to_def; //insisto...
    }
    
    return ret;
    
  } else {
    
    not_critical("dg_us_get_alldist: Digital interface not initialised\n");
    return false;
    
  }


 back_to_def:
  if(!send_cmd(dvc, US_CONT_MES))
    not_critical("dg_us_get_alldist: Error setting back continious measurement mode.\n");
  return false;

}

extern bool dg_com_get_head (DGDVC * dvc, uint16_t * h1, uint16_t * h2) {

  if(status.dg) {
  
    bool ret = true;

    if(dvc->type == HT_COMPASS) {

      uint16_t taux[2];
    
      if(send_message(dvc, HTCM_2_DEG, taux))
	*h1 = (uint8_t)taux[0] * 2;
      else {
	not_critical("dg_com_get_head: Error reading two degree heading.\n");
	ret = false;
      }
      
      if(send_message(dvc, HTCM_1_ADD, taux))
	*h1 += (uint8_t)taux[0];
      else {
	not_critical("dg_com_get_head: Error reading one degree adder.\n");
	ret = false;
      }

      if(send_message(dvc, HTCM_HEAD, taux)) {
	*h2 = (uint8_t)taux[0];
	*h2 |= (uint8_t)taux[1] << BYTE_LEN;
      } else {
	not_critical("dg_com_get_head: Error reading word heading.\n");
	ret = false;
      }

      
    } else {
      
      not_critical ("dg_com_get_head: Device type must be HT_COMPASS [%d]\n", HT_COMPASS);
      return false;
      
    }

    return ret;

  } else {
    
    not_critical("dg_com_get_head: Digital interface not initialised\n");
    return false;
    
  }

}

extern bool dg_acc_get_axis (DGDVC * dvc, int * x, int * y, int * z) {

  if(status.dg) {

    bool ret = true;

    if(dvc->type == HT_ACCEL){
      
      uint16_t taux[1];
      
      if(send_message(dvc, HTAC_X_UP, taux))
	*x = taux[0] & 0x80 ? (taux[0] - 256) << 2 : taux[0] << 2;
      else {
	not_critical("dg_acc_get_axis: Error reading \"X\" upper byte.\n");
	ret = false;
      }
      
      if(send_message(dvc, HTAC_X_LW, taux))
	*x |= taux[0] & 0x03;
      else {
	not_critical("dg_acc_get_axis: Error reading \"X\" lower byte.\n");
	ret = false;
      }
      
      if(send_message(dvc, HTAC_Y_UP, taux))
	*y = taux[0] & 0x80 ? (taux[0] - 256) << 2 : taux[0] << 2;
      else {
	not_critical("dg_acc_get_axis: Error reading \"Y\" upper byte.\n");
	ret = false;
      }
      
      if(send_message(dvc, HTAC_Y_LW, taux))
	*y |= taux[0] & 0x03;
      else {
	not_critical("dg_acc_get_axis: Error reading \"Y\" lower byte.\n");
	ret = false;
      }
      
      if(send_message(dvc, HTAC_Z_UP, taux))
	*z = taux[0] & 0x80 ? (taux[0] - 256) << 2 : taux[0] << 2;
      else {
	not_critical("dg_acc_get_axis: Error reading \"Z\" upper byte.\n");
	ret = false;
      }
    
      if(send_message(dvc, HTAC_Z_LW, taux))
	*z |= taux[0] & 0x03;
      else {
	not_critical("dg_acc_get_axis: Error reading \"Z\" lower byte.\n");
	ret = false;
      }
      
    } else {
      
      not_critical ("dg_acc_get_axis: Device type must be HT_ACCEL [%d]\n", HT_ACCEL);
      return false;

    }
  
    return ret;
    
  } else {
    
    not_critical("dg_acc_get_axis: Digital interface not initialised\n");
    return false;
    
  }

}

static void raw_data_to_str (uint16_t raw [], char out[], int len) {
  
  int i;

  for (i=0; i < len; i++)
    out[i] = (char)raw[i];
    
  out[i] = '\0';

}

extern bool dg_get_info (DGDVC * dvc, char * info [], bool titled){

  if(status.dg) {
    
    bool ret = true;

    if(dvc->type != DG_OTHER){

      uint16_t taux[IN_MAX_LEN];
      int i;
      char auxstr[IN_MAX_LEN];
      msgIdx base = dvc->type == LEGO_US ? US_PR_VER : HT_VER_NUM;
      int offset = dvc->type == LEGO_US ? US_INF_MAX_OFFS : HT_INF_MAX_OFFS;
      char * type = dvc->type == LEGO_US ? "LEGO_US" : dvc->type == HT_COMPASS ? "HT_COMPASS" : dvc->type == HT_ACCEL ? "HT_ACCEL" : dvc->type == HT_COLOR ? "HT_COLOR" : dvc->type == HT_IRS ? "HT_IRS" : ""; 
    
      for (i = 0; i < offset; i++) {
	
	if(send_message(dvc, base + i, taux)){
	  
	  if(titled) {
	    info[i] = calloc(strlen(dvc->type == LEGO_US ? us_titles[i] : ht_titles[i]) + regs[base + i].len, sizeof(char));
	    strcpy(info[i], dvc->type == LEGO_US ? us_titles[i] : ht_titles[i]);
	  }else{
	    info[i] = calloc(regs[base + i].len, sizeof(char));
	    strcpy(info[i], "");
	  }
	  
	  if ((i > 2 && i < 6) || i == US_INF_MAX_OFFS-1) //only for LEGO_US
	    sprintf(auxstr, "0x%02x", (int)taux[0]);
	  else
	    raw_data_to_str(taux, auxstr, regs[base + i].len);
	  
	  strcat(info[i], auxstr);
	  
	} else {
	  
	  not_critical("dg_get_info: Failed to get %s%s\n", dvc->type == LEGO_US ? us_titles[i] : ht_titles[i], type);
	  ret = false;
	  
	}
	
      }
      
      for(; i<DG_INFO_TABLE_TAM; i++){
	info[i] = malloc(sizeof(char));
	strcpy(info[i], "");
      }
      
    } else {
      
      not_critical ("dg_get_info: No unknown devices allowed\n");
      return false;
      
    }

    return ret;

  } else {
    
    not_critical("dg_get_info: Digital interface not initialised\n");
    return false;
    
  }

}

/*_____________________The following functions are the only ones meant to be used by unknown devices [type = DG_OTHER]____________________*/


extern bool dg_transfer (DGDVC * dev, uint8_t data_out [], int len_out, bool rs, uint16_t data_in [], int len_in, bool word_read, int cse) {

  /*
    I2C Transaction generated:

    START >> WR_ADDR >> data_out[0] >> data_out[1] >> ... >> data_out[len_out-1] >> <RS/START> >> RD_ADDR << data_in[0] << data_in[1] << ... << data_in[len_in-1] << STOP + DELAY(cse us)

    rs >> boolean indicating if we need a repeated start condition [STOP + START] between write and read transactions, if not a restart [START] will be generated

    word_read >> boolean indicating if the lenght of every element in data_in will be a byte [8 bits] (false in this case), or a word [16 bits] (true then) 
   
  */

  if(status.dg)
  
    return (i2c_transfer(dev->dvc, data_out, len_out, rs, data_in, len_in, word_read, cse));
  
  else {
    
    not_critical("dg_transfer: Digital interface not initialised\n");
    return false;
    
  }

}

extern bool dg_write (DGDVC * dev, uint8_t data_out [], int len_out, int cse) {

  /*
    I2C Transaction generated:

    START >> WR_ADDR >> data_out[0] >> data_out[1] >> ... >> data_out[len_out-1] >> STOP + DELAY (cse us)

   */

  if(status.dg) 
    
    return (i2c_write(dev->dvc, data_out, len_out, cse));
  
  else {
    
    not_critical("dg_write: Digital interface not initialised\n");
    return false;
    
  }

}

extern bool dg_read (DGDVC * dev, uint16_t data_in [], int len_in, bool word_read, int cse) {

  /*
  
  I2C Transaction generated:

  START >> RD_ADDR << data_in[0] << data_out[1] << ... << data_out[len_out-1] >> STOP + DELAY (cse us)
   
  word_read >> boolean indicating if the lenght of every element in data_in will be a byte [8 bits] (false in this case), or a word [16 bits] (true then)  
  
  */

  if(status.dg)
    
    return (i2c_read(dev->dvc, data_in, len_in, word_read, cse));
  
  else {
    
    not_critical("dg_read: Digital interface not initialised\n");
    return false;
    
  }


}


static void not_critical (char* fmt, ...) {

  if (dg_log_lvl < LOG_LVL_ADV)
    return;

  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);
  
}

static void debug (char* fmt, ...) {
  
  if (dg_log_lvl < LOG_LVL_DBG)
    return;
  
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);

}

