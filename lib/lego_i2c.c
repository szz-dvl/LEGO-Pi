#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <time.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdbool.h>

#include "lego_i2c.h"

static volatile uint32_t *gpio_reg;
static uint32_t gpio_busy = 0; //bitfield to save busy pins
static int pr_debug = LOG_PRINT;

static void * map_peripheral(uint32_t, uint32_t);
static void set_resistor (int, uint32_t);
static bool byte_out (uint8_t byte, int sda, int scl, int thold);
static bool byte_in (uint8_t len, int sda, int scl, int thold, uint16_t * resp);
static void nack(int, I2C_DVC *, int);
static void repeated_start(int sda, int scl, int thold);
static void stop_condition(int sda, int scl, int thold);
static void free_pins(int, int);
static void debug (char* fmt, ...);


void set_resistor (int pin, uint32_t option) {

  struct timespec sleepTime;
  
  sleepTime.tv_sec = 0;
  sleepTime.tv_nsec = 1000 * RESISTOR_SLEEP_US;
  
  /* Set the GPPUD register with the desired resistor type */
  GPIO_GPPUD = option;
  /* Wait for control signal to be set up */
  nanosleep(&sleepTime, NULL);
  /* Clock the control signal for desired resistor */
  GPIO_GPPUDCLK0 = (0x1 << pin);
  /* Hold to set */
  nanosleep(&sleepTime, NULL);
  GPIO_GPPUD = 0;
  GPIO_GPPUDCLK0 = 0;
  
}

// Peripherals memory mapping
static void * map_peripheral(uint32_t base, uint32_t len) {
  int fd = open("/dev/mem", O_RDWR);
  void * vaddr;
  
  if (fd < 0) {
    printf("i2c-map: Failed to open /dev/mem: %m\n");
    return NULL;
  }
  vaddr = mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, base);
  if (vaddr == MAP_FAILED) {
    printf("i2c-map: Failed to map peripheral at 0x%08x: %m\n", base);
    return NULL;
  }
  close(fd);
  
  return vaddr;
}

extern bool i2c_set_loglvl (int log_lvl) {

  if (log_lvl == LOG_PRINT || log_lvl == LOG_QUIET){
    pr_debug = log_lvl;
    return true;
  } else {
    debug("init_i2c: log_lvl not understood, setting to QUIET mode...\n");
    pr_debug = LOG_QUIET;
    return false;
  }

}

extern bool i2c_init(int log_lvl) { 
  
  gpio_reg = (volatile unsigned *) map_peripheral(GPIO_BASE, BLOCK_SIZE);
  if(gpio_reg == NULL){
    printf("Error maping GPIOs\n");
    exit(EXIT_FAILURE);
  }

  if (log_lvl == LOG_PRINT || log_lvl == LOG_QUIET)
    pr_debug = log_lvl;
  else {
    debug("init_i2c: log_lvl not understood, setting to QUIET mode...\n");
    pr_debug = LOG_QUIET;
  }

  return true;
}

extern bool i2c_new_device ( I2C_DVC * dvc, uint8_t addr, int freq, int sda, int scl, uint32_t sda_pud, uint32_t scl_pud) {

  
  if (gpio_busy & (1<< sda) || gpio_busy & (1 << scl)) {

    printf("i2c_new_device: Demanded pins are busy.\n");
    return false;

  } else {

    gpio_busy |= (1 << sda); 
    gpio_busy |= (1 << scl);
  
  }

  dvc->addr = addr;
  dvc->thold = (1000000/freq)/4; //micro seconds
  dvc->sda = sda;
  dvc->scl = scl;

  if(sda_pud == GPPUD_DISABLE || sda_pud == GPPUD_PULLDOWN || sda_pud == GPPUD_PULLUP)
    
    set_resistor(sda, sda_pud);
  
  else {
    
    free_pins(sda,scl);
    printf("i2c_new_device: SDA Pull Up/Down resistor option not understood.\n");
    return false;
  }

  if(scl_pud == GPPUD_DISABLE || scl_pud == GPPUD_PULLDOWN || scl_pud == GPPUD_PULLUP)
    
    set_resistor(scl, scl_pud);
      
  else {
    
    free_pins(sda,scl);
    printf("i2c_new_device: SCL Pull Up/Down resistor option not understood.\n");
    return false;
  }
  
  
  INP(sda);
  OUTP(sda);
  INP(scl);
  OUTP(scl);

  HIGHP(sda);
  HIGHP(scl);
  //stop_condition(sda, scl,dvc->thold); //guarantee the bus is freed
  HOLD(dvc->thold*4);

  return true;
} 

static bool byte_out (uint8_t byte, int sda, int scl, int thold) {

 
  bool nack;
  int i;
  /*Byte sending*/

  for(i=7; i>=0; i--){

    if((byte >> i) & 1)
      HIGHP(sda);
    else
      LOWP(sda);
   
    HOLD(thold);
    HIGHP(scl);
    HOLD(thold);
    LOWP(scl);
  }
  
  /* ACK clock */
  LOWP(sda);
  HOLD(10);
  
  HIGHP(scl);
  HIGHP(sda);
  INP(sda);
  HOLD(thold*2);
  nack = STATE(sda);
  LOWP(scl);
  
  INP(sda);
  OUTP(sda);
  LOWP(sda);
 
  HOLD(thold*8); 
  return !nack;
    
}

static bool byte_in (uint8_t len, int sda, int scl, int thold, uint16_t * resp) { 
  
  uint8_t i; 
  bool nack;
  uint16_t inbyte = 0;
  INP(sda);
  HOLD(thold); //let the slave a little time to set the first bit properly..

  for (i=0; i<len; i++){
    inbyte *= 2;
    HIGHP(scl);
    HOLD(thold);
    if(STATE(sda))
      inbyte |= 0x01;
    LOWP(scl);
    //HOLD(thold);
    i < len-1 ? HOLD(thold) : HOLD(thold*2); //this last clock is enlarged to allow the slave to release the SDA line acknowledging the sended message.
  }
   
  /*ACK management*/
  HIGHP(scl);
  nack = !STATE(sda);  
  INP(sda);
  OUTP(sda);
  LOWP(sda);
  HOLD(thold*2);
  LOWP(scl);

  if(!nack)
    *resp = inbyte;
    
  HOLD(thold *8);

  return !nack;

} 


static void nack(int byte, I2C_DVC * dvc, int dir) {
  
  HOLD(1000);

  if (byte == 0 && dir)
    debug("Received NACK sending WRITE_ADDR: 0x%02x\n", dvc->addr & 0xFE);
  else if (byte == -1)
    debug("Received NACK sending READ_ADDR: 0x%02x\n", dvc->addr | 0x01);
  else
    debug("Received NACK %s byte: %d\n",  dir ? "sending" : "receiving" , byte);
   
  stop_condition(dvc->sda, dvc->scl, dvc->thold); 

  //exit(EXIT_FAILURE);

}

extern void i2c_shutdown (I2C_DVC * dvc){

  
  stop_condition(dvc->sda,dvc->scl,dvc->thold);
  
  free_pins(dvc->sda,dvc->scl);
  
  //exit(EXIT_SUCCESS);


}

static void free_pins(int sda, int scl) {

  gpio_busy &= ~(1 << sda); 
  gpio_busy &= ~(1 << scl);

}

static void repeated_start(int sda, int scl, int thold){
  
  //HOLD(thold*4);
  HIGHP(scl);
  HOLD(thold*2);
  HIGHP(sda);
  HOLD(thold);
  LOWP(scl);
  HOLD(thold);
  HIGHP(scl);
  HOLD(thold);
  LOWP(sda);
  HOLD(thold*2);
  LOWP(scl);
  HOLD(thold*8);
      
}
static void stop_condition(int sda, int scl, int thold) {
  
  /*LOWP(scl);
  HOLD(thold);
  HIGHP(sda);
  HOLD(thold);*/
  HIGHP(sda);
  HIGHP(scl);
  HOLD(thold);
  LOWP(sda);
  HOLD(thold*2);
  LOWP(scl);
  HOLD(thold); //added
  HIGHP(sda);
  HIGHP(scl);
  HOLD(thold); //added
  

}

extern bool i2c_transfer (I2C_DVC * dvc, uint8_t * data_out, int len_out, bool rs, uint16_t * data_in, int len_in, bool word_read, int cse) {

  /* 
     data_out  >> array of bytes of length "len_out" to transfer to the slave device.
     rs        >> boolean value indicating if we need a repeated start condition between write and read.
     data_in   >> array of words of length "len_in" where the response of the slave will be put, if "len_in" equals to 0 no response expected.
     word_read >> responses from slave are word sized if true, byte sized if false.
     cse       >> int indicating if we need to wait at the end of the transaction, emulating a clock streching behaviour, avoiding INP(scl).
   
     assert: The actual transaction will consist in:
            START >> WR_ADDR >> data_out[0] >> data_out[1] >> ... >> data_out[len_out-1] >> <RS> >> RD_ADDR << data_in[0] << data_in[1] << ... << data_in[len_in-1] << STOP
  */

  if (!(gpio_busy & (1<< dvc->sda)) || !(gpio_busy & (1 << dvc->scl))) {
    printf("i2c_transfer: Device not recognized or not properly initialized.\n");
    return false;
  }

  if(len_out <= 0) {
    printf("i2c_transfer: Data_out length <= 0 nothing to do...\n");
    return false;
  }
  if(len_in < 0) {
    printf("i2c_transfer: Data_in length < 0, not understood\n");
    return false;
  }
  
  int i;
  uint8_t to_send [len_out+1];
  uint8_t rd_addr = dvc->addr | 0x01;//read addres of the slave
  uint8_t rlen = word_read ? WORD_LEN : BYTE_LEN;

  to_send[0] = dvc->addr & 0xFE; // write addres of the slave
  
  for (i=1; i<=len_out; i++)
    to_send[i] = data_out[i-1];

  /*  START condition */ 
  //HOLD(dvc->thold);
  LOWP(dvc->sda);
  HOLD(dvc->thold);
  LOWP(dvc->scl);
  
  /* END START condition */
  
  for (i=0; i<=len_out; i++){
    if(!byte_out(to_send[i], dvc->sda, dvc->scl, dvc->thold)){
      nack(i, dvc, 1);
      return false;
    }
  }
  
  if(len_in > 0){
    if(rs)
      /* repeated start: stop + start */
      repeated_start(dvc->sda, dvc->scl, dvc->thold);
    else {
      /* re-start */
      HIGHP(dvc->sda);
      HIGHP(dvc->scl);
      HOLD(dvc->thold*2);
      LOWP(dvc->sda);
      HOLD(dvc->thold);
      LOWP(dvc->scl);
    }
 
    if(!byte_out(rd_addr, dvc->sda, dvc->scl, dvc->thold)){
      nack(-1, dvc, 1);
      return false;
      }
    for(i=0; i<len_in; i++){
      if(!byte_in(rlen, dvc->sda, dvc->scl, dvc->thold, &data_in[i])){
	nack(i, dvc, 0);
	return false;
      }
    }
  }

  stop_condition(dvc->sda, dvc->scl, dvc->thold);
 
  if(cse)
    HOLD(cse);
     
  return true;
}

extern bool i2c_read (I2C_DVC * dvc, uint16_t * data_in, int len_in, bool word_read, int cse) {

  /*
    READ, without a previous WRITE command (Read address sent in first place) [to support unknown devices...<untested>]

    >> START >> RD_ADDR << data_in[0] << data_out[1] << ... << data_out[len_out-1] >> STOP

   */

  if (!(gpio_busy & (1<< dvc->sda)) || !(gpio_busy & (1 << dvc->scl))) {
    printf("i2c_read: Device not recognized or not properly initialized.\n");
    return false;
  }

  if(len_in <= 0) {
    printf("i2c_read: Data_in length < 0, nothing to do...\n");
    return false;
  }
  
  int i;
  uint8_t rlen = word_read ? WORD_LEN : BYTE_LEN;

 
  /* START condition */
  
  LOWP(dvc->sda);
  HOLD(dvc->thold);
  LOWP(dvc->scl);

  /* END START condition */
  
  if(!byte_out(dvc->addr | 0x01, dvc->sda, dvc->scl, dvc->thold)){
    nack(-1, dvc, 1);
    return false;
  }
      
  for(i=0; i<len_in; i++) {
    if(!byte_in(rlen, dvc->sda, dvc->scl, dvc->thold, &data_in[i])){
      nack(i, dvc, 0);
      return false;
    }
  }
  
  stop_condition(dvc->sda, dvc->scl, dvc->thold);
  
  if(cse)
    HOLD(cse);
  
  return true;

}

extern bool i2c_write (I2C_DVC * dvc, uint8_t * data_out, int len_out, int cse){

  /* 
     WRITE without expecting response (used for i2c commands):

     START >> WR_ADDR >> data_out[0] >> data_out[1] >> ... >> data_out[len_out-1] >> STOP

   */
  
  return (i2c_transfer (dvc, data_out, len_out, false, NULL, 0, false, cse));

}

static void debug (char* fmt, ...) {
  
  if (pr_debug == LOG_QUIET)
    return;
  
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);

}

