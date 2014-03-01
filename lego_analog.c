#include "lego_analog.h"

//#define NEVER_HAPPENS   4

//extern int an_fd;

static int SPIreceive (int, uint8_t [], int);

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

/*   printf("Received :\n") ;
   for (i = 0; i < LEN; i++)
		printf("%u, ", resp[i]) ;

printf("\n") ;*/

   return retval ;
}

extern void analog_setup() {
  
  if (!(an_fd = wiringPiSPISetup(CS, SPI_CLK)))
    fatal("analog_setup: Error setting up SPI interface, ERRNO: %d\n", errno);

}

extern int new_analog ( ANDVC* dvc, int port, int type ) {

  int ret = OK;

  if (port < MIN_PORT || port > MAX_PORT){
    not_critical("new_analog: port must be between %d - %d\n", MIN_PORT, MAX_PORT);
    ret = FAIL;
  } else {   
    dvc->port = port;
    dvc->lpin = port == 0 ? L_PORT0 : port == 1 ? L_PORT1 : port == 2 ? L_PORT2 : port == 3 ? L_PORT3 : 4;
  }
  
  if(ret) {
    if( type != LIGHT && type != PUSH ) {
      not_critical("new_analog: type must be %d (LIGHT) or %d (PUSH)\n", LIGHT, PUSH);
      ret = FAIL;
    } else 
      dvc->type = type;
  }

  return ret;

}

extern int analog_light_on (ANDVC* dvc){
  
  int ret = OK;
  
  if (dvc->type != LIGHT) {
    not_critical("analog_light_on: Device type must be %d (LIGHT)\n", LIGHT);
    ret = FAIL;
  } else 
    digitalWrite(dvc->lpin, HIGH);

  return ret;
}

extern int analog_light_off (ANDVC* dvc){

  int ret = OK;
  
  if (dvc->type != LIGHT) {
    not_critical("analog_light_off: Device type must be %d (LIGHT)\n", LIGHT);
    ret = FAIL;
  } else 
    digitalWrite(dvc->lpin, LOW);
  
  return ret;
}

extern int analog_read_int (ANDVC * dvc) { /* READ analog value (integer), max avalue = 1023 */

  uint8_t data [LEN] = { 0x00, 0x00, 0x00 } ;
  int ioctl, retval = OK, res;
    
  if (!(ioctl = SPIreceive(an_fd, data, dvc->port))) {
    
    not_critical("analog_read_int: Error comunicating to SPI device\n") ;
    retval = FAIL ;   
  }
  
  if(retval) {
    res = data[2];
    res |= ((data[1] & 0x07) << 8);
    return res;
    
  } else
    return retval;
  
}

extern double analog_read_voltage (ANDVC * dvc) { /* READ analog value (float), VOLTATGE */

  uint8_t data [LEN] = { 0x00, 0x00, 0x00 } ;
  int ioctl, retval = OK, res;
  
  if (!(ioctl = SPIreceive(an_fd, data, dvc->port))) {
    
   not_critical("analog_read_voltage: Error comunicating to SPI device\n");
   retval = FAIL;
   
  }
  
  if(retval) {
    res = data[2];
    res |= ((data[1] & 0x07) << 8);
    return ((double)((double)res/(double)MAX_VAL) * VREF);
    
  } else
    return retval;
  
}

extern bool analog_pushed (ANDVC * dvc) {
  
  double act_v;
  
  if (dvc->type != PUSH)
    not_critical("analog_pushed: Device type must be %d (PUSH)\n", PUSH);
  else {
    act_v = analog_read_voltage(dvc);
    if(act_v < (double)(VREF/2))
      return true;
    else
      return false;
  }
  
  return FAIL;
}
  
extern void analog_shutdown () {
  
  if(an_fd > 0)
    close(an_fd);

}





/*
int main (int argc, char *argv []) {
  
  //	int retval = OK, fd, ioctl ;
  //int len = ;

  int fd ;
  uint16_t rint ;
  double rfloat ;
  
  //	uint8_t data [LEN] = { 0x00, 0x00, 0x00 } ;
  
  int canal = argc > 1 ? atoi(argv[1]) : 0 ;
  //uint8_t change = argc > 2 ? atoi(argv[2]) : 0 ;
  
  	wiringPiSetupGpio() ;
	pinMode(8, OUTPUT) ;
	digitalWrite(8, HIGH);
	digitalWrite(8, HIGH);
  //	to_binary(TO_READ, data);
  
  fd = analog_setup() ;
  
  rint = analog_read_int(canal, fd) ;
  sleep(1);
  rfloat = analog_read_float(canal, fd) ;
  printf("Received: rint: %u, rfloat: %.2f\n", rint, rfloat) ;
  
  //	transfer(fd);
  
  	if (!(ioctl = wiringPiSPIDataRW(CHANN, data, LEN))) {
	
	printf("ERROR %d: Comunicating to SPI device\n", ioctl);
	retval = FAIL;
	
	} else {
	
	data[LEN] = '\0';
	printf("Received from A/D: %c, returned = %d\n", print_data(data), ioctl);
	
	}
	
	flush_data(data);
	
	if (!(ioctl = read(fd, data, LEN))) {
	
	printf("ERROR %d: Comunicating to SPI device\n", ioctl);
	retval = FAIL;
	
	} else {
	
	data[LEN] = '\0';
	printf("Received from A/D: %c, returned = %d\n", print_data(data), ioctl);
	
	}
	
	to_binary(TO_READ, data);
  //flush_data(data, LEN);
  
  	if (retval && !(ioctl = SPIreceive(fd, data, canal))) {
	
	printf("ERROR %d: Comunicating to SPI device\n", ioctl) ;
	retval = FAIL ;
	
	} else {
	
	//data[LEN] = '\0';
	printf("Received from A/D: ( %d ):\n", ioctl) ;
	print_data(data, LEN) ;
	
	}
	
	if(retval)
	join_bytes(data[1], data[2]) ; 
  
  return OK;
}
_______________________________OLD____________________________-


static void flush_data (uint8_t data[], int len) {

  int i;
  
  for ( i = 0; i < len; i++)
    data[i] = 0;
  
}

static char print_data (uint8_t data[], int len) {
  
  int i;
  
  for ( i = 0; i < len; i++)
    printf("%u, ", data[i]);
  
  printf("\n");
  
  return '\0';
 
}

static void to_binary (uint16_t byte, unsigned char * buffer, int len) {
  
  int i;
  
  for( i = 0; i < len; i++ ) {
    //printf("bit #%1d: %1u\n",i,(byte & (1 << i)));
    if((byte & (1 << i)) != 0)
      buffer[(len-1)-i] = '1';
    else
      buffer[(len-1)-i] = '0';
  }
  
  buffer[len] = '\0';
}

static uint16_t join_bytes (uint8_t msb, uint8_t lsb) {
  
  //uint16_t aux;
  uint16_t res ;
  //int i ;
  res = lsb ;
  
//   DEBUG: binaries to print 
  unsigned char lsb_bin[9] ;
  unsigned char msb_bin[9] ;
  unsigned char bin[13] ;
  
  to_binary(lsb, lsb_bin, 8) ;
  to_binary(msb, msb_bin, 8) ;
  
  	for (i = 0; i < 4; i++)
	res |= ((msb & (1 << i)) << (8 + i)) ; 
  
  Parece ser que, creo que por INL, la resolucion es de solo 10 bits 
  
  res |= (msb << 8);
  
  to_binary(res, bin, 12) ;

  printf("Valor llegit d'A/D: %u, BINARY: %s  MSB_bin: %s, LSB_bin: %s\n", res, bin, msb_bin, lsb_bin) ;
  
  return res ;

}

___________________________________________TEST________________________-


static void to_binary (uint16_t byte, unsigned char * buffer, int len) {
  
  int i;
  
  for( i = 0; i < len; i++ ) {
    //printf("bit #%1d: %1u\n",i,(byte & (1 << i)));
    if((byte & (1 << i)) != 0)
      buffer[(len-1)-i] = '1';
    else
      buffer[(len-1)-i] = '0';
  }
  
  buffer[len] = '\0';
}


*/
