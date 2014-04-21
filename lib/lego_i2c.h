#include <stdint.h>
#include <stdbool.h>

#define LOG_PRINT               1
#define LOG_QUIET               0

#define GPIO_FSEL0	        (0x00/4)
#define GPIO_SET0	        (0x1c/4)
#define GPIO_CLR0	        (0x28/4)
#define GPIO_BASE	        0x20200000
#define BLOCK_SIZE	        (4*1024)

#define IN_LVL_OFFSET	        13

#define BYTE_LEN	        8
#define WORD_LEN	        16

/* FROM: http://elinux.org/RPi_Low-level_peripherals#C_2 */
#define INP(g)	        *(gpio_reg+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUTP(g)		*(gpio_reg+((g)/10)) |=  (1<<(((g)%10)*3))

//ONLY to be used in OUTPUT MODE
#define LOWP(pin)                 (gpio_reg[GPIO_CLR0] = 1 << pin)
#define HIGHP(pin)                (gpio_reg[GPIO_SET0] = 1 << pin)

//ONLY to be usen in INPUT MODE
#define STATE(pin)               ((gpio_reg[IN_LVL_OFFSET] & (1 << pin)) != 0)

#define GPPUD_DISABLE           0x0      /**< Disables the resistor */
#define GPPUD_PULLDOWN          0x1      /**< Enables a pulldown resistor */
#define GPPUD_PULLUP            0x2      /**< Enables a pullup resistor */

#define GPPUD_OFFSET            0x000094
#define GPPUDCLK0_OFFSET        0x000098

#define GPIO_GPPUD              (gpio_reg[GPPUD_OFFSET])
#define GPIO_GPPUDCLK0          (gpio_reg[GPPUDCLK0_OFFSET])

#define RESISTOR_SLEEP_US       1

#define CLK_ID                  CLOCK_PROCESS_CPUTIME_ID

typedef struct timespec TSPEC;

#define HOLD(t)     nanosleep((TSPEC*)&(TSPEC){0, t*1000}, NULL)

struct i2c_device {
  int sda;
  int scl;
  double thold;
  uint8_t addr; //stands for write addres of the device, only 7 bit + 0 write bit addresses supported.
};

typedef struct i2c_device I2C_DVC;

extern void init_i2c(int log_lvl);
extern bool i2c_set_loglvl (int log_lvl);
extern bool i2c_new_device (I2C_DVC * dvc, uint8_t addr, int freq, int sda, int scl, uint32_t sda_pud, uint32_t scl_pud);
extern bool i2c_transfer (I2C_DVC * dvc, uint8_t * data_out, int len_out, bool rs, uint16_t * data_in, int len_in, bool word_read, int cse);
extern bool i2c_read (I2C_DVC * dvc, uint16_t * data_in, int len_in, bool word_read, int cse); //untested
extern bool i2c_write (I2C_DVC * dvc, uint8_t * data_out, int len_out, int cse);
extern void i2c_shutdown (I2C_DVC * dvc);
