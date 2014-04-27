#include "lego_shared.h"
#include <lego/lego_i2c.h>

#define LEGO_ADDR        0x02 
#define LEGO_FREQ        9600 //baudios/Hz
#define SDA_0            27
#define SCL_0            17
#define SDA_1            14
#define SCL_1            15

#define CMD_REG          0x41
#define MAX_PORT_DG      1
#define MIN_PORT_DG      0

#define RAW_LEN          2
#define US_DWAIT         4000
#define US_IWAIT         3000
#define US_CWAIT         19000 //avanti
#define HT_DLY           0
#define DEF_DELAY        5000

#define IN_MAX_LEN       8
#define MAX_IRS_STR      5 
#define MAX_US_DIST      7 

#define US_INF_MAX_OFFS  8 
#define HT_INF_MAX_OFFS  3

#define US_CMD_RMIN      0
#define US_CMD_RMAX      4

#define HTCS_CMD_RMIN    5
#define HTCS_CMD_RMAX    10

#define HTCM_CMD_RMIN    11
#define HTCM_CMD_RMAX    12

#define HTIS_CMD_RMIN    13
#define HTIS_CMD_RMAX    14

#define IS_US_CMD(c)     (c >= US_CMD_RMIN && c <= US_CMD_RMAX)
#define IS_HTCS_CMD(c)   (c >= HTCS_CMD_RMIN && c <= HTCS_CMD_RMAX)
#define IS_HTCM_CMD(c)   (c >= HTCM_CMD_RMIN && c <= HTCM_CMD_RMAX)
#define IS_HTIS_CMD(c)   (c >= HTIS_CMD_RMIN && c <= HTIS_CMD_RMAX)

#define IS_HITECH(x)     (x>1)

//Meant to be used by the developer
#define US_DIST_TABLE_TAM 8
#define IRS_STR_TABLE_TAM 5  
#define DG_INFO_TABLE_TAM 9

typedef enum {
  
  DG_OTHER,
  LEGO_US,
  HT_COMPASS,
  HT_ACCEL,
  HT_COLOR,
  HT_IRS

} dgType;

struct digital_device {
  
  dgType type;
  int port;
  int vers;
  I2C_DVC * dvc;

};
typedef struct digital_device DGDVC;

struct sensor_command {

  uint8_t reg;
  uint8_t val;
  int delay;
  int minvrs;

};
typedef struct sensor_command CMD;

struct sensor_msg {
  
  uint8_t base;
  int len;
  int delay;
  int minvrs; 

};
typedef struct sensor_msg MSG;

typedef enum {

  US_OFF,
  US_SIN_SHOT,
  US_CONT_MES,
  US_EVENT_CAP,
  US_WARM_RESET,
  HTCS_CAL_WHITE,
  HTCS_ACT_MD,
  HTCS_PAS_MD,
  HTCS_RAW_MD,
  HTCS_50_MD,
  HTCS_60_MD,
  HTCM_MES_MD,
  HTCM_CAL_MD,
  HTIS_DSP_12,
  HTIS_DSP_6,

} cmdIdx;

typedef enum {

  HT_VER_NUM,
  HT_MANFAC,
  HT_SEN_TYPE,
  HT_STATE,
  HTCS_COL_NUM,
  HTCS_RED,
  HTCS_GREEN,
  HTCS_BLUE,
  HTCS_RAW_RED,
  HTCS_RAW_GREEN,
  HTCS_RAW_BLUE,
  HTCS_COL_IDX,
  HTCS_NRM_RED,
  HTCS_NRM_GREEN,
  HTCS_NRM_BLUE,
  HTCS2_WHITE,
  HTCS2_COL_IDX,
  HTCS2_NRM_RED,
  HTCS2_NRM_GREEN,
  HTCS2_NRM_BLUE,
  HTCS2_RAW_RED,
  HTCS2_RAW_GREEN,
  HTCS2_RAW_BLUE,
  HTCS2_RAW_WHITE,
  HTIS_DC_DIR,
  HTIS_DCSTR_1,
  HTIS_DCSTR_2,
  HTIS_DCSTR_3,
  HTIS_DCSTR_4,
  HTIS_DCSTR_5,
  HTIS_DC_AVG,
  HTIS_AC_DIR,
  HTIS_ACSTR_1,
  HTIS_ACSTR_2,
  HTIS_ACSTR_3,
  HTIS_ACSTR_4,
  HTIS_ACSTR_5,
  HTAC_X_UP,
  HTAC_Y_UP,
  HTAC_Z_UP,
  HTAC_X_LW,
  HTAC_Y_LW,
  HTAC_Z_LW,
  HTCM_2_DEG,
  HTCM_1_ADD,
  HTCM_HEAD,
  US_PR_VER,
  US_PR_ID,
  US_SEN_TYPE,
  US_FAC_ZERO,
  US_FAC_SC_FAC,
  US_FAC_SC_DIV,
  US_MES_UNITS,
  US_MES_INT,
  US_STATE,
  US_DIST_0,
  US_DIST_1,
  US_DIST_2,
  US_DIST_3,
  US_DIST_4,
  US_DIST_5,
  US_DIST_6,
  US_DIST_7

}msgIdx;

//{0x50, 1, IWAIT, "Actual Zero: "}, 
//{0x51, 1, IWAIT, "Actual Scale Factor: "},
//{0x52, 1, IWAIT, "Actual Scale Divisor: "},

extern bool dg_init(int retry);
extern bool dg_new (DGDVC * dev, dgType type, int port);
extern bool dg_new_unknown (DGDVC * dev, uint8_t addr, int freq, int port);
extern void dg_shutdown ();
extern bool dg_send_cmd (DGDVC * dev, cmdIdx cmd);
extern bool dg_get_state (DGDVC * dvc, uint8_t * state);
extern bool dg_col_get_rgb (DGDVC * dvc, uint8_t * red, uint8_t * green, uint8_t * blue);
extern bool dg_col_get_norm (DGDVC * dvc, uint8_t * red, uint8_t * green, uint8_t * blue);
extern bool dg_col_get_index (DGDVC * dvc, uint8_t * idx);
extern bool dg_col_get_number (DGDVC * dvc, uint8_t * num);
extern bool dg_col_get_white (DGDVC * dvc, uint16_t * white, bool raw, bool passive);
extern bool dg_col_get_raw (DGDVC * dvc, uint16_t * red, uint16_t * green, uint16_t * blue, bool passive);
extern bool dg_irs_get_dir (DGDVC * dvc, uint8_t * dir, bool dc);
extern bool dg_irs_get_str (DGDVC * dvc, uint8_t * str, int num, bool dc);
extern bool dg_irs_get_dcavg (DGDVC * dvc, uint8_t * avg);
extern bool dg_irs_get_allstr (DGDVC * dvc, uint8_t * strt, bool dc);
extern bool dg_us_get_dist (DGDVC * dvc, uint8_t * dist, int num);
extern bool dg_us_get_alldist (DGDVC * dvc, uint8_t tdist []);
extern bool dg_com_get_head (DGDVC * dvc, uint16_t * h1, uint16_t * h2);
extern bool dg_acc_get_axis (DGDVC * dvc, int * x, int * y, int * z);
extern bool dg_get_info (DGDVC * dvc, char * info [], bool titled);
extern bool dg_transfer (DGDVC * dev, uint8_t data_out [], int len_out, bool rs, uint16_t data_in [], int len_in, bool word_read, int cse);
extern bool dg_write (DGDVC * dev, uint8_t data_out [], int len_out, int cse);
extern bool dg_read (DGDVC * dev, uint16_t data_in [], int len_in, bool word_read, int cse);
