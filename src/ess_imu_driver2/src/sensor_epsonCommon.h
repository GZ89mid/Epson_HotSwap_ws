//==============================================================================
//
//  sensor_epsonCommon.h - Epson IMU sensor specific definitions common
//                      for all IMU models
//
//
//  THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  NONINFRINGEMENT, SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A
//  PARTICULAR PURPOSE. IN NO EVENT SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE
//  OR CLAIM, ARISING FROM OR IN CONNECTION WITH THE SOFTWARE OR THE USE OF THE
//  SOFTWARE.
//
//==============================================================================
#pragma once

#include <stdbool.h>

#ifdef SPI
#include "sensor_epsonSpi.h"
#else
#include "sensor_epsonUart.h"
#endif  // #ifdef SPI

#ifndef TRUE
#define TRUE (true)
#endif

#ifndef FALSE
#define FALSE (false)
#endif

#define BIT0 (1)
#define BIT1 (1 << 1)
#define BIT2 (1 << 2)
#define BIT3 (1 << 3)
#define BIT4 (1 << 4)
#define BIT5 (1 << 5)
#define BIT6 (1 << 6)
#define BIT7 (1 << 7)
#define BIT8 (1 << 8)
#define BIT9 (1 << 9)
#define BIT10 (1 << 10)
#define BIT11 (1 << 11)
#define BIT12 (1 << 12)
#define BIT13 (1 << 13)
#define BIT14 (1 << 14)
#define BIT15 (1 << 15)

#define EPSON_RESET_LOW_WIDTH_MS (100)
#define EPSON_RESET_DELAY_MS (3000)  // G570PR20 has max delay of 3000 msecs

// Conversion constants
#define DEG2RAD (3.14159 / 180.0)   // Degrees to radians
#define RAD2DEG (180.0 / 3.14159)   // Radians to degrees
#define MG2MPS2 (9.80665 / 1000.0)  // milli-G to meters/sec^2
#define MPS22MG (1000.0 / 9.80665)  // meters/sec^2 to milli-G

/*                   -- Commands --
- ADDR_ address byte of transfer to select the register to access

- All accesses are 16 bit transfers
- For SPI IF:
    - For SPI write accesses
        - 8-bit address with msb=1b (even or odd) + write byte
        - No response
    - For SPI read accesses
        - 8-bit address with msb=0b(even only) + dummy byte
        - Response is transferred on MOSI on next SPI access
        - Return value is 16-bit read data (high byte + low byte)
- For UART IF:
        - For UART write accesses
            - 8-bit address with msb=1b(even or odd) + write byte + delimiter
            - No response
        - For UART read accesses
            - 8-bit address with msb=0b(even only) + dummy byte + delimiter
            - Response is transferred immediately
            - Return value
                - Address + 16-bit read data (high byte + low byte) + delimiter

- NOTE: Register Address Maps that depend on the WINDOW_ID (page)
- NOTE: Not all registers listed are supported by all device models */

// WINDOW_ID 0
#define ADDR_MODE_CTRL_LO 0x02  // MODE_CTRL Byte0 (W0)
#define ADDR_MODE_CTRL_HI 0x03  // MODE_CTRL Byte1 (W0)
#define ADDR_DIAG_STAT 0x04     // DIAG_STAT Byte0 (W0)
#define ADDR_FLAG 0x06          // FLAG(ND/EA) (W0)
#define ADDR_GPIO 0x08          // GPIO  (W0)
#define ADDR_COUNT 0x0A         // COUNT (W0)
#define ADDR_RANGE_OVER 0x0C    // RANGE_OVER (W0)
#define ADDR_TEMP_HIGH 0x0E     // TEMPC HIGH (W0)
#define ADDR_TEMP_LOW 0x10      // TEMPC LOW  (W0)
#define ADDR_XGYRO_HIGH 0x12    // XGYRO HIGH (W0)
#define ADDR_XGYRO_LOW 0x14     // XGYRO LOW  (W0)
#define ADDR_YGYRO_HIGH 0x16    // YGYRO HIGH (W0)
#define ADDR_YGYRO_LOW 0x18     // YGYRO LOW  (W0)
#define ADDR_ZGYRO_HIGH 0x1A    // ZGYRO HIGH (W0)
#define ADDR_ZGYRO_LOW 0x1C     // ZGYRO LOW  (W0)
#define ADDR_XACCL_HIGH 0x1E    // XACCL HIGH (W0)
#define ADDR_XACCL_LOW 0x20     // XACCL LOW  (W0)
#define ADDR_YACCL_HIGH 0x22    // YACCL HIGH (W0)
#define ADDR_YACCL_LOW 0x24     // YACCL LOW  (W0)
#define ADDR_ZACCL_HIGH 0x26    // ZACCL HIGH (W0)
#define ADDR_ZACCL_LOW 0x28     // ZACCL LOW  (W0)

#define ADDR_RT_DIAG 0x2B  // RT_DIAG LOW (W0)

#define ADDR_ID 0x4C         // ID LOW  (W0)
#define ADDR_QTN0_HIGH 0x50  // QTN0 HIGH (W0)
#define ADDR_QTN0_LOW 0x52   // QTN0 LOW  (W0)
#define ADDR_QTN1_HIGH 0x54  // QTN1 HIGH (W0)
#define ADDR_QTN1_LOW 0x56   // QTN1 LOW  (W0)
#define ADDR_QTN2_HIGH 0x58  // QTN2 HIGH (W0)
#define ADDR_QTN2_LOW 0x5A   // QTN2 LOW  (W0)
#define ADDR_QTN3_HIGH 0x5C  // QTN3 HIGH (W0)
#define ADDR_QTN3_LOW 0x5E   // QTN3 LOW  (W0)

#define ADDR_ANG1_HIGH 0x64  // ANG1 HIGH (W0)
#define ADDR_ANG1_LOW 0x66   // ANG1 LOW  (W0)
#define ADDR_ANG2_HIGH 0x68  // ANG2 HIGH (W0)
#define ADDR_ANG2_LOW 0x6A   // ANG2 LOW  (W0)
#define ADDR_ANG3_HIGH 0x6C  // ANG3 HIGH (W0)
#define ADDR_ANG3_LOW 0x6E   // ANG3 LOW  (W0)

#define ADDR_XDLTA_HIGH 0x64  // XDLTA HIGH (W0)
#define ADDR_XDLTA_LOW 0x66   // XDLTA LOW  (W0)
#define ADDR_YDLTA_HIGH 0x68  // YDLTA HIGH (W0)
#define ADDR_YDLTA_LOW 0x6A   // YDLTA LOW  (W0)
#define ADDR_ZDLTA_HIGH 0x6C  // ZDLTA HIGH (W0)
#define ADDR_ZDLTA_LOW 0x6E   // ZDLTA LOW  (W0)
#define ADDR_XDLTV_HIGH 0x70  // XDLTV HIGH (W0)
#define ADDR_XDLTV_LOW 0x72   // XDLTV LOW  (W0)
#define ADDR_YDLTV_HIGH 0x74  // YDLTV HIGH (W0)
#define ADDR_YDLTV_LOW 0x76   // YDLTV LOW  (W0)
#define ADDR_ZDLTV_HIGH 0x78  // ZDLTV HIGH (W0)
#define ADDR_ZDLTV_LOW 0x7A   // ZDLTV LOW  (W0)

// WINDOW_ID 1
#define ADDR_SIG_CTRL_LO 0x00     // SIG_CTRL Byte0 (W1)
#define ADDR_SIG_CTRL_HI 0x01     // SIG_CTRL Byte1 (W1)
#define ADDR_MSC_CTRL_LO 0x02     // MSC_CTRL Byte0 (W1)
#define ADDR_MSC_CTRL_HI 0x03     // MSC_CTRL Byte1 (W1)
#define ADDR_SMPL_CTRL_LO 0x04    // SMPL_CTRL Byte0 (W1)
#define ADDR_SMPL_CTRL_HI 0x05    // SMPL_CTRL Byte1 (W1)
#define ADDR_FILTER_CTRL_LO 0x06  // FILTER_CTRL Byte0 (W1)
#define ADDR_FILTER_CTRL_HI 0x07  // FILTER_CTRL Byte1 (W1)
#define ADDR_UART_CTRL_LO 0x08    // UART_CTRL Byte0 (W1)
#define ADDR_UART_CTRL_HI 0x09    // UART_CTRL Byte1 (W1)
#define ADDR_GLOB_CMD_LO 0x0A     // GLOB_CMD Byte0 (W1)
#define ADDR_GLOB_CMD_HI 0x0B     // GLOB_CMD Byte1 (W1)
#define ADDR_BURST_CTRL1_LO 0x0C  // BURST_CTRL1 Byte0 (W1)
#define ADDR_BURST_CTRL1_HI 0x0D  // BURST_CTRL1 Byte1 (W1)
#define ADDR_BURST_CTRL2_LO 0x0E  // BURST_CTRL2 Byte0 (W1)
#define ADDR_BURST_CTRL2_HI 0x0F  // BURST_CTRL2 Byte1 (W1)
#define ADDR_POL_CTRL_LO 0x10     // POL_CTRL Byte0 (W1)
#define ADDR_POL_CTRL_HI 0x11     // POL_CTRL Byte1 (W1)

#define ADDR_DLT_CTRL_LO 0x12   // DLT_CTRL Byte0 (W1)
#define ADDR_DLT_CTRL_HI 0x13   // DLT_CTRL Byte1 (W1)
#define ADDR_GLOB_CMD3_LO 0x12  // GLOB_CMD3 Byte0 (W1) Same address as DLT_CTRL
#define ADDR_GLOB_CMD3_HI 0x13  // GLOB_CMD3 Byte1 (W1) Same address as DLT_CTRL

#define ADDR_ATTI_CTRL_LO 0x14  // ATTI_CTRL Byte0 (W1)
#define ADDR_ATTI_CTRL_HI 0x15  // ATTI_CTRL Byte1 (W1)
#define ADDR_GLOB_CMD2_LO 0x16  // ATTI_GLOB_CMD2 Byte0 (W1)
#define ADDR_GLOB_CMD2_HI 0x17  // ATTI_GLOB_CMD2 Byte1 (W1)

// For G330PDG0, G366PDG0, G370PDG0, G370PDT0 Rotation Matrix for both Gyro and
// Accel
#define ADDR_R_MATRIX_M11_LO 0x38  // R_MATRIX_M11 Byte0 (W1)
#define ADDR_R_MATRIX_M11_HI 0x39  // R_MATRIX_M11 Byte1 (W1)
#define ADDR_R_MATRIX_M12_LO 0x3A  // R_MATRIX_M12 Byte0 (W1)
#define ADDR_R_MATRIX_M12_HI 0x3B  // R_MATRIX_M12 Byte1 (W1)
#define ADDR_R_MATRIX_M13_LO 0x3C  // R_MATRIX_M13 Byte0 (W1)
#define ADDR_R_MATRIX_M13_HI 0x3D  // R_MATRIX_M13 Byte1 (W1)
#define ADDR_R_MATRIX_M21_LO 0x3E  // R_MATRIX_M21 Byte0 (W1)
#define ADDR_R_MATRIX_M21_HI 0x3F  // R_MATRIX_M21 Byte1 (W1)
#define ADDR_R_MATRIX_M22_LO 0x40  // R_MATRIX_M22 Byte0 (W1)
#define ADDR_R_MATRIX_M22_HI 0x41  // R_MATRIX_M22 Byte1 (W1)
#define ADDR_R_MATRIX_M23_LO 0x42  // R_MATRIX_M23 Byte0 (W1)
#define ADDR_R_MATRIX_M23_HI 0x43  // R_MATRIX_M23 Byte1 (W1)
#define ADDR_R_MATRIX_M31_LO 0x44  // R_MATRIX_M31 Byte0 (W1)
#define ADDR_R_MATRIX_M31_HI 0x45  // R_MATRIX_M31 Byte1 (W1)
#define ADDR_R_MATRIX_M32_LO 0x46  // R_MATRIX_M32 Byte0 (W1)
#define ADDR_R_MATRIX_M32_HI 0x47  // R_MATRIX_M32 Byte1 (W1)
#define ADDR_R_MATRIX_M33_LO 0x48  // R_MATRIX_M33 Byte0 (W1)
#define ADDR_R_MATRIX_M33_HI 0x49  // R_MATRIX_M33 Byte1 (W1)

// For G365PDC1, G365PDF1, G370PDF1, G370PDS0 Rotation Matrix for Accel
#define R_MATRIX_A_M11_LO 0x4A  // R_MATRIX_A_M11 Byte0 (W1)
#define R_MATRIX_A_M11_HI 0x4B  // R_MATRIX_A_M11 Byte1 (W1)
#define R_MATRIX_A_M12_LO 0x4C  // R_MATRIX_A_M12 Byte0 (W1)
#define R_MATRIX_A_M12_HI 0x4D  // R_MATRIX_A_M12 Byte1 (W1)
#define R_MATRIX_A_M13_LO 0x4E  // R_MATRIX_A_M13 Byte0 (W1)
#define R_MATRIX_A_M13_HI 0x4F  // R_MATRIX_A_M13 Byte1 (W1)
#define R_MATRIX_A_M21_LO 0x50  // R_MATRIX_A_M21 Byte0 (W1)
#define R_MATRIX_A_M21_HI 0x51  // R_MATRIX_A_M21 Byte1 (W1)
#define R_MATRIX_A_M22_LO 0x52  // R_MATRIX_A_M22 Byte0 (W1)
#define R_MATRIX_A_M22_HI 0x53  // R_MATRIX_A_M22 Byte1 (W1)
#define R_MATRIX_A_M23_LO 0x54  // R_MATRIX_A_M23 Byte0 (W1)
#define R_MATRIX_A_M23_HI 0x55  // R_MATRIX_A_M23 Byte1 (W1)
#define R_MATRIX_A_M31_LO 0x56  // R_MATRIX_A_M31 Byte0 (W1)
#define R_MATRIX_A_M31_HI 0x57  // R_MATRIX_A_M31 Byte1 (W1)
#define R_MATRIX_A_M32_LO 0x58  // R_MATRIX_A_M32 Byte0 (W1)
#define R_MATRIX_A_M32_HI 0x59  // R_MATRIX_A_M32 Byte1 (W1)
#define R_MATRIX_A_M33_LO 0x5A  // R_MATRIX_A_M33 Byte0 (W1)
#define R_MATRIX_A_M33_HI 0x5B  // R_MATRIX_A_M33 Byte1 (W1)

#define ADDR_PROD_ID1 0x6A     // PROD_ID1(W1)
#define ADDR_PROD_ID2 0x6C     // PROD_ID2(W1)
#define ADDR_PROD_ID3 0x6E     // PROD_ID3(W1)
#define ADDR_PROD_ID4 0x70     // PROD_ID4(W1)
#define ADDR_VERSION 0x72      // VERSION(W1)
#define ADDR_SERIAL_NUM1 0x74  // SERIAL_NUM1(W1)
#define ADDR_SERIAL_NUM2 0x76  // SERIAL_NUM2(W1)
#define ADDR_SERIAL_NUM3 0x78  // SERIAL_NUM3(W1)
#define ADDR_SERIAL_NUM4 0x7A  // SERIAL_NUM4(W1)
#define ADDR_WIN_CTRL 0x7E     // WIN_CTRL(W0 or W1)

#define WIN_ID0 0x00
#define WIN_ID1 0x01

#define CMD_BURST 0x80     // Write value to Issue Burst Read
#define CMD_SAMPLING 0x01  // Write value for MODE_CMD_HI to begin sampling
#define CMD_CONFIG 0x02    // Write value for MODE_CMD_HI to stop sampling

// Write values for ADDR_SMPL_CTRL_HI to set Output Rate
#define CMD_RATE2000 0x00    // TAP>=0
#define CMD_RATE1000 0x01    // TAP>=2
#define CMD_RATE500 0x02     // TAP>=4
#define CMD_RATE250 0x03     // TAP>=8
#define CMD_RATE125 0x04     // TAP>=16
#define CMD_RATE62_5 0x05    // TAP>=32
#define CMD_RATE31_25 0x06   // TAP>=64
#define CMD_RATE15_625 0x07  // TAP=128
#define CMD_RATE400 0x08     // TAP>=8
#define CMD_RATE200 0x09     // TAP>=16
#define CMD_RATE100 0x0A     // TAP>=32
#define CMD_RATE80 0x0B      // TAP>=32
#define CMD_RATE50 0x0C      // TAP>=64
#define CMD_RATE40 0x0D      // TAP>=64
#define CMD_RATE25 0x0E      // TAP=128
#define CMD_RATE20 0x0F      // TAP=128

// Write values for FILTER_CTRL_LO to set Filter
#define CMD_FLTAP0 0x00
#define CMD_FLTAP2 0x01
#define CMD_FLTAP4 0x02
#define CMD_FLTAP8 0x03
#define CMD_FLTAP16 0x04
#define CMD_FLTAP32 0x05
#define CMD_FLTAP64 0x06
#define CMD_FLTAP128 0x07
#define CMD_FIRTAP32FC50 0x08
#define CMD_FIRTAP32FC100 0x09
#define CMD_FIRTAP32FC200 0x0A
#define CMD_FIRTAP32FC400 0x0B
#define CMD_FIRTAP64FC50 0x0C
#define CMD_FIRTAP64FC100 0x0D
#define CMD_FIRTAP64FC200 0x0E
#define CMD_FIRTAP64FC400 0x0F
#define CMD_FIRTAP128FC50 0x10
#define CMD_FIRTAP128FC100 0x11
#define CMD_FIRTAP128FC200 0x12
#define CMD_FIRTAP128FC400 0x13

// Write values for ATTITUDE_MOTION_PROFILE
#define CMD_ATM_MODEA 0x00
#define CMD_ATM_MODEB 0x10
#define CMD_ATM_MODEC 0x20

// MODE STAT
#define VAL_SAMPLING_MODE 0x00
#define VAL_CONFIG_MODE 0x04

#ifdef __cplusplus
extern "C" {
#endif

enum EpsonModels {
  G_EMPTY,
  G320PDG0,
  G320PDGN,
  G354PDH0,
  G364PDCA,
  G364PDC0,
  G365PDC1,
  G365PDF1,
  G370PDF1,
  G370PDFN,
  G370PDS0,
  G330PDG0,
  G366PDG0,
  G370PDG0,
  G370PDT0,
  G570PR20,
  G_UNKNOWN,
};

enum EpsonFeatureFlags {
  HAS_ATTITUDE_OUTPUT = 1,
  HAS_DLT_OUTPUT = 2,
  HAS_ATTI_ON_REG = 4,
  HAS_ROT_MATRIX = 8,
  HAS_RANGE_OVER = 16,
  HAS_RTDIAG = 32,
  HAS_ARANGE = 64,
  HAS_INITIAL_BACKUP = 128,
};

// Epson sensor model specific properties
struct EpsonProperties {
  int model;
  const char* product_id;
  int feature_flags;
  double gyro_sf_dps;
  double accl_sf_mg;
  double tempc_sf_degc;
  int tempc_25c_offset;
  int rstcnt_sf_micros;
  double ang_sf_deg;
  double qtn_sf;
  double dlta0_sf_deg;
  double dltv0_sf_mps;
  int delay_reset_ms;
  int delay_flashtest_ms;
  int delay_flashbackup_ms;
  int delay_selftest_ms;
  int delay_filter_ms;
  int delay_atti_profile_ms;
};

// Array of EpsonProperties structs
extern struct EpsonProperties
  epson_sensors[G_UNKNOWN + 1];  // number of items in enum EpsonModels

// Epson sensor initialization options
struct EpsonOptions {
  // MSC_CTRL
  int ext_sel;
  int ext_pol;
  int drdy_on;
  int drdy_pol;

  // SMPL_CTRL
  int dout_rate;

  // FILTER_CTRL
  int filter_sel;

  // BURST_CTRL1
  int flag_out;
  int temp_out;
  int gyro_out;
  int accel_out;
  int gyro_delta_out;
  int accel_delta_out;
  int qtn_out;
  int atti_out;

  int gpio_out;
  int count_out;
  int checksum_out;

  // BURST_CTRL2
  int temp_bit;
  int gyro_bit;
  int accel_bit;
  int gyro_delta_bit;
  int accel_delta_bit;
  int qtn_bit;
  int atti_bit;

  // POL_CTRL
  int invert_xgyro;
  int invert_ygyro;
  int invert_zgyro;
  int invert_xaccel;
  int invert_yaccel;
  int invert_zaccel;

  int a_range_ctrl;

  // DLT_CTRL
  int dlta_range_ctrl;
  int dltv_range_ctrl;

  // ATTI_CTRL
  int atti_mode;
  int atti_conv;

  // ATTI_MOTION_PROFILE
  int atti_profile;
};

// Epson sensor burst data storage structure
struct EpsonData {
  unsigned short ndflags;
  double temperature;
  double gyro_x, gyro_y, gyro_z;
  double accel_x, accel_y, accel_z;
  double gyro_delta_x, gyro_delta_y, gyro_delta_z;
  double accel_delta_x, accel_delta_y, accel_delta_z;
  double qtn0, qtn1, qtn2, qtn3;
  double ang1, ang2, ang3;
  unsigned short gpio;
  int count;
  unsigned short chksm;
};

// ***************************************************
// Defined in sensor_epsonUart.c or sensor_epsonSpi.c
void writeByte(unsigned char, unsigned char, unsigned int);
void registerWriteByte(unsigned char, unsigned char, unsigned char,
                       unsigned int);
unsigned short read16(unsigned char, unsigned int);
unsigned short registerRead16(unsigned char, unsigned char, unsigned int);
int sensorDataReadBurstNOptions(const struct EpsonProperties*,
                                const struct EpsonOptions*, struct EpsonData*);
// ***************************************************

int sensorHWReset(void);
int sensorPowerOn(void);
void sensorStart(void);
void sensorStop(void);
void sensorReset(const struct EpsonProperties*);
int sensorFlashTest(const struct EpsonProperties*);
int sensorSelfTest(const struct EpsonProperties*);
int sensorInitialBackup(const struct EpsonProperties*);
int sensorFlashBackup(const struct EpsonProperties*);
unsigned int sensorDataByteLength(const struct EpsonProperties*,
                                  const struct EpsonOptions*);
void sensorDummyWrite(void);
char* sensorGetProductId(char* prod_id);
char* sensorGetSerialId(char* serial_id);
int sensorInitOptions(const struct EpsonProperties*, struct EpsonOptions*);
void sensorDumpRegisters(const struct EpsonProperties*);
int sensorGetDeviceModel(struct EpsonProperties*, char* prod_id,
                         char* serial_id);

#ifdef __cplusplus
}
#endif
