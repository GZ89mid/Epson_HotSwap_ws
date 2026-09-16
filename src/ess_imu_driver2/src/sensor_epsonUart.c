//==============================================================================
//
//  sensor_epsonUart.c - Epson IMU sensor protocol UART specific code
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

#include <stdio.h>

#include "hcl.h"
#include "hcl_uart.h"
#include "sensor_epsonCommon.h"
#include "sensor_epsonUart.h"

// This is declared by the main() application for UART IF
extern const char* IMUSERIAL;  // COM port device name

// UART Byte Markers
// Start of all UART transfers
const unsigned char UART_HEADER = 0x80;
// End of all UART transfers
const unsigned char UART_DELIMITER = 0x0D;

// COM port receive buffer
static unsigned char rxByteBuf[256];

// Macros & variables used by state machine for processing UART burst read data
#define START 0
#define DATA 1
#define END 2
static int state = START;
static int data_count = 0;

// Local function prototypes
int sensorDataReadyOptions(const struct EpsonProperties*,
                           const struct EpsonOptions*);
void sensorDataScaling(const struct EpsonProperties*,
                       const struct EpsonOptions*, struct EpsonData*);

/*****************************************************************************
** Function name:       writeByte
** Description:         Write Byte to Register = Write Data
**                      to Register (no WIN_ID)
** Parameters:          Register Address, Register Write Byte,
**                      Verbose Flag
** Return value:        None
*****************************************************************************/
void writeByte(unsigned char regAddr, unsigned char regByte,
               unsigned int verbose) {
  unsigned char txData[3];

  // msb is 1b for register writes
  txData[0] = regAddr | 0x80;
  txData[1] = regByte;
  txData[2] = UART_DELIMITER;
  writeComPort(txData, 3);
  epsonStall();

  if (verbose) {
    printf("\r\nREG[0x%02X] < 0x%02X\t", regAddr, regByte);
  }
}

/*****************************************************************************
** Function name:       registerWriteByte
** Description:         Write Byte to Register = Set WIN_ID, Write Data
**                      to Register
** Parameters:          Window Number, Register Address, Register Write Byte,
**                      Verbose Flag
** Return value:        None
*****************************************************************************/
void registerWriteByte(unsigned char winNumber, unsigned char regAddr,
                       unsigned char regByte, unsigned int verbose) {
  writeByte(ADDR_WIN_CTRL, winNumber, 0);
  writeByte(regAddr, regByte, 0);
  if (verbose) {
    printf("\r\nREG[0x%02X(W%01d)] < 0x%02X\t", regAddr, winNumber, regByte);
  }
}

/*****************************************************************************
** Function name:       read16
** Description:         Read 16-bit from Register (No WIN_ID)
** Parameters:          Register Address, Verbose Flag
** Return value:        Register Read Value 16-bit
*****************************************************************************/
unsigned short read16(unsigned char regAddr, unsigned int verbose) {
  unsigned char response[4] = {0};
  int size;
  unsigned char txData[3];

  // msb is 0b for register reads & address must be even
  txData[0] = regAddr & 0x7E;
  txData[1] = 0x00;
  txData[2] = UART_DELIMITER;
  writeComPort(txData, 3);
  epsonStall();

  // Attempt to read 4 bytes from serial port
  // Validation check: Should be atleast 4 bytes,
  // First byte should be Register, Address,
  // Last byte should be delimiter
  size = readComPort(&response[0], 4);

  if ((size < 4) || (response[0] != txData[0]) ||
      (response[3] != UART_DELIMITER)) {
    printf("Returned less data or unexpected data from previous command.\n");
    printf("Return data: 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", response[0],
           response[1], response[2], response[3]);
  }

  if (verbose) {
    printf("REG[0x%02X] > 0x%02X%02X\t", regAddr, response[1], response[2]);
  }
  return (unsigned short)response[1] << 8 | (unsigned short)response[2];
}

/*****************************************************************************
** Function name:       registerRead16
** Description:         Read 16-bit from Register
** Parameters:          Window Number, Register Address, Verbose Flag
** Return value:        Register Read Value 16-bit
*****************************************************************************/
unsigned short registerRead16(unsigned char winNumber, unsigned char regAddr,
                              unsigned int verbose) {
  unsigned short rxData;

  writeByte(ADDR_WIN_CTRL, winNumber, 0);
  rxData = read16(regAddr, 0);
  if (verbose) {
    printf("REG[0x%02X(W%01d)] > 0x%04X\t", regAddr, winNumber, rxData);
  }
  return rxData;
}

/*****************************************************************************
** Function name:       sensorDataReadyOptions
** Description:         For UART interface check if comport recv buffer
**                      contains a burst of data based on expected byte length
**                      from sensorDataByteLength()
** Parameters:          None
** Return value:        OK or NG
*****************************************************************************/
int sensorDataReadyOptions(const struct EpsonProperties* esensor,
                           const struct EpsonOptions* options) {
  seDelayMicroSecs(100);
  unsigned int count = numBytesReadComPort();

  if (count >= sensorDataByteLength(esensor, options)) return OK;
  return NG;
}

/*****************************************************************************
** Function name:       sensorDataScaling
** Description:         Retrieves burst data buffer, converts and stores into
**                      sensor data struct based on settings.
**                      based on configuration.
** Parameters:          pointer to struct describing IMU properties.
**                      pointer to struct describing IMU settings.
**                      pointer to struct for converted sensor data.
** Return value:        none
** Notes:
******************************************************************************/
void sensorDataScaling(const struct EpsonProperties* esensor,
                       const struct EpsonOptions* options,
                       struct EpsonData* data) {
  // stores the sensor data array index when parsing out data fields
  int idx = 0;

  // parsing of data fields applying conversion factor if applicable
  if (options->flag_out) {
    unsigned short ndflags = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
    data->ndflags = ndflags;
    idx += 2;
  }

  if (options->temp_out) {
    if (options->temp_bit) {
      // 32-bit calculation
      int temp = (rxByteBuf[idx] << 8 * 3) + (rxByteBuf[idx + 1] << 8 * 2) +
                 (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];

      if ((esensor->model == G330PDG0) || (esensor->model == G366PDG0) ||
          (esensor->model == G370PDG0) || (esensor->model == G370PDT0) ||
          (esensor->model == G570PR20)) {
        // These models do not have a 25degC temperature offset
        data->temperature = temp * esensor->tempc_sf_degc / 65536 + 25;
      } else {
        data->temperature = (temp - esensor->tempc_25c_offset * 65536) *
                              esensor->tempc_sf_degc / 65536 +
                            25;
      }

      idx += 4;
    } else {
      // 16-bit calculation
      short temp = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];

      if ((esensor->model == G330PDG0) || (esensor->model == G366PDG0) ||
          (esensor->model == G370PDG0) || (esensor->model == G370PDT0) ||
          (esensor->model == G570PR20)) {
        // These models do not have a 25degC temperature offset
        data->temperature = temp * esensor->tempc_sf_degc + 25;
      } else {
        data->temperature =
          (temp - esensor->tempc_25c_offset) * esensor->tempc_sf_degc + 25;
      }

      idx += 2;
    }
  }

  if (options->gyro_out) {
    if (options->gyro_bit) {
      // 32-bit calculation
      int gyro_x = (rxByteBuf[idx] << 8 * 3) + (rxByteBuf[idx + 1] << 8 * 2) +
                   (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      int gyro_y = (rxByteBuf[idx + 4] << 8 * 3) +
                   (rxByteBuf[idx + 5] << 8 * 2) + (rxByteBuf[idx + 6] << 8) +
                   rxByteBuf[idx + 7];
      int gyro_z = (rxByteBuf[idx + 8] << 8 * 3) +
                   (rxByteBuf[idx + 9] << 8 * 2) + (rxByteBuf[idx + 10] << 8) +
                   rxByteBuf[idx + 11];
      data->gyro_x = (esensor->gyro_sf_dps / 65536) * DEG2RAD * gyro_x;
      data->gyro_y = (esensor->gyro_sf_dps / 65536) * DEG2RAD * gyro_y;
      data->gyro_z = (esensor->gyro_sf_dps / 65536) * DEG2RAD * gyro_z;
      idx += 12;
    } else {
      // 16-bit calculation
      short gyro_x = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
      short gyro_y = (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      short gyro_z = (rxByteBuf[idx + 4] << 8) + rxByteBuf[idx + 5];
      data->gyro_x = esensor->gyro_sf_dps * DEG2RAD * gyro_x;
      data->gyro_y = esensor->gyro_sf_dps * DEG2RAD * gyro_y;
      data->gyro_z = esensor->gyro_sf_dps * DEG2RAD * gyro_z;
      idx += 6;
    }
  }

  if (options->accel_out) {
    if (options->accel_bit) {
      // 32-bit calculation
      int accel_x = (rxByteBuf[idx] << 8 * 3) + (rxByteBuf[idx + 1] << 8 * 2) +
                    (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      int accel_y = (rxByteBuf[idx + 4] << 8 * 3) +
                    (rxByteBuf[idx + 5] << 8 * 2) + (rxByteBuf[idx + 6] << 8) +
                    rxByteBuf[idx + 7];
      int accel_z = (rxByteBuf[idx + 8] << 8 * 3) +
                    (rxByteBuf[idx + 9] << 8 * 2) + (rxByteBuf[idx + 10] << 8) +
                    rxByteBuf[idx + 11];
      data->accel_x = (esensor->accl_sf_mg / 65536) * MG2MPS2 * accel_x;
      data->accel_y = (esensor->accl_sf_mg / 65536) * MG2MPS2 * accel_y;
      data->accel_z = (esensor->accl_sf_mg / 65536) * MG2MPS2 * accel_z;
      idx += 12;
    } else {
      // 16-bit calculation
      short accel_x = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
      short accel_y = (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      short accel_z = (rxByteBuf[idx + 4] << 8) + rxByteBuf[idx + 5];
      data->accel_x = (esensor->accl_sf_mg) * MG2MPS2 * accel_x;
      data->accel_y = (esensor->accl_sf_mg) * MG2MPS2 * accel_y;
      data->accel_z = (esensor->accl_sf_mg) * MG2MPS2 * accel_z;
      idx += 6;
    }
  }

  if (options->gyro_delta_out) {
    double da_sf =
      esensor->dlta0_sf_deg * (1 << options->dlta_range_ctrl) * DEG2RAD;
    if (options->gyro_delta_bit) {
      // 32-bit calculation
      int gyro_delta_x = (rxByteBuf[idx] << 8 * 3) +
                         (rxByteBuf[idx + 1] << 8 * 2) +
                         (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      int gyro_delta_y = (rxByteBuf[idx + 4] << 8 * 3) +
                         (rxByteBuf[idx + 5] << 8 * 2) +
                         (rxByteBuf[idx + 6] << 8) + rxByteBuf[idx + 7];
      int gyro_delta_z = (rxByteBuf[idx + 8] << 8 * 3) +
                         (rxByteBuf[idx + 9] << 8 * 2) +
                         (rxByteBuf[idx + 10] << 8) + rxByteBuf[idx + 11];

      data->gyro_delta_x = gyro_delta_x * (da_sf) / 65536;
      data->gyro_delta_y = gyro_delta_y * (da_sf) / 65536;
      data->gyro_delta_z = gyro_delta_z * (da_sf) / 65536;
      idx += 12;
    } else {
      // 16-bit calculation
      short gyro_delta_x = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
      short gyro_delta_y = (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      short gyro_delta_z = (rxByteBuf[idx + 4] << 8) + rxByteBuf[idx + 5];
      data->gyro_delta_x = gyro_delta_x * (da_sf);
      data->gyro_delta_y = gyro_delta_y * (da_sf);
      data->gyro_delta_z = gyro_delta_z * (da_sf);
      idx += 6;
    }
  }

  if (options->accel_delta_out) {
    double dv_sf = esensor->dltv0_sf_mps * (1 << options->dltv_range_ctrl);
    if (options->accel_delta_bit) {
      // 32-bit calculation
      int accel_delta_x = (rxByteBuf[idx] << 8 * 3) +
                          (rxByteBuf[idx + 1] << 8 * 2) +
                          (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      int accel_delta_y = (rxByteBuf[idx + 4] << 8 * 3) +
                          (rxByteBuf[idx + 5] << 8 * 2) +
                          (rxByteBuf[idx + 6] << 8) + rxByteBuf[idx + 7];
      int accel_delta_z = (rxByteBuf[idx + 8] << 8 * 3) +
                          (rxByteBuf[idx + 9] << 8 * 2) +
                          (rxByteBuf[idx + 10] << 8) + rxByteBuf[idx + 11];
      data->accel_delta_x = accel_delta_x * (dv_sf) / 65536;
      data->accel_delta_y = accel_delta_y * (dv_sf) / 65536;
      data->accel_delta_z = accel_delta_z * (dv_sf) / 65536;
      idx += 12;
    } else {
      // 16-bit calculation
      short accel_delta_x = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
      short accel_delta_y = (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      short accel_delta_z = (rxByteBuf[idx + 4] << 8) + rxByteBuf[idx + 5];
      data->accel_delta_x = accel_delta_x * (dv_sf);
      data->accel_delta_y = accel_delta_y * (dv_sf);
      data->accel_delta_z = accel_delta_z * (dv_sf);
      idx += 6;
    }
  }

  if (options->qtn_out) {
    if (options->qtn_bit) {
      // 32-bit calculation
      int qtn0 = (rxByteBuf[idx] << 8 * 3) + (rxByteBuf[idx + 1] << 8 * 2) +
                 (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      int qtn1 = (rxByteBuf[idx + 4] << 8 * 3) + (rxByteBuf[idx + 5] << 8 * 2) +
                 (rxByteBuf[idx + 6] << 8) + rxByteBuf[idx + 7];
      int qtn2 = (rxByteBuf[idx + 8] << 8 * 3) + (rxByteBuf[idx + 9] << 8 * 2) +
                 (rxByteBuf[idx + 10] << 8) + rxByteBuf[idx + 11];
      int qtn3 = (rxByteBuf[idx + 12] << 8 * 3) +
                 (rxByteBuf[idx + 13] << 8 * 2) + (rxByteBuf[idx + 14] << 8) +
                 rxByteBuf[idx + 15];
      data->qtn0 = (double)qtn0 * esensor->qtn_sf / 65536;
      data->qtn1 = (double)qtn1 * esensor->qtn_sf / 65536;
      data->qtn2 = (double)qtn2 * esensor->qtn_sf / 65536;
      data->qtn3 = (double)qtn3 * esensor->qtn_sf / 65536;
      idx += 16;
    } else {
      // 16-bit calculation
      short qtn0 = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
      short qtn1 = (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      short qtn2 = (rxByteBuf[idx + 4] << 8) + rxByteBuf[idx + 5];
      short qtn3 = (rxByteBuf[idx + 6] << 8) + rxByteBuf[idx + 7];
      data->qtn0 = (double)qtn0 * esensor->qtn_sf;
      data->qtn1 = (double)qtn1 * esensor->qtn_sf;
      data->qtn2 = (double)qtn2 * esensor->qtn_sf;
      data->qtn3 = (double)qtn3 * esensor->qtn_sf;
      idx += 8;
    }
  }

  if (options->atti_out) {
    if (options->atti_bit) {
      // 32-bit calculation
      int ang1 = (rxByteBuf[idx] << 8 * 3) + (rxByteBuf[idx + 1] << 8 * 2) +
                 (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      int ang2 = (rxByteBuf[idx + 4] << 8 * 3) + (rxByteBuf[idx + 5] << 8 * 2) +
                 (rxByteBuf[idx + 6] << 8) + rxByteBuf[idx + 7];
      int ang3 = (rxByteBuf[idx + 8] << 8 * 3) + (rxByteBuf[idx + 9] << 8 * 2) +
                 (rxByteBuf[idx + 10] << 8) + rxByteBuf[idx + 11];
      data->ang1 = (esensor->ang_sf_deg / 65536) * DEG2RAD * ang1;
      data->ang2 = (esensor->ang_sf_deg / 65536) * DEG2RAD * ang2;
      data->ang3 = (esensor->ang_sf_deg / 65536) * DEG2RAD * ang3;
      idx += 12;
    } else {
      // 16-bit calculation
      short ang1 = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
      short ang2 = (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      short ang3 = (rxByteBuf[idx + 4] << 8) + rxByteBuf[idx + 5];
      data->ang1 = esensor->ang_sf_deg * DEG2RAD * ang1;
      data->ang2 = esensor->ang_sf_deg * DEG2RAD * ang2;
      data->ang3 = esensor->ang_sf_deg * DEG2RAD * ang3;
      idx += 6;
    }
  }

  if (options->gpio_out) {
    unsigned short gpio = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
    data->gpio = gpio;
    idx += 2;
  }

  if (options->count_out) {
    int count = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
    if (options->ext_sel == 1)
      data->count = count * esensor->rstcnt_sf_micros;
    else
      data->count = count;
  }
}

/*****************************************************************************
** Function name:       sensorDataReadBurstNOptions
** Description:         Retrieves bytes from the incoming IMU stream of UART
**                      based on expected burst length and searching for START
**                      and END markers. Then calls sensorDataScaling() to
**                      post process into struct data.
** Parameters:          pointer to struct describing IMU properties.
**                      pointer to struct describing IMU settings.
**                      pointer to struct that stores sensor data.
** Return value:        OK or NG (checksum error)
** Notes:
******************************************************************************/
int sensorDataReadBurstNOptions(const struct EpsonProperties* esensor,
                                const struct EpsonOptions* options,
                                struct EpsonData* data) {
  int byte_length = sensorDataByteLength(esensor, options);

#ifdef DEBUG
  printf("Expecting: %d bytes\n", byte_length);
#endif

  int data_length = byte_length - 2;  // exclude the START and END markers
  unsigned char byte;

  while (readComPort(&byte, 1) > 0) {
#ifdef DEBUG
    printf("state: %d, byte: 0x%02X\n", state, byte);
#endif
    // State machine to seek out START & END markers and then
    // call to sensorDataScaling()
    switch (state) {
      case START:
        if (byte == UART_HEADER) state = DATA;
        break;
      case DATA:
        rxByteBuf[data_count] = byte;
        data_count++;
        if (data_count == data_length) state = END;
        break;
      case END:
        data_count = 0;
        state = START;
        if (byte == UART_DELIMITER) {
#ifdef DEBUG
          for (int i = 0; i < data_length; i++) printf("0x%02X ", rxByteBuf[i]);
          printf("\n");
#endif
          // If checksum enabled, validate
          // match = populate sensor data structure
          // no match = print error msg and skip current sensor burst data
          if (options->checksum_out == 1) {
            unsigned short calc_checksum = 0;
            for (int i = 0; i < data_length - 2; i += 2) {
              calc_checksum += (rxByteBuf[i] << 8) + rxByteBuf[i + 1];
            }
            unsigned short epson_checksum =
              (rxByteBuf[data_length - 2] << 8) + rxByteBuf[data_length - 1];

            if (calc_checksum != epson_checksum) {
              printf("checksum failed\n");
              return NG;
            }
          }
          sensorDataScaling(esensor, options, data);
          return OK;
        }
        break;
      default:
        // Should never get here
        printf("Invalid State in Read Burst Processing\n");
    }
  }
  // No byte received in serial port yet
  return NG;
}
