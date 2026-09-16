//==============================================================================
//
//  sensor_epsonSpi.c - Epson IMU sensor protocol SPI specific code
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
#include "hcl_gpio.h"
#include "sensor_epsonCommon.h"
#include "sensor_epsonSpi.h"

static unsigned short rxdata[128];

// Local function prototypes
int sensorDataReady(void);
void sensorDataReadN(unsigned short[], unsigned int, unsigned char);
void sensorDataReadBurstN(unsigned short[], unsigned int);
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
  selEpson();
  spiTransfer(regAddr | 0x80);  // msb is 1b for register writes
  spiTransfer(regByte);
  epsonStall();
  deselEpson();

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
  short rxData[] = {0x00, 0x00};

  selEpson();
  spiTransfer(regAddr &
              0x7E);  // msb is 0b for register reads & address must be even
  spiTransfer(0x00);
  epsonStall();

  rxData[0] = spiTransfer(0x00);
  rxData[1] = spiTransfer(0x00);
  epsonStall();
  deselEpson();

  if (verbose) {
    printf("REG[0x%02X] > 0x%02X%02X\t", regAddr, rxData[0], rxData[1]);
  }
  return (rxData[0] << 8 | rxData[1]);
}

/*****************************************************************************
** Function name:       registerRead16
** Description:         Read 16-bit from Register
** Parameters:          Window Number, Register Address, Verbose Flag
** Return value:        Register Read Value 16-bit
*****************************************************************************/
unsigned short registerRead16(unsigned char winNumber, unsigned char regAddr,
                              unsigned int verbose) {
  writeByte(ADDR_WIN_CTRL, winNumber, 0);

  unsigned short rxData = 0x0000;
  rxData = read16(regAddr, 0);
  if (verbose) {
    printf("REG[0x%02X(W%01d)] > 0x%04X\t", regAddr, winNumber, rxData);
  }
  return rxData;
}

/*****************************************************************************
** Function name:       sensorDataReady
** Description:         For SPI IF check if DataReady is HIGH.
** Parameters:          None
** Return value:        1=DataReady HIGH or 0=DataReady LOW
*****************************************************************************/
int sensorDataReady(void) {
  // Sensor data is ready when the DRDY Pin is HIGH
  return (gpioGetPinLevel(EPSON_DRDY));
}

/*****************************************************************************
** Function name:       sensorDataReadN
** Description:         Wait for DRDY assertion, then sequentially
**                      read sensor data (address is incremented by 2)
** Parameters:          pointer to unsigned short array
**                      size of array,
**                      register start address
** Return value:        none
** Notes:
** 1. Each register contains 16-bit data, and typical registers to read are:
**
**    For 32-bit output (It is recommended to use sensorDataBurstReadN()
**        instead):
**    COUNT [0Ah-0Bh], DUMMY[0Ch-0Dh], TEMPC [0Eh-11h], GX GY GZ [12h-1Ch],
**        AX AY AZ [1Eh-28h]
**    Total = Count(2) + Dummy(2) + Temp(4) + GyroXYZ(4*3) + AccelXYZ(4*3)
**        = 32 bytes = 16 words
**
**    For 16-bit output:
**    NDFLAG [00h-01h], TEMP[02h-03h], GX GY GZ[04h-09h], AX AY AZ[0Ah-0Fh],
**        GPIO[10h-11h], COUNT[012h-13h]
**    Total = ND_FLAGS(2) + Temp(2) + GyroXYZ(2*3) + AccelXYZ(2*3) + GPIO(2) +
**        Count(2) = 20 bytes = 10 words
**
** 2. This function will send N+1 SPI commands to read N registers.
**    To achieve high performance, we use this function to retrieve burst sensor
**    data instead of calling separate registerRead16().
**    Maximum SPI clock is 2MHz for non-burst SPI reads.
*****************************************************************************/
void sensorDataReadN(unsigned short sensorReadData[], unsigned int readLen,
                     unsigned char regAddr) {
  unsigned int i;

  // Wait for DataReady until retry timeout
  int retryCount = 50000;
  do {
    seDelayMicroSecs(10);
    retryCount--;
    if (retryCount == 0) {
      printf("Retry exceeded waiting for DRDY\n");
      break;
    }
  } while (!sensorDataReady());

  selEpson();
  spiTransfer(regAddr);
  spiTransfer(0x00);
  epsonStall();

  for (i = 0; i < readLen; i++) {
    signed int tmp = spiTransfer(regAddr + (2 * (i + 1)));
    sensorReadData[i] = (tmp << 8) + spiTransfer(0x00);
    epsonStall();
  }
  deselEpson();
}

/*****************************************************************************
** Function name:       sensorDataReadBurstN
** Description:         Wait for DRDY assertion, then burst read sensor data
**                      according to Epson sensor protocol
** Parameters:          pointer to unsigned short array
**                      size of array
** Return value:        none
** Notes:
** 1. The burst packet consists of 16-bit data units.
**    Ex. for 32-bit sensor output, Total = GyroXYZ(4*3) + AccelXYZ(4*3) +
**        Count(2) + Chksum(2) = 28 bytes = 14 words
**    For 16-bit sensor output, Total = ND_FLAGS(2) + Temp(2) + GyroXYZ(2*3) +
**        AccelXYZ(2*3) + GPIO(2) + Count(2) = 20 bytes = 10 words
** 2. For SPI interface, this function will send N+1 SPI commands to read N
**    registers.
**    Maximum SPI clock is 1MHz for burst SPI reads.
** 3. No checksum verification is performed (in this function)
**    To achieve high performance, we use this function to retrieve burst sensor
**    data instead of calling separate registerRead16().
**
*****************************************************************************/
void sensorDataReadBurstN(unsigned short sensorReadData[],
                          unsigned int readLen) {
  unsigned int i;

  // Wait for DataReady until retry timeout
  int retryCount = 50000;
  do {
    seDelayMicroSecs(10);
    retryCount--;
    if (retryCount == 0) {
      printf("Retry exceeded waiting for DRDY\n");
      break;
    }
  } while (!sensorDataReady());

  selEpson();
  spiTransfer(CMD_BURST);
  spiTransfer(0x00);
  burstStall1();

  for (i = 0; i < readLen; i++) {
    signed short tmp = spiTransfer(0x00);
    sensorReadData[i] = (tmp << 8) + spiTransfer(0x00);
    burstStall2();
  }
  deselEpson();
}

/*****************************************************************************
** Function name:       sensorDataScaling
** Description:         Processed burst buffer, converts and stores into
**                      sensor data struct based on device settings.
**                      based on configuration.
** Parameters:          esensor - pointer to struct describing IMU properties.
**                      options - pointer to struct describing IMU settings.
**                      data - pointer to struct for converted sensor data.
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
    unsigned short ndflags = rxdata[idx];
    data->ndflags = ndflags;
    idx++;
  }

  if (options->temp_out) {
    if (options->temp_bit) {
      // 32-bit calculation
      int temp = (rxdata[idx] << 8 * 2) + rxdata[idx + 1];
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
      idx += 2;

    } else {
      // 16-bit calculation
      short temp = rxdata[idx];
      if ((esensor->model == G330PDG0) || (esensor->model == G366PDG0) ||
          (esensor->model == G370PDG0) || (esensor->model == G370PDT0) ||
          (esensor->model == G570PR20)) {
        // These models do not have a 25degC temperature offset
        data->temperature = temp * esensor->tempc_sf_degc + 25;
      } else {
        data->temperature =
          (temp - esensor->tempc_25c_offset) * esensor->tempc_sf_degc + 25;
      }
      idx++;
    }
  }

  if (options->gyro_out) {
    if (options->gyro_bit) {
      // 32-bit calculation
      int gyro_x = (rxdata[idx] << 8 * 2) + rxdata[idx + 1];
      int gyro_y = (rxdata[idx + 2] << 8 * 2) + rxdata[idx + 3];
      int gyro_z = (rxdata[idx + 4] << 8 * 2) + rxdata[idx + 5];
      data->gyro_x = (esensor->gyro_sf_dps / 65536) * DEG2RAD * gyro_x;
      data->gyro_y = (esensor->gyro_sf_dps / 65536) * DEG2RAD * gyro_y;
      data->gyro_z = (esensor->gyro_sf_dps / 65536) * DEG2RAD * gyro_z;
      idx += 6;
    } else {
      // 16-bit calculation
      short gyro_x = rxdata[idx];
      short gyro_y = rxdata[idx + 1];
      short gyro_z = rxdata[idx + 2];
      data->gyro_x = esensor->gyro_sf_dps * DEG2RAD * gyro_x;
      data->gyro_y = esensor->gyro_sf_dps * DEG2RAD * gyro_y;
      data->gyro_z = esensor->gyro_sf_dps * DEG2RAD * gyro_z;
      idx += 3;
    }
  }

  if (options->accel_out) {
    if (options->accel_bit) {
      // 32-bit calculation
      int accel_x = (rxdata[idx] << 8 * 2) + rxdata[idx + 1];
      int accel_y = (rxdata[idx + 2] << 8 * 2) + rxdata[idx + 3];
      int accel_z = (rxdata[idx + 4] << 8 * 2) + rxdata[idx + 5];
      data->accel_x = (esensor->accl_sf_mg / 65536) * MG2MPS2 * accel_x;
      data->accel_y = (esensor->accl_sf_mg / 65536) * MG2MPS2 * accel_y;
      data->accel_z = (esensor->accl_sf_mg / 65536) * MG2MPS2 * accel_z;
      idx += 6;
    } else {
      // 16-bit calculation
      short accel_x = rxdata[idx];
      short accel_y = rxdata[idx + 1];
      short accel_z = rxdata[idx + 2];
      data->accel_x = (esensor->accl_sf_mg) * MG2MPS2 * accel_x;
      data->accel_y = (esensor->accl_sf_mg) * MG2MPS2 * accel_y;
      data->accel_z = (esensor->accl_sf_mg) * MG2MPS2 * accel_z;
      idx += 3;
    }
  }

  if (options->gyro_delta_out) {
    double da_sf =
      esensor->dlta0_sf_deg * (1 << options->dlta_range_ctrl) * DEG2RAD;
    if (options->gyro_delta_bit) {
      // 32-bit calculation
      int gyro_delta_x = (rxdata[idx] << 8 * 2) + rxdata[idx + 1];
      int gyro_delta_y = (rxdata[idx + 2] << 8 * 2) + rxdata[idx + 3];
      int gyro_delta_z = (rxdata[idx + 4] << 8 * 2) + rxdata[idx + 5];
      data->gyro_delta_x = gyro_delta_x * (da_sf) / 65536;
      data->gyro_delta_y = gyro_delta_y * (da_sf) / 65536;
      data->gyro_delta_z = gyro_delta_z * (da_sf) / 65536;
      idx += 6;
    } else {
      // 16-bit calculation
      short gyro_delta_x = rxdata[idx];
      short gyro_delta_y = rxdata[idx + 1];
      short gyro_delta_z = rxdata[idx + 2];
      data->gyro_delta_x = gyro_delta_x * (da_sf);
      data->gyro_delta_y = gyro_delta_y * (da_sf);
      data->gyro_delta_z = gyro_delta_z * (da_sf);
      idx += 3;
    }
  }

  if (options->accel_delta_out) {
    double dv_sf = esensor->dltv0_sf_mps * (1 << options->dltv_range_ctrl);
    if (options->accel_delta_bit) {
      // 32-bit calculation
      int accel_delta_x = (rxdata[idx] << 8 * 2) + rxdata[idx + 1];
      int accel_delta_y = (rxdata[idx + 2] << 8 * 2) + rxdata[idx + 3];
      int accel_delta_z = (rxdata[idx + 4] << 8 * 2) + rxdata[idx + 5];
      data->accel_delta_x = accel_delta_x * (dv_sf) / 65536;
      data->accel_delta_y = accel_delta_y * (dv_sf) / 65536;
      data->accel_delta_z = accel_delta_z * (dv_sf) / 65536;
      idx += 6;
    } else {
      // 16-bit calculation
      short accel_delta_x = rxdata[idx];
      short accel_delta_y = rxdata[idx + 1];
      short accel_delta_z = rxdata[idx + 2];
      data->accel_delta_x = accel_delta_x * (dv_sf);
      data->accel_delta_y = accel_delta_y * (dv_sf);
      data->accel_delta_z = accel_delta_z * (dv_sf);
      idx += 3;
    }
  }

  if (options->qtn_out) {
    if (options->qtn_bit) {
      // 32-bit calculation
      int qtn0 = (rxdata[idx] << 8 * 2) + rxdata[idx + 1];
      int qtn1 = (rxdata[idx + 2] << 8 * 2) + rxdata[idx + 3];
      int qtn2 = (rxdata[idx + 4] << 8 * 2) + rxdata[idx + 5];
      int qtn3 = (rxdata[idx + 6] << 8 * 2) + rxdata[idx + 7];
      data->qtn0 = (double)qtn0 * esensor->qtn_sf / 65536;
      data->qtn1 = (double)qtn1 * esensor->qtn_sf / 65536;
      data->qtn2 = (double)qtn2 * esensor->qtn_sf / 65536;
      data->qtn3 = (double)qtn3 * esensor->qtn_sf / 65536;
      idx += 8;
    } else {
      // 16-bit calculation
      short qtn0 = rxdata[idx];
      short qtn1 = rxdata[idx + 1];
      short qtn2 = rxdata[idx + 2];
      short qtn3 = rxdata[idx + 3];
      data->qtn0 = (double)qtn0 * esensor->qtn_sf;
      data->qtn1 = (double)qtn1 * esensor->qtn_sf;
      data->qtn2 = (double)qtn2 * esensor->qtn_sf;
      data->qtn3 = (double)qtn3 * esensor->qtn_sf;
      idx += 4;
    }
  }

  if (options->atti_out) {
    if (options->atti_bit) {
      // 32-bit calculation
      int ang1 = (rxdata[idx] << 8 * 2) + rxdata[idx + 1];
      int ang2 = (rxdata[idx + 2] << 8 * 2) + rxdata[idx + 3];
      int ang3 = (rxdata[idx + 4] << 8 * 2) + rxdata[idx + 5];
      data->ang1 = (esensor->ang_sf_deg / 65536) * DEG2RAD * ang1;
      data->ang2 = (esensor->ang_sf_deg / 65536) * DEG2RAD * ang2;
      data->ang3 = (esensor->ang_sf_deg / 65536) * DEG2RAD * ang3;
      idx += 6;
    } else {
      // 16-bit calculation
      short ang1 = rxdata[idx];
      short ang2 = rxdata[idx + 1];
      short ang3 = rxdata[idx + 2];
      data->ang1 = esensor->ang_sf_deg * DEG2RAD * ang1;
      data->ang2 = esensor->ang_sf_deg * DEG2RAD * ang2;
      data->ang3 = esensor->ang_sf_deg * DEG2RAD * ang3;
      idx += 3;
    }
  }

  if (options->gpio_out) {
    unsigned short gpio = rxdata[idx];
    data->gpio = gpio;
    idx++;
  }

  if (options->count_out) {
    int count = rxdata[idx];
    if (options->ext_sel == 1)
      data->count = count * esensor->rstcnt_sf_micros;
    else
      data->count = count;
  }
}

/*****************************************************************************
** Function name:       sensorDataReadBurstNOptions
** Description:         Wait for DataReady to be active, then burst reads
**                      1 packet from sensor device, then calls
**                      sensorDataScaling() to post process into data struct.
** Parameters:          pointer to struct describing IMU properties.
**                      pointer to struct describing IMU settings.
**                      pointer to struct that stores sensor data.
** Return value:        OK or NG (DataReady retry exceed or checksum error)
** Notes:
******************************************************************************/
int sensorDataReadBurstNOptions(const struct EpsonProperties* esensor,
                                const struct EpsonOptions* options,
                                struct EpsonData* data) {
  // Wait for DataReady or return NG if retry timeout
  int retryCount = 50000;
  do {
    seDelayMicroSecs(10);
    retryCount--;
    if (retryCount == 0) {
      return NG;
    }
  } while (!sensorDataReady());

  // Burst read the sensor data based on calculated burst size from
  // sensor properties and options
  unsigned int data_length = sensorDataByteLength(esensor, options) / 2;
  sensorDataReadBurstN(rxdata, data_length);
  // If checksum enabled, validate checksum and populate sensor data in
  // structure
  if (options->checksum_out == 1) {
    unsigned short calc_checksum = 0;
    for (unsigned int i = 0; i < data_length - 1; i++) {
      calc_checksum += rxdata[i];
    }
    unsigned short epson_checksum = rxdata[data_length - 1];

    if (calc_checksum != epson_checksum) {
      printf("checksum failed\n");
      return NG;
    }
  }
  sensorDataScaling(esensor, options, data);
  return OK;
}
