//==============================================================================
//
//  sensor_epsonCommon.c - Epson IMU sensor protocol specific code common
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

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hcl.h"
#include "hcl_gpio.h"
#include "sensor_epsonCommon.h"

//----------------------------------------------------------------------
// Array of structs for Epson device properties on per model basis
//----------------------------------------------------------------------
struct EpsonProperties epson_sensors[] = {
  [G_EMPTY] =
    {
      .model = G_EMPTY,
      .product_id = "",
      .feature_flags = 0,
      .gyro_sf_dps = (0),
      .accl_sf_mg = (0),
      .tempc_sf_degc = (0.0),
      .tempc_25c_offset = (0),
      .rstcnt_sf_micros = (0),
      .ang_sf_deg = (0),
      .qtn_sf = (0),
      .dlta0_sf_deg = (0),
      .dltv0_sf_mps = (0),
      .delay_reset_ms = (800),
      .delay_flashtest_ms = (30),
      .delay_flashbackup_ms = (200),
      .delay_selftest_ms = (80),
      .delay_filter_ms = (1),
      .delay_atti_profile_ms = (1),
    },
  [G320PDG0] =
    {
      .model = G320PDG0,
      .product_id = "G320PDG0",
      .feature_flags = (HAS_DLT_OUTPUT),
      .gyro_sf_dps = (1.0 / 125),
      .accl_sf_mg = (1.0 / 5),
      .tempc_sf_degc = (-0.0037918),
      .tempc_25c_offset = (2364),
      .rstcnt_sf_micros = (21333),
      .ang_sf_deg = (0),
      .qtn_sf = (0),
      .dlta0_sf_deg = (1.0 / 125 * 1 / 2000),
      .dltv0_sf_mps = (1.0 / 5 * 1 / 1000 * 1 / 2000 * 9.80665),
      .delay_reset_ms = (800),
      .delay_flashtest_ms = (5),
      .delay_flashbackup_ms = (200),
      .delay_selftest_ms = (80),
      .delay_filter_ms = (1),
      .delay_atti_profile_ms = (0),
    },
  [G320PDGN] =
    {
      .model = G320PDGN,
      .product_id = "G320PDGN",
      .feature_flags = (HAS_DLT_OUTPUT),
      .gyro_sf_dps = (1.0 / 125),
      .accl_sf_mg = (1.0 / 5),
      .tempc_sf_degc = (-0.0037918),
      .tempc_25c_offset = (2364),
      .rstcnt_sf_micros = (21333),
      .ang_sf_deg = (0),
      .qtn_sf = (0),
      .dlta0_sf_deg = (1.0 / 125 * 1 / 2000),
      .dltv0_sf_mps = (1.0 / 5 * 1 / 1000 * 1 / 2000 * 9.80665),
      .delay_reset_ms = (800),
      .delay_flashtest_ms = (5),
      .delay_flashbackup_ms = (200),
      .delay_selftest_ms = (80),
      .delay_filter_ms = (1),
      .delay_atti_profile_ms = (0),
    },
  [G354PDH0] =
    {
      .model = G354PDH0,
      .product_id = "G354PDH0",
      .feature_flags = (HAS_DLT_OUTPUT),
      .gyro_sf_dps = (1.0 / 62.5),
      .accl_sf_mg = (1.0 / 5),
      .tempc_sf_degc = (-0.0037918),
      .tempc_25c_offset = (2364),
      .rstcnt_sf_micros = (21333),
      .ang_sf_deg = (0),
      .qtn_sf = (0),
      .dlta0_sf_deg = (1.0 / 62.5 * 1 / 2000),
      .dltv0_sf_mps = (1.0 / 5 * 1 / 1000 * 1 / 2000 * 9.80665),
      .delay_reset_ms = (800),
      .delay_flashtest_ms = (5),
      .delay_flashbackup_ms = (200),
      .delay_selftest_ms = (80),
      .delay_filter_ms = (1),
      .delay_atti_profile_ms = (0),
    },
  [G364PDCA] =
    {
      .model = G364PDCA,
      .product_id = "G364PDCA",
      .feature_flags = (HAS_DLT_OUTPUT),
      .gyro_sf_dps = (0.00375),
      .accl_sf_mg = (1.0 / 8),
      .tempc_sf_degc = (-0.0037918),
      .tempc_25c_offset = (2364),
      .rstcnt_sf_micros = (21333),
      .ang_sf_deg = (0),
      .qtn_sf = (0),
      .dlta0_sf_deg = (0.00375 * 1 / 2000),
      .dltv0_sf_mps = (1.0 / 8 * 1 / 1000 * 1 / 2000 * 9.80665),
      .delay_reset_ms = (800),
      .delay_flashtest_ms = (5),
      .delay_flashbackup_ms = (200),
      .delay_selftest_ms = (80),
      .delay_filter_ms = (1),
      .delay_atti_profile_ms = (0),
    },
  [G364PDC0] =
    {
      .model = G364PDC0,
      .product_id = "G364PDC0",
      .feature_flags = (HAS_DLT_OUTPUT),
      .gyro_sf_dps = (0.00750),
      .accl_sf_mg = (1.0 / 8),
      .tempc_sf_degc = (-0.0037918),
      .tempc_25c_offset = (2364),
      .rstcnt_sf_micros = (21333),
      .ang_sf_deg = (0),
      .qtn_sf = (0),
      .dlta0_sf_deg = (0.00750 * 1 / 2000),
      .dltv0_sf_mps = (1.0 / 8 * 1 / 1000 * 1 / 2000 * 9.80665),
      .delay_reset_ms = (800),
      .delay_flashtest_ms = (5),
      .delay_flashbackup_ms = (200),
      .delay_selftest_ms = (80),
      .delay_filter_ms = (1),
      .delay_atti_profile_ms = (0),
    },
  [G365PDC1] =
    {
      .model = G365PDC1,
      .product_id = "G365PDC1",
      .feature_flags = (HAS_ATTITUDE_OUTPUT | HAS_DLT_OUTPUT | HAS_ATTI_ON_REG |
                        HAS_ROT_MATRIX | HAS_RANGE_OVER | HAS_INITIAL_BACKUP),
      .gyro_sf_dps = (1.0 / 66),
      .accl_sf_mg = (1.0 / 6.25),
      .tempc_sf_degc = (-0.0037918),
      .tempc_25c_offset = (2364),
      .rstcnt_sf_micros = (16000),
      .ang_sf_deg = (0.00699411),
      .qtn_sf = (1.0 / (2 << 13)),
      .dlta0_sf_deg = (1.0 / 66 * 1 / 2000),
      .dltv0_sf_mps = (1.0 / 6.25 * 1 / 1000 * 1 / 2000 * 9.80665),
      .delay_reset_ms = (800),
      .delay_flashtest_ms = (5),
      .delay_flashbackup_ms = (200),
      .delay_selftest_ms = (80),
      .delay_filter_ms = (1),
      .delay_atti_profile_ms = (1),
    },
  [G365PDF1] =
    {
      .model = G365PDF1,
      .product_id = "G365PDF1",
      .feature_flags = (HAS_ATTITUDE_OUTPUT | HAS_DLT_OUTPUT | HAS_ATTI_ON_REG |
                        HAS_ROT_MATRIX | HAS_RANGE_OVER | HAS_INITIAL_BACKUP),
      .gyro_sf_dps = (1.0 / 66),
      .accl_sf_mg = (1.0 / 2.5),
      .tempc_sf_degc = (-0.0037918),
      .tempc_25c_offset = (2364),
      .rstcnt_sf_micros = (16000),
      .ang_sf_deg = (0.00699411),
      .qtn_sf = (1.0 / (2 << 13)),
      .dlta0_sf_deg = (1.0 / 66 * 1 / 2000),
      .dltv0_sf_mps = (1.0 / 2.5 * 1 / 1000 * 1 / 2000 * 9.80665),
      .delay_reset_ms = (800),
      .delay_flashtest_ms = (5),
      .delay_flashbackup_ms = (200),
      .delay_selftest_ms = (80),
      .delay_filter_ms = (1),
      .delay_atti_profile_ms = (1),
    },
  [G370PDF1] =
    {
      .model = G370PDF1,
      .product_id = "G370PDF1",
      .feature_flags =
        (HAS_DLT_OUTPUT | HAS_ATTI_ON_REG | HAS_ROT_MATRIX |
         HAS_RANGE_OVER | HAS_RTDIAG | HAS_INITIAL_BACKUP),
      .gyro_sf_dps = (1.0 / 66),
      .accl_sf_mg = (1.0 / 2.5),
      .tempc_sf_degc = (-0.0037918),
      .tempc_25c_offset = (2364),
      .rstcnt_sf_micros = (16000),
      .ang_sf_deg = (0),
      .qtn_sf = (0),
      .dlta0_sf_deg = (1.0 / 66 * 1 / 1000),
      .dltv0_sf_mps = (1.0 / 2.5 * 1 / 1000 * 1 / 1000 * 9.80665),
      .delay_reset_ms = (800),
      .delay_flashtest_ms = (5),
      .delay_flashbackup_ms = (200),
      .delay_selftest_ms = (80),
      .delay_filter_ms = (1),
      .delay_atti_profile_ms = (0),
    },
  [G370PDFN] =
    {
      .model = G370PDFN,
      .product_id = "G370PDFN",
      .feature_flags =
        (HAS_DLT_OUTPUT | HAS_ATTI_ON_REG | HAS_ROT_MATRIX |
         HAS_RANGE_OVER | HAS_RTDIAG | HAS_INITIAL_BACKUP),
      .gyro_sf_dps = (1.0 / 66),
      .accl_sf_mg = (1.0 / 2.5),
      .tempc_sf_degc = (-0.0037918),
      .tempc_25c_offset = (2364),
      .rstcnt_sf_micros = (16000),
      .ang_sf_deg = (0),
      .qtn_sf = (0),
      .dlta0_sf_deg = (1.0 / 66 * 1 / 1000),
      .dltv0_sf_mps = (1.0 / 2.5 * 1 / 1000 * 1 / 1000 * 9.80665),
      .delay_reset_ms = (800),
      .delay_flashtest_ms = (5),
      .delay_flashbackup_ms = (200),
      .delay_selftest_ms = (150),
      .delay_filter_ms = (1),
      .delay_atti_profile_ms = (0),
    },
  [G370PDS0] =
    {
      .model = G370PDS0,
      .product_id = "G370PDS0",
      .feature_flags =
        (HAS_DLT_OUTPUT | HAS_ATTI_ON_REG | HAS_ROT_MATRIX |
         HAS_RANGE_OVER | HAS_RTDIAG | HAS_INITIAL_BACKUP),
      .gyro_sf_dps = (1.0 / 150),
      .accl_sf_mg = (1.0 / 2.5),
      .tempc_sf_degc = (-0.0037918),
      .tempc_25c_offset = (2364),
      .rstcnt_sf_micros = (16000),
      .ang_sf_deg = (0),
      .qtn_sf = (0),
      .dlta0_sf_deg = (1.0 / 150 * 1 / 1000),
      .dltv0_sf_mps = (1.0 / 2.5 * 1 / 1000 * 1 / 1000 * 9.80665),
      .delay_reset_ms = (800),
      .delay_flashtest_ms = (5),
      .delay_flashbackup_ms = (200),
      .delay_selftest_ms = (150),
      .delay_filter_ms = (1),
      .delay_atti_profile_ms = (0),
    },
  [G330PDG0] =
    {
      .model = G330PDG0,
      .product_id = "G330PDG0",
      .feature_flags =
        (HAS_ATTITUDE_OUTPUT | HAS_DLT_OUTPUT | HAS_ATTI_ON_REG |
         HAS_ROT_MATRIX | HAS_RANGE_OVER | HAS_ARANGE | HAS_INITIAL_BACKUP),
      .gyro_sf_dps = (1.0 / 66),
      .accl_sf_mg = (1.0 / 4),
      .tempc_sf_degc = (0.00390625),
      .tempc_25c_offset = (0),
      .rstcnt_sf_micros = (16000),
      .ang_sf_deg = (0.00699411),
      .qtn_sf = (1.0 / (2 << 13)),
      .dlta0_sf_deg = (1.0 / 66 * 1 / 2000),
      .dltv0_sf_mps = (1.0 / 4 * 1 / 1000 * 1 / 2000 * 9.80665),
      .delay_reset_ms = (800),
      .delay_flashtest_ms = (30),
      .delay_flashbackup_ms = (200),
      .delay_selftest_ms = (80),
      .delay_filter_ms = (1),
      .delay_atti_profile_ms = (1),
    },
  [G366PDG0] =
    {
      .model = G366PDG0,
      .product_id = "G366PDG0",
      .feature_flags =
        (HAS_ATTITUDE_OUTPUT | HAS_DLT_OUTPUT | HAS_ATTI_ON_REG |
         HAS_ROT_MATRIX | HAS_RANGE_OVER | HAS_ARANGE | HAS_INITIAL_BACKUP),
      .gyro_sf_dps = (1.0 / 66),
      .accl_sf_mg = (1.0 / 4),
      .tempc_sf_degc = (0.00390625),
      .tempc_25c_offset = (0),
      .rstcnt_sf_micros = (16000),
      .ang_sf_deg = (0.00699411),
      .qtn_sf = (1.0 / (2 << 13)),
      .dlta0_sf_deg = (1.0 / 66 * 1 / 2000),
      .dltv0_sf_mps = (1.0 / 4 * 1 / 1000 * 1 / 2000 * 9.80665),
      .delay_reset_ms = (800),
      .delay_flashtest_ms = (30),
      .delay_flashbackup_ms = (200),
      .delay_selftest_ms = (80),
      .delay_filter_ms = (1),
      .delay_atti_profile_ms = (1),
    },
  [G370PDG0] =
    {
      .model = G370PDG0,
      .product_id = "G370PDG0",
      .feature_flags =
        (HAS_DLT_OUTPUT | HAS_ATTI_ON_REG | HAS_ROT_MATRIX |
         HAS_RANGE_OVER | HAS_ARANGE | HAS_INITIAL_BACKUP),
      .gyro_sf_dps = (1.0 / 66),
      .accl_sf_mg = (1.0 / 4),
      .tempc_sf_degc = (0.00390625),
      .tempc_25c_offset = (0),
      .rstcnt_sf_micros = (16000),
      .ang_sf_deg = (0),
      .qtn_sf = (0),
      .dlta0_sf_deg = (1.0 / 66 * 1 / 2000),
      .dltv0_sf_mps = (1.0 / 4 * 1 / 1000 * 1 / 2000 * 9.80665),
      .delay_reset_ms = (800),
      .delay_flashtest_ms = (30),
      .delay_flashbackup_ms = (200),
      .delay_selftest_ms = (80),
      .delay_filter_ms = (1),
      .delay_atti_profile_ms = (0),
    },
  [G370PDT0] =
    {
      .model = G370PDT0,
      .product_id = "G370PDT0",
      .feature_flags =
        (HAS_DLT_OUTPUT | HAS_ATTI_ON_REG | HAS_ROT_MATRIX |
         HAS_RANGE_OVER | HAS_ARANGE | HAS_INITIAL_BACKUP),
      .gyro_sf_dps = (1.0 / 150),
      .accl_sf_mg = (1.0 / 4),
      .tempc_sf_degc = (0.00390625),
      .tempc_25c_offset = (0),
      .rstcnt_sf_micros = (16000),
      .ang_sf_deg = (0),
      .qtn_sf = (0),
      .dlta0_sf_deg = (1.0 / 66 * 1 / 2000),
      .dltv0_sf_mps = (1.0 / 4 * 1 / 1000 * 1 / 2000 * 9.80665),
      .delay_reset_ms = (800),
      .delay_flashtest_ms = (30),
      .delay_flashbackup_ms = (200),
      .delay_selftest_ms = (80),
      .delay_filter_ms = (1),
      .delay_atti_profile_ms = (0),
    },
  [G570PR20] =
    {
      .model = G570PR20,
      .product_id = "G570PR20",
      .feature_flags = (HAS_ROT_MATRIX | HAS_RANGE_OVER | HAS_INITIAL_BACKUP),
      .gyro_sf_dps = (1.0 / 66),
      .accl_sf_mg = (1.0 / 2),
      .tempc_sf_degc = (0.00390625),
      .tempc_25c_offset = (0),
      .rstcnt_sf_micros = (1),
      .ang_sf_deg = (0),
      .qtn_sf = (0),
      .dlta0_sf_deg = (0),
      .dltv0_sf_mps = (0),
      .delay_reset_ms = (5000),
      .delay_flashtest_ms = (0),
      .delay_flashbackup_ms = (200),
      .delay_selftest_ms = (10000),
      .delay_filter_ms = (0),
      .delay_atti_profile_ms = (0),
    },
  [G_UNKNOWN] =
    {
      .model = G_UNKNOWN,
      .product_id = "G_UNKNOWN",
      .feature_flags = 0,
      .gyro_sf_dps = (0),
      .accl_sf_mg = (0),
      .tempc_sf_degc = (0.0),
      .tempc_25c_offset = (0),
      .rstcnt_sf_micros = (0),
      .ang_sf_deg = (0),
      .qtn_sf = (0),
      .dlta0_sf_deg = (0),
      .dltv0_sf_mps = (0),
      .delay_reset_ms = (800),
      .delay_flashtest_ms = (30),
      .delay_flashbackup_ms = (200),
      .delay_selftest_ms = (80),
      .delay_filter_ms = (1),
      .delay_atti_profile_ms = (1),
    },

};

/*****************************************************************************
** Function name:       sensorHWReset
** Description:         Toggle the RESET pin, delay 3secs, wait for NOT_READY
**                      This is only applicable on embedded platforms with
**                      GPIO pin connected to IMU RESET#
** Parameters:          None
** Return value:        OK or NG
*****************************************************************************/
int sensorHWReset(void) {
  unsigned int debug = FALSE;
  unsigned short rxData;
  unsigned short retryCount = 3;

  gpioClr(EPSON_RESET);  // Assert RESET (LOW)
  seDelayMS(EPSON_RESET_LOW_WIDTH_MS);
  gpioSet(EPSON_RESET);  // De-assert RESET (HIGH)
  seDelayMS(EPSON_RESET_DELAY_MS);

  // Poll NOT_READY bit every 1msec until returns 0
  // Exit after specified retries
  do {
    rxData = registerRead16(WIN_ID1, ADDR_GLOB_CMD_LO, debug);
    seDelayMS(1);
    retryCount--;
  } while ((rxData & BIT10) && (retryCount > 0));

  if (retryCount == 0) {
    printf("\r\n...Error: NOT_READY stuck HIGH.");
    return NG;
  }
  return OK;
}

/*****************************************************************************
** Function name:       sensorPowerOn
** Description:         Goto Config mode, check ID register,
**                      Check for Hardware Error Flags
** Parameters:          None
** Return value:        OK or NG
*****************************************************************************/
int sensorPowerOn(void) {
  unsigned short rxData = 0xFFFF;
  const unsigned short ID_VAL = 0x5345;
  unsigned int debug = FALSE;
  unsigned short retryCount = 10;

  do {
    registerWriteByte(WIN_ID0, ADDR_MODE_CTRL_HI, CMD_CONFIG, debug);
    rxData = registerRead16(WIN_ID0, ADDR_MODE_CTRL_LO, debug);
    seDelayMS(200);  // allow some time for device to CONFIG
    retryCount--;
  } while ((rxData & BIT10) == 0 && (retryCount > 0));

  if (retryCount == 0) {
    printf("\r\n...Error: Stuck in Sampling Mode.");
    printf("\r\n...Attempting Hardware Reset if connected.");
    if (sensorHWReset() == NG) {
      return NG;
    }
  }

  rxData = registerRead16(WIN_ID0, ADDR_ID, debug);
  if (rxData != ID_VAL) {
    printf("\r\n...Error: Unexpected ID return value - 0x%04X", rxData);
    printf("\r\n...Attempting Hardware Reset if connected.");
    if (sensorHWReset() == NG) {
      return NG;
    }
  }

  // Check for error flags
  rxData = registerRead16(WIN_ID0, ADDR_DIAG_STAT, debug);
  if (rxData != 0x0000) {
    printf("\r\n...Error: DIAG_STAT return value - 0x%04X", rxData);
    return NG;
  }
  return OK;
}

/*****************************************************************************
** Function name:       sensorStart
** Description:         Start sensor sampling (goto Sampling Mode)
** Parameters:          None
** Return value:        None
*****************************************************************************/
void sensorStart(void) {
  unsigned int debug = FALSE;

  registerWriteByte(WIN_ID0, ADDR_MODE_CTRL_HI, CMD_SAMPLING, debug);
  printf("\r\n...Sensor start.");
}

/*****************************************************************************
** Function name:       sensorStop
** Description:         Stop sensor sampling (goto Config Mode)
**                      assumes currently in Sampling mode & WIN_ID = 0
** Parameters:          None
** Return value:        None
*****************************************************************************/
void sensorStop(void) {
  unsigned int debug = FALSE;

  writeByte(ADDR_MODE_CTRL_HI, CMD_CONFIG, debug);
  seDelayMS(200);  // Provide 200msec for sensor to finish sending sample
#ifndef SPI
  // Flush the receive buffer of incoming residual data
  purgeComPort();
#endif  // #ifndef SPI
  printf("\r\n...Sensor stop.");
}

/*****************************************************************************
** Function name:       sensorReset
** Description:         Send Software Reset to Sensor + Delay 800 msec
** Parameters:          None
** Return value:        None
*****************************************************************************/
void sensorReset(const struct EpsonProperties* esensor) {
  unsigned int debug = FALSE;
  unsigned char SOFT_RST_BIT = 0x80;

  printf("\r\n...Software Reset begin.");
  registerWriteByte(WIN_ID1, ADDR_GLOB_CMD_LO, SOFT_RST_BIT, debug);
  seDelayMS(esensor->delay_reset_ms);
  printf("\r\n...Software Reset complete.");
}

/*****************************************************************************
** Function name:       sensorFlashTest
** Description:         Send Flashtest command to Sensor and check status
**                      NOTE: Not supported for V340PDD0
** Parameters:          None
** Return value:        OK or NG
*****************************************************************************/
int sensorFlashTest(const struct EpsonProperties* esensor) {
  unsigned int debug = FALSE;
  unsigned short rxData;
  unsigned short retryCount = 3000;
  unsigned char FLASH_TEST_BIT = 0x08;

  printf("\r\n...Flash test begin.");
  registerWriteByte(WIN_ID1, ADDR_MSC_CTRL_HI, FLASH_TEST_BIT, debug);
  seDelayMS(esensor->delay_flashtest_ms);
  do {
    rxData = registerRead16(WIN_ID1, ADDR_MSC_CTRL_LO, debug);
    retryCount--;
  } while ((rxData & BIT11) && (retryCount > 0));
  if (retryCount == 0) {
    printf("\r\n...Error: Flashtest bit did not return to 0b.");
    return NG;
  }

  rxData = registerRead16(WIN_ID0, ADDR_DIAG_STAT, debug);
  printf("\r\n...Flash test complete.");

  if ((rxData & BIT2) != 0x0000) return NG;
  return OK;
}

/*****************************************************************************
** Function name:       sensorSelfTest
** Description:         Send SelfTest command to Sensor and check status
** Parameters:          None
** Return value:        OK or NG
*****************************************************************************/
int sensorSelfTest(const struct EpsonProperties* esensor) {
  unsigned int debug = FALSE;
  unsigned short rxData;
  unsigned short retryCount = 3000;
  unsigned char SELF_TEST_BIT = 0x04;

  printf("\r\n...Self test begin.");
  registerWriteByte(WIN_ID1, ADDR_MSC_CTRL_HI, SELF_TEST_BIT, debug);
  seDelayMS(esensor->delay_selftest_ms);
  do {
    rxData = registerRead16(WIN_ID1, ADDR_MSC_CTRL_LO, debug);
    retryCount--;
  } while ((rxData & BIT10) && (retryCount > 0));
  if (retryCount == 0) {
    printf("\r\n...Error: Self test bit did not return to 0b.");
    return NG;
  }

  rxData = registerRead16(WIN_ID0, ADDR_DIAG_STAT, debug);
  printf("\r\n...Self test complete.");

  if ((rxData & 0x7800) != 0) return NG;
  return OK;
}

/*****************************************************************************
** Function name:       sensorInitialBackup
** Description:         Send InitialBackup command (restore defaults) to Sensor
**                      and check status
**                      NOTE: Not supported by all models
** Parameters:          None
** Return value:        OK or NG
*****************************************************************************/
int sensorInitialBackup(const struct EpsonProperties* esensor) {
  if (!(esensor->feature_flags & HAS_INITIAL_BACKUP)) {
    // always return NG since InitialBackup bit does not exist
    return NG;
  }

  unsigned int debug = FALSE;
  unsigned short rxData;
  unsigned short retryCount = 3000;
  unsigned char INITIAL_BACKUP_BIT = 0x10;

  printf("\r\n...InitialBackup begin.");
  registerWriteByte(WIN_ID1, ADDR_GLOB_CMD_LO, INITIAL_BACKUP_BIT, debug);
  seDelayMS(esensor->delay_flashbackup_ms);

  do {
    rxData = registerRead16(WIN_ID1, ADDR_GLOB_CMD_LO, debug);
    retryCount--;
  } while ((rxData & BIT4) && (retryCount > 0));
  if (retryCount == 0) {
    printf("\r\n...Error: InitialBackup bit did not return to 0b.");
    return NG;
  }

  printf("\r\n...Initial Backup complete.");
  return OK;
}

/*****************************************************************************
** Function name:       sensorFlashBackup
** Description:         Send FlashBackup command (save defaults) to Sensor
**                      and check status
** Parameters:          None
** Return value:        OK or NG
*****************************************************************************/
int sensorFlashBackup(const struct EpsonProperties* esensor) {
  unsigned int debug = FALSE;
  unsigned short rxData;
  unsigned short retryCount = 3000;
  unsigned char FLASH_BACKUP_BIT = 0x08;

  printf("\r\n...FlashBackup begin.");
  registerWriteByte(WIN_ID1, ADDR_GLOB_CMD_LO, FLASH_BACKUP_BIT, debug);
  seDelayMS(esensor->delay_flashbackup_ms);

  do {
    rxData = registerRead16(WIN_ID1, ADDR_GLOB_CMD_LO, debug);
    retryCount--;
  } while ((rxData & BIT3) && (retryCount > 0));
  if (retryCount == 0) {
    printf("\r\n...Error: FlashBackup bit did not return to 0b.");
    return NG;
  }

  rxData = registerRead16(WIN_ID0, ADDR_DIAG_STAT, debug);
  printf("\r\n...Flashback complete.");

  if ((rxData & BIT0) != 0) return NG;
  return OK;
}

/*****************************************************************************
** Function name:       sensorDataByteLength
** Description:         Determines the sensor burst read packet data length
**                      based on the parameters in the EpsonOptions struct.
** Parameters:          pointer to options struct describing IMU configuration.
** Return value:        data byte length
*****************************************************************************/
unsigned int sensorDataByteLength(const struct EpsonProperties* esensor,
                                  const struct EpsonOptions* options) {
  unsigned int length = 0;

  // 16 bit ND_EA FLAG
  if (options->flag_out) length += 2;

  // 16 or 32 bit Temperature Output
  if (options->temp_out) {
    if (options->temp_bit)
      length += 4;
    else
      length += 2;
  }

  // 16 or 32 bit Gyro X, Y, Z Output
  if (options->gyro_out) {
    if (options->gyro_bit)
      length += 12;
    else
      length += 6;
  }

  // 16 or 32 bit Accl X, Y, Z Output
  if (options->accel_out) {
    if (options->accel_bit)
      length += 12;
    else
      length += 6;
  }

  if (esensor->feature_flags & HAS_DLT_OUTPUT) {
    // 16 or 32 bit Delta Angle X, Y, Z Output
    if (options->gyro_delta_out) {
      if (options->gyro_delta_bit)
        length += 12;
      else
        length += 6;
    }

    // 16 or 32 bit Delta Velocity X, Y, Z Output
    if (options->accel_delta_out) {
      if (options->accel_delta_bit)
        length += 12;
      else
        length += 6;
    }
  }

  if (esensor->feature_flags & HAS_ATTITUDE_OUTPUT) {
    if (options->qtn_out) {
      if (options->qtn_bit)
        length += 16;
      else
        length += 8;
    }
    // 16 or 32 bit Attitude X, Y, Z Output
    if (options->atti_out) {
      if (options->atti_bit)
        length += 12;
      else
        length += 6;
    }
  }

  // 16 bit GPIO status output
  if (options->gpio_out) {
    length += 2;
  }
  // 16 bit Count output
  if (options->count_out) {
    length += 2;
  }
  // 16 bit Checksum output
  if (options->checksum_out) {
    length += 2;
  }

#ifndef SPI
  // For Start and End byte when using UART interface
  length += 2;
#endif

  return length;
}

/*****************************************************************************
** Function name:       sensorDummyWrite
** Description:         Sets the WINDOW_ID of IMU to 0
**                      This is a workaround to flush the UART port on embedded
**                      Linux platform to prevent hanging if the first register
**                      access is register read
** Parameters:          None
** Return value:        None
*****************************************************************************/
void sensorDummyWrite(void) {
  unsigned int debug = FALSE;

  seDelayMS(100);
  writeByte(ADDR_WIN_CTRL, 0x00, debug);
  seDelayMS(100);
  printf("...sensorDummyWrite...");
}

/*****************************************************************************
** Function name:       getProductId
** Description:         Updates char array with Product ID ASCII
**
** Parameters:          pointer to String
** Return value:        pointer to String
*****************************************************************************/
char* sensorGetProductId(char* pcharArr) {
  unsigned short prod_id1 = registerRead16(WIN_ID1, ADDR_PROD_ID1, FALSE);
  unsigned short prod_id2 = registerRead16(WIN_ID1, ADDR_PROD_ID2, FALSE);
  unsigned short prod_id3 = registerRead16(WIN_ID1, ADDR_PROD_ID3, FALSE);
  unsigned short prod_id4 = registerRead16(WIN_ID1, ADDR_PROD_ID4, FALSE);

  pcharArr[0] = (char)prod_id1;
  pcharArr[1] = (char)(prod_id1 >> 8);
  pcharArr[2] = (char)prod_id2;
  pcharArr[3] = (char)(prod_id2 >> 8);
  pcharArr[4] = (char)prod_id3;
  pcharArr[5] = (char)(prod_id3 >> 8);
  pcharArr[6] = (char)prod_id4;
  pcharArr[7] = (char)(prod_id4 >> 8);
  pcharArr[8] = '\0';

  return (pcharArr);
}

/*****************************************************************************
** Function name:       getSerialId
** Description:         Updates char array with Serial ID ASCII
**
** Parameters:          pointer to String
** Return value:        pointer to String
*****************************************************************************/
char* sensorGetSerialId(char* pcharArr) {
  unsigned short ser_num1 = registerRead16(WIN_ID1, ADDR_SERIAL_NUM1, FALSE);
  unsigned short ser_num2 = registerRead16(WIN_ID1, ADDR_SERIAL_NUM2, FALSE);
  unsigned short ser_num3 = registerRead16(WIN_ID1, ADDR_SERIAL_NUM3, FALSE);
  unsigned short ser_num4 = registerRead16(WIN_ID1, ADDR_SERIAL_NUM4, FALSE);

  pcharArr[0] = (char)ser_num1;
  pcharArr[1] = (char)(ser_num1 >> 8);
  pcharArr[2] = (char)ser_num2;
  pcharArr[3] = (char)(ser_num2 >> 8);
  pcharArr[4] = (char)ser_num3;
  pcharArr[5] = (char)(ser_num3 >> 8);
  pcharArr[6] = (char)ser_num4;
  pcharArr[7] = (char)(ser_num4 >> 8);
  pcharArr[8] = '\0';

  return pcharArr;
}

/*****************************************************************************
** Function name:       sensorInitOptions
** Description:         Initialize the sensor hardware to desired settings
**                      based on EpsonOptions
** Parameters:          struct EpsonOptions
** Return value:        OK or NG
**
*****************************************************************************/
int sensorInitOptions(const struct EpsonProperties* esensor,
                      struct EpsonOptions* options) {
  unsigned int debug = FALSE;

  // Disable attitude/quaternion output if not supported
  if (!(esensor->feature_flags & HAS_ATTITUDE_OUTPUT)) {
    options->atti_out = 0;
    options->qtn_out = 0;
  }
  // Disable delta output if not supported
  if (!(esensor->feature_flags & HAS_DLT_OUTPUT)) {
    options->gyro_delta_out = 0;
    options->accel_delta_out = 0;
  }
  // Disable attitude output if delta output enabled
  if (options->gyro_delta_out | options->accel_delta_out) {
    options->atti_out = 0;
    options->qtn_out = 0;
  }

  // SIG_CTRL
  // ND flags for gyro_delta_out X,Y,Z are enabled if gyro_delta_out is enabled
  // ND flags for accel_delta_out X,Y,Z are enabled if accel_delta_out is
  // enabled
  int sig_ctrl_lo = 0;
  if (esensor->feature_flags & HAS_DLT_OUTPUT) {
    sig_ctrl_lo = (options->accel_delta_out & 0x01) << 2 |
                  (options->accel_delta_out & 0x01) << 3 |
                  (options->accel_delta_out & 0x01) << 4 |
                  (options->gyro_delta_out & 0x01) << 5 |
                  (options->gyro_delta_out & 0x01) << 6 |
                  (options->gyro_delta_out & 0x01) << 7;
  }

  // ND flags for gyro_out X,Y,Z are enabled if gyro_out is enabled
  // ND flags for accel_out X,Y,Z are enabled if accel_out is enabled
  // ND flag for temp_out is enabled if temp_out is enabled
  int sig_ctrl_hi =
    (options->accel_out & 0x01) << 1 | (options->accel_out & 0x01) << 2 |
    (options->accel_out & 0x01) << 3 | (options->gyro_out & 0x01) << 4 |
    (options->gyro_out & 0x01) << 5 | (options->gyro_out & 0x01) << 6 |
    (options->temp_out & 0x01) << 7;

  // MSC_CTRL
  // Configure DRDY function (if needed) & EXT pin function on GPIO2 (if needed)
  // External Counter Reset is typically used when GPIO2 is connected to a
  // PPS-like signal
  int msc_ctrl_lo = 0;
  if (esensor->model == G570PR20) {
    msc_ctrl_lo =
      ((options->drdy_pol & 0x01) << 1) | ((options->drdy_on & 0x01) << 2);
  } else {
    msc_ctrl_lo =
      (options->drdy_pol & 0x01) << 1 | (options->drdy_on & 0x01) << 2 |
      (options->ext_pol & 0x01) << 5 | (options->ext_sel & 0x03) << 6;
  }

  // SMPL_CTRL
  // Configures the Data Output Rate of the IMU.
  // Refer to Datasheet for valid Data Output Rate & Filter Setting combinations
  int smpl_ctrl_hi = (options->dout_rate & 0x0F);

  // FILTER_CTRL
  // Configures the FIR filter of the IMU.
  // Refer to Datasheet for valid Data Output Rate & Filter Setting combinations
  int filter_ctrl_lo = (options->filter_sel & 0x1F);

  // BURST_CTRL1
  // These enable or disable certain data fields in the burst read packet
  // For G330 or G366 specifically, attitude output can be enabled or disabled.
  int burst_ctrl1_lo = (options->checksum_out & 0x1) |
                       (options->count_out & 0x1) << 1 |
                       (options->gpio_out & 0x01) << 2;

  int burst_ctrl1_hi = 0;
  if (esensor->feature_flags & HAS_ATTITUDE_OUTPUT) {
    burst_ctrl1_hi |=
      ((options->atti_out & 0x1) | (options->qtn_out & 0x1) << 1);
  }
  if (esensor->feature_flags & HAS_DLT_OUTPUT) {
    burst_ctrl1_hi |= ((options->accel_delta_out & 0x1) << 2 |
                       (options->gyro_delta_out & 0x01) << 3);
  }
  burst_ctrl1_hi |=
    ((options->accel_out & 0x01) << 4 | (options->gyro_out & 0x01) << 5 |
     (options->temp_out & 0x01) << 6 | (options->flag_out & 0x01) << 7);

  // BURST_CTRL2
  // If certain data fields are enabled, these bits determine if the
  // data fields are 16 or 32 bit
  int burst_ctrl2_hi = 0;
  if (esensor->feature_flags & HAS_ATTITUDE_OUTPUT) {
    burst_ctrl2_hi |=
      ((options->atti_bit & 0x1) | (options->qtn_bit & 0x01) << 1);
  }
  if (esensor->feature_flags & HAS_DLT_OUTPUT) {
    burst_ctrl2_hi |= ((options->accel_delta_bit & 0x01) << 2 |
                       (options->gyro_delta_bit & 0x01) << 3);
  }
  burst_ctrl2_hi |=
    ((options->accel_bit & 0x01) << 4 | (options->gyro_bit & 0x01) << 5 |
     (options->temp_bit & 0x01) << 6);

  // POL_CTRL
  // If these bits are set, then the axis values are reverse polarity
  int pol_ctrl_lo =
    (options->invert_zaccel & 0x01) << 1 |
    (options->invert_yaccel & 0x01) << 2 |
    (options->invert_xaccel & 0x01) << 3 | (options->invert_zgyro & 0x01) << 4 |
    (options->invert_ygyro & 0x01) << 5 | (options->invert_xgyro & 0x01) << 6;

  // DLT_CTRL
  // Enable or disable Delta Angle/Velocity overflow flag in DIAG_STAT
  // Set A_RANGE_CTRL bit if device supports it
  // Set the Delta Angle/Velocity Scale Factor
  int dlt_ctrl_hi = 0;
  if (esensor->feature_flags & HAS_ARANGE) {
    dlt_ctrl_hi = (options->a_range_ctrl & 0x01);
  }

  int dlt_ctrl_lo = 0;
  if (esensor->feature_flags & HAS_DLT_OUTPUT) {
    dlt_ctrl_lo = (options->dlta_range_ctrl & 0x0F) << 4 |
                  (options->dltv_range_ctrl & 0x0F);
  }

  // ATTI_CTRL
  // Attitude Output & Delta Angle/Velocity functions are mutually exclusive.
  // User should only enable one or the other, but not both.
  // Attitude Mode = 0=Inclination 1=Euler
  int atti_ctrl_hi = 0;
  int atti_ctrl_lo = 0;
  if (esensor->feature_flags & HAS_ATTITUDE_OUTPUT) {
    atti_ctrl_hi |= (((options->atti_out | options->qtn_out) & 0x01) << 2 |
                     (options->atti_mode & 0x01) << 3);

    // Refer to datasheet to determine the different Euler Angle configurations
    if (!options->qtn_out) {
      atti_ctrl_lo = (options->atti_conv & 0x1f);
    }
  }
  if ((esensor->feature_flags & (HAS_DLT_OUTPUT | HAS_ATTI_ON_REG)) ==
      (HAS_DLT_OUTPUT | HAS_ATTI_ON_REG)) {
    atti_ctrl_hi |=
      (((options->gyro_delta_out & 0x01) | (options->accel_delta_out & 0x01))
       << 1);
  }

  // GLOB_CMD2
  // Setting Attitude Motion Profile
  int glob_cmd2_lo = 0;
  if (esensor->feature_flags & HAS_ATTITUDE_OUTPUT) {
    glob_cmd2_lo = (options->atti_profile & 0x03) << 4;
  }
  if (!(esensor->model == G570PR20)) {
    registerWriteByte(WIN_ID1, ADDR_SIG_CTRL_HI, sig_ctrl_hi, debug);
    registerWriteByte(WIN_ID1, ADDR_SIG_CTRL_LO, sig_ctrl_lo, debug);
  }
  registerWriteByte(WIN_ID1, ADDR_MSC_CTRL_LO, msc_ctrl_lo, debug);
  registerWriteByte(WIN_ID1, ADDR_SMPL_CTRL_HI, smpl_ctrl_hi, debug);
  registerWriteByte(WIN_ID1, ADDR_FILTER_CTRL_LO, filter_ctrl_lo, debug);
  seDelayMS(esensor->delay_filter_ms);

  // Check that the FILTER_BUSY bit returns 0
  unsigned short rxData;
  unsigned short retryCount = 3000;
  do {
    rxData = registerRead16(WIN_ID1, ADDR_FILTER_CTRL_LO, debug);
    retryCount--;
  } while ((rxData & BIT5) && (retryCount > 0));

  if (retryCount == 0) {
    printf("\r\n...Error: Filter busy bit did not return to 0b.");
    return NG;
  }

#ifdef SPI
  // Always disable UART_AUTO mode when using SPI IF
  registerWriteByte(WIN_ID1, ADDR_UART_CTRL_LO, 0x00, debug);
#else
  // Always enable UART_AUTO mode when using UART IF
  registerWriteByte(WIN_ID1, ADDR_UART_CTRL_LO, 0x01, debug);
#endif

  registerWriteByte(WIN_ID1, ADDR_BURST_CTRL1_LO, burst_ctrl1_lo, debug);
  registerWriteByte(WIN_ID1, ADDR_BURST_CTRL1_HI, burst_ctrl1_hi, debug);
  registerWriteByte(WIN_ID1, ADDR_BURST_CTRL2_HI, burst_ctrl2_hi, debug);
  registerWriteByte(WIN_ID1, ADDR_POL_CTRL_LO, pol_ctrl_lo, debug);

  if (esensor->feature_flags & HAS_ARANGE) {
    registerWriteByte(WIN_ID1, ADDR_DLT_CTRL_HI, dlt_ctrl_hi, debug);
  }
  if (esensor->feature_flags & HAS_DLT_OUTPUT) {
    registerWriteByte(WIN_ID1, ADDR_DLT_CTRL_LO, dlt_ctrl_lo, debug);
  }
  if (esensor->feature_flags & HAS_ATTI_ON_REG) {
    registerWriteByte(WIN_ID1, ADDR_ATTI_CTRL_HI, atti_ctrl_hi, debug);
  }
  if (esensor->feature_flags & HAS_ATTITUDE_OUTPUT) {
    registerWriteByte(WIN_ID1, ADDR_ATTI_CTRL_LO, atti_ctrl_lo, debug);

    registerWriteByte(WIN_ID1, ADDR_GLOB_CMD2_LO, glob_cmd2_lo, debug);
    seDelayMS(esensor->delay_atti_profile_ms);

    // Check that the ATTI_MOTION_PROFILE_STAT bit returns 0
    retryCount = 3000;
    do {
      rxData = registerRead16(WIN_ID1, ADDR_GLOB_CMD2_LO, debug);
      retryCount--;
    } while ((rxData & BIT6) && (retryCount > 0));

    if (retryCount == 0) {
      printf(
        "\r\n...Error: ATTI_MOTION_PROFILE_STAT bit did not return to 0b.");
      return NG;
    }
  }
  return OK;
}

/*****************************************************************************
** Function name:       sensorDumpRegisters
** Description:         Read all registers for debug purpose
** Parameters:          None
** Return value:        None
*****************************************************************************/
void sensorDumpRegisters(const struct EpsonProperties* esensor) {
  unsigned int debug = TRUE;
  printf("\r\nRegister Dump:\r\n");
  registerRead16(WIN_ID0, ADDR_MODE_CTRL_LO, debug);
  registerRead16(WIN_ID0, ADDR_DIAG_STAT, debug);
  registerRead16(WIN_ID0, ADDR_FLAG, debug);
  printf("\r\n");
  registerRead16(WIN_ID0, ADDR_GPIO, debug);

  if (esensor->model != G570PR20) {
    registerRead16(WIN_ID0, ADDR_COUNT, debug);
  }

  if (esensor->feature_flags & HAS_RANGE_OVER) {
    registerRead16(WIN_ID0, ADDR_RANGE_OVER, debug);
  }

  if (esensor->model != G570PR20) {
    printf("\r\n");
    registerRead16(WIN_ID0, ADDR_TEMP_HIGH, debug);
    registerRead16(WIN_ID0, ADDR_TEMP_LOW, debug);
    registerRead16(WIN_ID0, ADDR_XGYRO_HIGH, debug);
    printf("\r\n");
    registerRead16(WIN_ID0, ADDR_XGYRO_LOW, debug);
    registerRead16(WIN_ID0, ADDR_YGYRO_HIGH, debug);
    registerRead16(WIN_ID0, ADDR_YGYRO_LOW, debug);
    printf("\r\n");
    registerRead16(WIN_ID0, ADDR_ZGYRO_HIGH, debug);
    registerRead16(WIN_ID0, ADDR_ZGYRO_LOW, debug);
    registerRead16(WIN_ID0, ADDR_XACCL_HIGH, debug);
    printf("\r\n");
    registerRead16(WIN_ID0, ADDR_XACCL_LOW, debug);
    registerRead16(WIN_ID0, ADDR_YACCL_HIGH, debug);
    registerRead16(WIN_ID0, ADDR_YACCL_LOW, debug);
    printf("\r\n");
    registerRead16(WIN_ID0, ADDR_ZACCL_HIGH, debug);
    registerRead16(WIN_ID0, ADDR_ZACCL_LOW, debug);
  }

  if (esensor->feature_flags & HAS_RTDIAG) {
    registerRead16(WIN_ID0, ADDR_RT_DIAG, debug);
  }
  printf("\r\n");
  registerRead16(WIN_ID0, ADDR_ID, debug);
  printf("\r\n");

  // Output only QTN because ANG and DLTA share the same registers
  if (esensor->feature_flags & HAS_ATTITUDE_OUTPUT) {
    registerRead16(WIN_ID0, ADDR_QTN0_HIGH, debug);
    registerRead16(WIN_ID0, ADDR_QTN0_LOW, debug);
    registerRead16(WIN_ID0, ADDR_QTN1_HIGH, debug);
    printf("\r\n");
    registerRead16(WIN_ID0, ADDR_QTN1_LOW, debug);
    registerRead16(WIN_ID0, ADDR_QTN2_HIGH, debug);
    registerRead16(WIN_ID0, ADDR_QTN2_LOW, debug);
    printf("\r\n");
    registerRead16(WIN_ID0, ADDR_QTN3_HIGH, debug);
    registerRead16(WIN_ID0, ADDR_QTN3_LOW, debug);
    printf("\r\n");
  }

  if (esensor->feature_flags & HAS_DLT_OUTPUT) {
    registerRead16(WIN_ID0, ADDR_XDLTA_HIGH, debug);
    registerRead16(WIN_ID0, ADDR_XDLTA_LOW, debug);
    registerRead16(WIN_ID0, ADDR_YDLTA_HIGH, debug);
    printf("\r\n");
    registerRead16(WIN_ID0, ADDR_YDLTA_LOW, debug);
    registerRead16(WIN_ID0, ADDR_ZDLTA_HIGH, debug);
    registerRead16(WIN_ID0, ADDR_ZDLTA_LOW, debug);
    printf("\r\n");
    registerRead16(WIN_ID0, ADDR_XDLTV_HIGH, debug);
    registerRead16(WIN_ID0, ADDR_XDLTV_LOW, debug);
    registerRead16(WIN_ID0, ADDR_YDLTV_HIGH, debug);
    printf("\r\n");
    registerRead16(WIN_ID0, ADDR_YDLTV_LOW, debug);
    registerRead16(WIN_ID0, ADDR_ZDLTV_HIGH, debug);
    registerRead16(WIN_ID0, ADDR_ZDLTV_LOW, debug);
  }
  printf("\r\n\r\n");
  if (esensor->model != G570PR20) {
    registerRead16(WIN_ID1, ADDR_SIG_CTRL_LO, debug);
  }
  registerRead16(WIN_ID1, ADDR_MSC_CTRL_LO, debug);
  registerRead16(WIN_ID1, ADDR_SMPL_CTRL_LO, debug);
  printf("\r\n");
  registerRead16(WIN_ID1, ADDR_FILTER_CTRL_LO, debug);
  registerRead16(WIN_ID1, ADDR_UART_CTRL_LO, debug);
  registerRead16(WIN_ID1, ADDR_GLOB_CMD_LO, debug);
  printf("\r\n");
  registerRead16(WIN_ID1, ADDR_BURST_CTRL1_LO, debug);
  registerRead16(WIN_ID1, ADDR_BURST_CTRL2_LO, debug);
  registerRead16(WIN_ID1, ADDR_POL_CTRL_LO, debug);

  if ((esensor->feature_flags & HAS_DLT_OUTPUT) |
      (esensor->feature_flags & HAS_ARANGE)) {
    printf("\r\n");
    registerRead16(WIN_ID1, ADDR_GLOB_CMD3_LO, debug);
  }

  if (esensor->feature_flags & HAS_ATTI_ON_REG) {
    registerRead16(WIN_ID1, ADDR_ATTI_CTRL_LO, debug);
  }
  if (esensor->feature_flags & HAS_ATTITUDE_OUTPUT) {
    registerRead16(WIN_ID1, ADDR_GLOB_CMD2_LO, debug);
  }

  printf("\r\n");
  registerRead16(WIN_ID1, ADDR_PROD_ID1, debug);
  registerRead16(WIN_ID1, ADDR_PROD_ID2, debug);
  registerRead16(WIN_ID1, ADDR_PROD_ID3, debug);
  printf("\r\n");
  registerRead16(WIN_ID1, ADDR_PROD_ID4, debug);
  registerRead16(WIN_ID1, ADDR_VERSION, debug);
  registerRead16(WIN_ID1, ADDR_SERIAL_NUM1, debug);
  printf("\r\n");
  registerRead16(WIN_ID1, ADDR_SERIAL_NUM2, debug);
  registerRead16(WIN_ID1, ADDR_SERIAL_NUM3, debug);
  registerRead16(WIN_ID1, ADDR_SERIAL_NUM4, debug);
  printf("\r\n");
  registerRead16(WIN_ID1, ADDR_WIN_CTRL, debug);
  printf("\r\n");
}

/*****************************************************************************
** Function name:       sensorGetDeviceModel
** Description:         Identifies Epson device attached
** Parameters:          pointer to EpsonProperties struct
**                      pointer to string for storing 8 ASCII Epson Product ID
**                      pointer to string for storing 8 ASCII Epson Serial ID
** Return value:        OK or NG
*****************************************************************************/
int sensorGetDeviceModel(struct EpsonProperties* esensor, char* prod_id,
                         char* serial_id) {
  int i;

  printf("\r\nReading device model and serial number...");
  sensorGetProductId(prod_id);
  sensorGetSerialId(serial_id);
  for (i = G_EMPTY; i < G_UNKNOWN; i++) {
    if (strcmp(epson_sensors[i].product_id, prod_id) == 0) break;
  }

  if ((i == G_EMPTY) || (i == G_UNKNOWN)) {
    printf("\r\nERROR: Could not identify product id.");
    return NG;
  }

  // Print out which model executable was compiled and identify model
  printf("\r\nPRODUCT ID:\t%s", prod_id);
  printf("\r\nSERIAL ID:\t%s", serial_id);
  *esensor = epson_sensors[i];

  return OK;
}
