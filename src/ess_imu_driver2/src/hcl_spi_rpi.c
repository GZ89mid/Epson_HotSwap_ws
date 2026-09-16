//==============================================================================
//
//  hcl_spi_rpi.c - Seiko Epson Hardware Control Library
//
//  This layer of indirection is for the Raspberry Pi specific implementation
//  of the SPI protocol. It uses the wiringPi library for the low-level SPI
//  transfers.
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
#include <unistd.h>
#include <wiringPiSPI.h>

#include "hcl.h"
#include "hcl_spi.h"
#include "sensor_epsonCommon.h"

static int mySpiFd;
int deviceOk(void);

/*****************************************************************************
** Function name:       spiInit
** Description:         Initialize the RPI SPI library.
**
** NOTE: This function assumes that the seInit function has been called first
** and initialized the wiringPi library. Then the gpioInit called and setup all
** the necessary pins. CS should be HIGH in case of any glitches on the SPI bus
** during its setup. Attempts to perform low-level check of device ID register
** for expected response for Epson device.
**
** Parameters:          SPI Mode, SPI clock speed
** Return value:        OK, or NG
*****************************************************************************/
int spiInit(uint8_t mode, uint32_t khzspeed) {
  if (mode != SPI_MODE3) {
    return NG;
  }

  if ((mySpiFd = wiringPiSPISetupMode(SPI_CHAN, khzspeed, mode)) < 0) {
    return NG;
  }
  if (!deviceOk()) {
    printf("Invalid device response\n");
    return NG;
  }
  return OK;
}

/*****************************************************************************
** Function name:       spiRelease
** Description:         Release the SPI Interface.
**
** Parameters:          None
** Return value:        OK
*****************************************************************************/
int spiRelease(void) {
  close(mySpiFd);
  return OK;
}

/*****************************************************************************
** Function name:       spiTransfer
** Description:         Initiates an 8-bit SPI transfer
**                      Transfers out 8-bit on MOSI
**                      Transfers in 8-bit on MISO
** Parameters:          8-bit Data to Send to SPI Slave
** Return value:        8-bit Data Received from SPI Slave
*****************************************************************************/
uint8_t spiTransfer(uint8_t value) {
  wiringPiSPIDataRW(SPI_CHAN, &value, 1);
  return value;
}

/*****************************************************************************
** Function name:       deviceOk
** Description:         (Optional) Checks for Epson device.
**                      In case the SPI interface to device
**                      was left in inconsistent state.
** Parameters:          None
** Return value:        OK or NG
*****************************************************************************/
int deviceOk(void) {
  int retry_count = 5;
  const unsigned short ID_VAL_ = 0x5345;
  unsigned short rxData[] = {0x00, 0x00};

  while (retry_count > 0) {
    // Set WIN_ID = 0
    selEpson();
    spiTransfer(ADDR_WIN_CTRL | 0x80);
    spiTransfer(WIN_ID0);
    deselEpson();
    epsonStall();

    // Read ID Register
    selEpson();
    spiTransfer(ADDR_ID);
    spiTransfer(0x00);
    epsonStall();
    ;

    // Response
    rxData[0] = spiTransfer(0x00);
    rxData[1] = spiTransfer(0x00);
    deselEpson();
    epsonStall();
    if ((rxData[0] << 8 | rxData[1]) == ID_VAL_) {
      return OK;
    }
    retry_count--;
  }
  return NG;
}
