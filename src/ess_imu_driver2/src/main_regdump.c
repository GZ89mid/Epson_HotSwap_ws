//==============================================================================
//
//  main_regdump.c - Epson IMU sensor test application
//                 - This program reads all registers values for debug purpose
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
#include <time.h>

#ifndef SPI
#include <termios.h>
#endif  // !SPI

#include "hcl.h"
#include "hcl_gpio.h"

#ifdef SPI
#include "hcl_spi.h"
#else
#include "hcl_uart.h"
#endif  // SPI

#include "sensor_epsonCommon.h"

#ifndef SPI
// Modify below as needed for hardware
const char* IMUSERIAL = "/dev/ttyUSB0";
#endif  // !SPI

int main(int argc, char* argv[]) {
  char prod_id[9];  // Device Product ID
  char ser_id[9];   // Device Serial ID
  struct EpsonProperties epson_sensor = epson_sensors[G_UNKNOWN];

  // 1) Initialize the Seiko Epson HCL layer
  printf("\r\nInitializing HCL layer...");
  if (!seInit()) {
    printf(
      "\r\nError: could not initialize the Seiko Epson HCL layer. "
      "Exiting...\r\n");
    return -1;
  }
  printf("...done.\r\n");

  // 2) Initialize the GPIO interfaces, For GPIO control of pins SPI CS, RESET,
  // DRDY
  printf("\r\nInitializing GPIO interface...");
  if (!gpioInit()) {
    printf("\r\nError: could not initialize the GPIO layer. Exiting...\r\n");
    seRelease();
    return -1;
  }
  printf("...done.\r\n");

#ifdef SPI
  // 3) Initialize SPI Interface
  printf("\r\nInitializing SPI interface...");
  // The max SPI clock rate is 1MHz for burst reads in Epson IMUs
  if (!spiInit(SPI_MODE3, 1000000)) {
    printf("\r\nError: could not initialize SPI interface. Exiting...\r\n");
    gpioRelease();
    seRelease();
    return -1;
  }
  sensorDummyWrite();
#else
  // 3) Initialize UART Interface
  //    The baudrate value should be set the the same setting as currently
  //    flashed value in the IMU UART_CTRL BAUD_RATE register
  printf("\r\nInitializing UART interface...");
  if (!uartInit(IMUSERIAL, BAUD_460800)) {
    printf("\r\nError: could not initialize UART interface. Exiting...\r\n");
    gpioRelease();
    seRelease();
    return -1;
  }
#endif  // SPI

  printf("...done.\r\n");

  // 4) Power on sequence - force sensor to config mode, read ID and
  //    check for errors

  printf("\r\nSensor starting up...");
  if (!sensorPowerOn()) {
    printf("\r\nError: failed to power on sensor. Exiting...\r\n");

#ifdef SPI
    spiRelease();
#else
    uartRelease();
#endif

    gpioRelease();
    seRelease();
    return -1;
  }
  printf("...done.\r\n");

  // Auto-Detect Epson Sensor Model Properties
  printf("\r\nDetecting sensor model...");
  if (!sensorGetDeviceModel(&epson_sensor, prod_id, ser_id)) {
    printf("\r\nError: could not detect Epson Sensor. Exiting...\r\n");

#ifdef SPI
    spiRelease();
#else
    uartRelease();
#endif

    gpioRelease();
    seRelease();
    return -1;
  }

  sensorDumpRegisters(&epson_sensor);

#ifdef SPI
  spiRelease();
#else
  uartRelease();
#endif
  gpioRelease();
  seRelease();
  printf("\r\n");

  return 0;
}
