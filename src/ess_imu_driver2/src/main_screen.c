//==============================================================================
//
//  main_screen.c - Epson IMU sensor test application
//                   - This program initializes the Epson IMU and
//                     sends sensor output to console
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

#ifdef SPI
#else
#include <termios.h>
#endif  // SPI

#include "hcl.h"
#include "hcl_gpio.h"

#ifdef SPI
#include "hcl_spi.h"
#else
#include "hcl_uart.h"
#endif  // SPI

#include "main_helper.h"
#include "sensor_epsonCommon.h"

#ifdef SPI
#else

// Modify below as needed for hardware
const char *IMUSERIAL = "/dev/ttyUSB0";
#endif  // SPI

// Specify the number of samples to readout before exiting the program
const unsigned int NUM_SAMPLES = 1000;

int main(int argc, char *argv[]) {
  char prod_id[9];  // Device Product ID
  char ser_id[9];   // Device Serial ID
  struct EpsonProperties epson_sensor = epson_sensors[G_UNKNOWN];

  // Specify IMU options
  struct EpsonOptions epson_options = {
    .ext_sel = 1,  // 0 = Sample Counter 1=Reset Counter 2=External Trigger
    .ext_pol = 0,
#ifdef SPI
    .drdy_on = 1,
#endif  // SPI
    .drdy_pol = 1,
    .dout_rate = CMD_RATE125,
    .filter_sel = CMD_FLTAP32,
    .flag_out = 1,
    .temp_out = 1,
    .gyro_out = 1,
    .accel_out = 1,
    .gyro_delta_out = 0,
    .accel_delta_out = 0,
    .qtn_out = 0,   // Only valid for devices that support attitude output
    .atti_out = 0,  // Only valid for devices that support attitude output
    .count_out = 1,
    .checksum_out = 1,

    // Set 0=16bit 1=32bit sensor output.
    // These only have effect if above related "_out = 1"
    .temp_bit = 1,
    .gyro_bit = 1,
    .accel_bit = 1,
    .gyro_delta_bit = 1,
    .accel_delta_bit = 1,
    .qtn_bit = 1,
    .atti_bit = 1,

    .dlta_range_ctrl = 8,
    .dltv_range_ctrl = 8,

    // NOTE: The following are only valid when attitude output is enabled
    .atti_mode = 1,    // 0=Inclination mode 1=Euler mode
    .atti_conv = 0,    // Attitude Conversion Mode, must be 0 when quaternion
                       // output is enabled
    .atti_profile = 0  // Attitude Motion Profile 0=modeA 1=modeB 2=modeC
  };

  // Stores the post-processed sensor data
  struct EpsonData epson_data;

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

  // Initialize sensor with desired settings
  printf("\r\nInitializing sensor...");
  if (!sensorInitOptions(&epson_sensor, &epson_options)) {
    printf("\r\nError: could not initialize Epson Sensor. Exiting...\r\n");

#ifdef SPI
    spiRelease();
#else
    uartRelease();
#endif

    gpioRelease();
    seRelease();
    return -1;
  }
  printf("...Epson IMU initialized.\r\n");

  // Initialize text files for data logs
  const time_t date =
    time(NULL);  // Functions for obtaining and printing time and date

  printf("Date: %s", ctime(&date));
  printf("...Epson IMU Logging.\r\n");
  sensorStart();
  printHeaderRow(stdout, &epson_options);

  unsigned int sample = 0;
  while (sample < (NUM_SAMPLES - 1)) {
    if (sensorDataReadBurstNOptions(&epson_sensor, &epson_options,
                                    &epson_data) == OK) {
      printSensorRow(stdout, &epson_options, &epson_data, sample);
      sample++;
    }
  }

  const time_t end =
    time(NULL);  // Functions for obtaining and printing time and data

  printf("\r\nEnd: ");
  printf("%s", ctime(&end));

  sensorStop();
  seDelayMS(1000);

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
