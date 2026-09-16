# README for Epson IMU Linux C Driver using UART or SPI interface

<!---toc start-->

- [README for Epson IMU Linux C Driver using UART or SPI interface](#readme-for-epson-imu-linux-c-driver-using-uart-or-spi-interface)
- [Disclaimer](#disclaimer)
- [Test machines](#test-machines)
- [Requirements](#requirements)
- [When using the UART Interface](#when-using-the-uart-interface)
- [When using SPI Interface on RaspberryPi](#when-using-spi-interface-on-raspberrypi)
- [Compiling the software](#compiling-the-software)
- [Important for configuring delays](#important-for-configuring-delays)
- [Important for GPIO usage](#important-for-gpio-usage)
- [How to run the program](#how-to-run-the-program)
- [File listing](#file-listing)
- [Change record:](#change-record)

<!---toc end-->

# Disclaimer

THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT,
SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE OR CLAIM, ARISING FROM OR
IN CONNECTION WITH THE SOFTWARE OR THE USE OF THE SOFTWARE.

# Test machines

- UART Interface

  - Ubuntu 20.04 Mate running in Oracle VirtualBox on Core i7 Win10 PC
  - Ubuntu 22.04 Mate on Core i7 PC
  - Raspberry Pi 3B+ (with Epson IMU USB evalboard)

- SPI Interface

  - RaspberryPi 3B+ with Ubuntu Mate-20.04-desktop-armhf
  - RaspberryPi 4 with Ubuntu Mate-22.04-desktop-arm64
  - [Unofficial wiringPi](https://github.com/WiringPi/WiringPi/)

# Requirements

- Standard GCC/G++/GNU Make tools are necessary to build this software.
- Ensure you have the latest updates for your Linux system.

```
sudo apt install build-essential
sudo apt-get update
sudo apt-get upgrade
```

# When using the UART Interface

- For using IMU UART interface, this software should compile and work on any generic unix (POSIX) serial port with gcc or g++
- The application assumes that the Epson IMU is connected to serial tty (UART) either through USB evaluation board or directly to the UART port on an embedded system
- Edit the main_xxx source files to specify the correct serial port on host system that the Epson device is attached to

```
const char *IMUSERIAL = "/dev/ttyxxx";
```

# When using SPI Interface on RaspberryPi

1. If the SPI interface is not enabled, use `raspi-config` utility to enable the SPI interface:

```
sudo raspi-config
```

2. Choose the Advanced Options -> SPI -> enable SPI kernel module to be loaded by default "Yes"

```
sudo reboot
```

3. Verify the SPI interface is enabled

```
lsmod | grep spi\_
ls /dev | grep spi
```

4. Install the wiringPi library
   This should already be installed if running a RPI-specific Linux distro, otherwise go to [Unofficial wiringPi](https://github.com/WiringPi/WiringPi/) for installation

The software is initially configured with the following pin mapping for the SPI, DRDY, and RESET (which can be modified by the user):

| Epson IMU   | Raspberry Pi            |
| ----------- | ----------------------- |
| EPSON_RESET | RPI_GPIO_P1_15 (GPIO22) |
| EPSON_DRDY  | RPI_GPIO_P1_18 (GPIO24) |
| EPSON_CS    | RPI_GPIO_P1_16 (GPIO23) |
| SPI_SCK     | RPI_GPIO_P1_23 (GPIO11) |
| SPI_MISO    | RPI_GPIO_P1_21 (GPIO9)  |
| SPI_MOSI    | RPI_GPIO_P1_19 (GPIO10) |

# Compiling the software

Please specify the # of samples to capture in main_csvlogger.c or main_screen.c.

```
const unsigned int NUM_SAMPLES = xxxx;
```

1. Run `make clean`. This is recommended to remove previous build files before creating new builds
2. Run `make` specifying a `target`, `IF=`, and `PLATFORM=` parameters
   - Supported `target` options are:

     - `screen`
     - `csvlogger`
     - `regdump`
     - `all` (This is default if `target` not specified)

   - Supported `IF=` options are:

     - `UART` (This is default if `IF=` is not specified)
     - `SPI` (If `IF=SPI` is specified, then `PLATFORM=RPI` is automatically specified)

   - Supported `PLATFORM=` options are:

     - `NONE` (This is default if `PLATFORM=` is not specified, which implies standard PC without GPIO support)
     - `RPI` (This is necessary to enable GPIO functions in the build, i.e. RESET and DRDY)

   - Example commands:

     - `make screen IF=SPI`
     - `make csvlogger`
     - `make regdump PLATFORM=RPI`

The executable will be in the found in the same folder as the `Makefile` and source files.

**NOTE:** Modify the EpsonOptions struct to configure sensor settings in main application in `main_xxxx.c`.

**NOTE:** Any references to SPI or GPIO interface refers to using the software on RaspberryPi only. For `PLATFORM=NONE`, there are dummy assignments and GPIO functions do nothing and SPI interface is not supported.
The end user is required to connect the low level code to the GPIO function calls to make use of them on an embedded platform. PCs do not have GPIO or SPI interfaces.

# Important for configuring delays

In the `hcl_linux.c`, there are functions for time delays in millisecond and microseconds using *seDelayMS()* and *seDelayMicroSecs()*, respectively.
On embedded Linux platforms, these may need to be redirected to HW specific delay routines if there is no native support for *usleep()*.

For example on RaspberryPi, the time delay functions for millisecond and microseconds in `hcl_rpi.c` are redirected to WiringPi library *delay()* and *delayMicroseconds()*, respectively.

# Important for GPIO usage

Since PCs typically do not have hardware support for GPIOs, GPIO function is mainly intended for use on embedded Linux platforms.
When this driver connects to the IMU using the UART interface, the use of GPIO pins for connecting to the IMU RESET# or DRDY is optional.
When this driver connects to the IMU using the SPI interface, the use of GPIO pins for connecting to the DRDY is **MANDATORY**.
When possible, connecting the RESET# is recommended to support asserting Hardware Reset during IMU initialization for better robustness and debugging.

This software does not implement the low-level GPIO functions. However, the code is structured to easily redirect to low-level hardware GPIO function calls to simplify implementation.
Review the source files with `_rpi` in the filename for guidance on redirecting to GPIO function calls for low-level implementation.

There is no standard method to implement GPIO connections on embedded Linux platform, but the following files typically need changes:

```
hcl_linux.c
hcl_gpio.c
hcl_gpio.h
```

Typically, an external library needs to be invoked to initialize the library for GPIO HW functions.

This requires the following changes to `hcl_linux.c`:

1. Add `#include` to an external GPIO library near the top of `hcl_linux.c`
2. Add `#include hcl_gpio.h` near the top of the `hcl_linux.c`
3. Add the GPIO library initialization call inside the seInit() function in `hcl_linux.c`

For example on a Raspberry Pi with wiringPi library, the following are changes to `hcl_linux.c`:

```
...
#include <stdint.h>
#include <stdio.h>
#include <wiringPi.h>  // <== Added external library

int seInit(void)
{
  // Initialize wiringPi libraries                                                   // <== Added
  printf("\r\nInitializing libraries...");                                           // <== Added
  if(wiringPiSetupGpio() != 0) {                                                     // <== Added external library initialization
    printf("\r\nError: could not initialize wiringPI libraries. Exiting...\r\n");    // <== Added
    return NG;                                                                       // <== Added
  }                                                                                  // <== Added
  printf("...done.");

  return OK;
}
...
```

Typically, the GPIO pins need to be assigned to pin numbering that is specific to the HW platform.
This requires changes to `hcl_gpio.h`.

For example on a Raspberry Pi with wiringPi, the following are changes to `hcl_gpio.h` for the pin mapping:

| Epson IMU   | Raspberry Pi                   |
| ----------- | ------------------------------ |
| EPSON_RESET | RPI_GPIO_P1_15 (GPIO22) Output |
| EPSON_DRDY  | RPI_GPIO_P1_18 (GPIO24) Input  |

```
// Prototypes for generic GPIO functions
int gpioInit(void);
int gpioRelease(void);

void gpioSet(uint8_t pin);
void gpioClr(uint8_t pin);
uint8_t gpioGetPinLevel(uint8_t pin);

#define RPI_GPIO_P1_15              22                    // <== Added
#define RPI_GPIO_P1_18              24                    // <== Added

#define EPSON_RESET                 RPI_GPIO_P1_15        // <== Added
#define EPSON_DRDY                  RPI_GPIO_P1_18        // <== Added
...
```

Typically, the external library will have GPIO pin control such as *set_output()*, *set_input()*, *set()*, *clear()*, *read_pin_level()*, etc...
A redirect to those GPIO pin control functions are required to `hcl_gpio.c`.

Redirect the function calls in `hcl_gpio.c` for *gpioInit()*, *gpioRelease()*, *gpioSet()*, *gpioClr()*, *gpioGetPinLevel()* to map to an appropriate external library pin control functions.

For example on a Raspberry Pi, the following are changes to `hcl_gpio.c`:

```
#include "hcl.h"
#include "hcl_gpio.h"
#include <wiringPi.h>                         // <== Added external library

...

int gpioInit(void) {
  pinMode(EPSON_RESET, OUTPUT);               // <== Added external call RESET Output Pin
  pinMode(EPSON_DRDY, INPUT);                 // <== Added external call DRDY Input Pin
  pullUpDnControl(EPSON_DRDY, PUD_DOWN) ;     // <== Added external call to enable pull-down resistance

  return OK;
}

...

int gpioRelease(void) {
  return OK;
}

...

void gpioSet(uint8_t pin) {
  digitalWrite(pin, HIGH);                    // <== Added external call set pin HIGH
}


...

void gpioClr(uint8_t pin) {
  digitalWrite(pin, LOW);                     // <== Added external call set pin LOW
}


...

uint8_t gpioGetPinLevel(uint8_t pin) {
  return (digitalRead(pin));                  // <== Added external call to read pin state of input pin
}

...
```

# How to run the program

1. Run the executable from console.
   This may require root access to execute if regular user does not have permission to access the UART or SPI port.

```
sudo ./<executable filename>
```

2. The default `csvlogger` or `screen` program outputs scaled sensor data for 1000 samples with the following settings:

- Output date rate = 125 Hz
- Filter Tap = Moving Average TAP32
- Sensor Output = NDFlags 32-bit Gyro X,Y,Z Accel X,Y,Z, ResetCounter, Checksum

**NOTE:** Output fields can be enabled or disabled by modifying the *EpsonOptions struct* of the `main_xxx.c` file.

# File listing

```
hcl.h                       - Headers of abstraction layer for top-level hardware platform functions (typically does not need modifying)
hcl_linux.c                 - Abstraction layer for top-level on a generic PC Linux
hcl_rpi.c                   - Abstraction layer for top-level on a RaspberryPi using WiringPi library
hcl_gpio.h                  - Header of GPIO abstraction layer. Modify GPIO pin assignments as needed for host platform
hcl_gpio.c                  - GPIO abstraction layer for connections to RESET, DRDY, CS# (modify to redirect to GPIO library calls as needed for the host platform)
hcl_gpio_rpi.c              - GPIO abstraction layer for RaspberryPi using WiringPi library for connection to RESET, DRDY, CS#
hcl_spi.h                   - Header of SPI abstraction layer
hcl_spi_rpi.c               - SPI abstraction layer for RaspberryPi uses WiringPi library
hcl_uart.h                  - Header of UART abstraction layer
hcl_uart.c                  - UART abstraction layer using termios library on Linux
main_csvlogger.c            - Test application - Initialize IMU, and read sensor data to CSV log file
main_regdump.c              - Test application - Output register settings to console for debug purpose
main_screen.c               - Test application - Initialize IMU, and read sensor data to console
main_helper.c               - Helper functions for test application
main_helper.h               - Header of helper functions
Makefile                    - For make utility to compile test applications
README_src.md               - This file
sensor_epsonCommon.c        - Common function calls for Epson IMU
sensor_epsonCommon.h        - Header of common functions for Epson IMU.
sensor_epsonSpi.c           - Register read/write and burst read functions for SPI
sensor_epsonSpi.h           - Header for Epson SPI functions
sensor_epsonUart.c          - Register read/write and burst read functions for UART
sensor_epsonUart.h          - Header for Epson UART functions.
```

# Change record:

```
2023-11-08  v1.0.0    - Merge UART Driver v1.9 and SPI Driver v1.7, and minor updates
2024-07-17  v2.0.0    - Add runtime auto-detect/select of IMU model, re-organize code structure for better consistency
2024-09-05  v2.0.1    - Minor bugfix for swapped bits in SIG_CTRL_LO, BURST_CTRL1_HI, BURST_CTRL2_HI for gyro_delta_out & accel_delta_out
```
