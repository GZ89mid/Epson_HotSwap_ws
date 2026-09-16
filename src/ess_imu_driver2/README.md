# README for Epson IMU Driver for ROS2 Node

> ## ⚠️ 本仓库定制说明 (2026-08 重构)
>
> 本仓库在官方驱动基础上做了大幅改造, 以下官方章节仅作参考:
>
> ### 当前架构 (单进程热插拔驱动)
> - **只保留 UART 接口驱动** `src/imu_hub_node.cpp` — 单进程管理**任意数量** IMU
> - **实时热插拔**: 插入自动发布话题, 拔出自动删除话题 (实时话题删减)
> - **绑定 ID**: `config/imu_bindings.yaml` 按 CH343 USB 序列号绑定名称
> - **只输出 raw 数据**: 话题 `/<名称>/data_raw` (sensor_msgs/Imu) + `/<名称>/tempc` (sensor_msgs/Temperature)
> - **所有滤波/姿态算法已删除** (Mahony, EKF 等), 不提供姿态
>
> ### 关于 orientation 字段 (raw 无姿态)
> `sensor_msgs/msg/Imu` 消息类型**强制包含** `orientation` 四元数字段, 无法从消息中删除。
> raw 驱动按 **REP-145 约定**表达"无姿态":
> - `orientation` 填占位的**单位四元数** `(w=1, x=0, y=0, z=0)`
> - **`orientation_covariance[0] = -1`** — 消费者检测到此值即应**忽略 orientation** (如 rviz、robot_localization 等均按此标准处理)
>
> ### 快速启动
> ```bash
> ros2 launch ess_imu_driver2 imu_hub_launch.py
> ```
> 详见 `docs/启动流程.md` 与 `docs/四IMU多设备启动指南.md`。

<!---toc start-->

- [README for Epson IMU Driver for ROS2 Node](#readme-for-epson-imu-driver-for-ros2-node)
  - [What is this repository for?](#what-is-this-repository-for)
  - [What kind of hardware or software will I likely need?](#what-kind-of-hardware-or-software-will-i-likely-need)
    - [For the UART Interface:](#for-the-uart-interface)
    - [For the SPI Interface:](#for-the-spi-interface)
  - [How do I use the driver?](#how-do-i-use-the-driver)
  - [How do I use the driver if usleep() is not supported for time delays?](#how-do-i-use-the-driver-if-usleep-is-not-supported-for-time-delays)
  - [How do I use the driver with GPIOs to control IMU RESET#, DRDY, EXT pins?](#how-do-i-use-the-driver-with-gpios-to-control-imu-reset-drdy-ext-pins)
  - [How do I build, install, run this package?](#how-do-i-build-install-run-this-package)
    - [Example console output of *colcon build*:](#example-console-output-of-colcon-build)
    - [Example console output of launching ROS node:](#example-console-output-of-launching-ros-node)
  - [What does this ROS IMU Node publish as messages?](#what-does-this-ros-imu-node-publish-as-messages)
    - [ROS Topic Message /imu/data](#ros-topic-message-imudata)
    - [ROS Topic Message /imu/data_raw](#ros-topic-message-imudata_raw)
  - [Why am I seeing inaccurate ROS timestamps, high latencies, or slower than expected IMU data rates?](#why-am-i-seeing-inaccurate-ros-timestamps-high-latencies-or-slower-than-expected-imu-data-rates)
    - [ROS timestamps](#ros-timestamps)
    - [When using USB-UART bridges](#when-using-usb-uart-bridges)
      - [Modifying *latency_timer* by udev mechanism](#modifying-latency_timer-by-udev-mechanism)
      - [Modifying *latency_timer* by sysfs mechanism](#modifying-latency_timer-by-sysfs-mechanism)
      - [Modifying *low_latency* flag using *setserial* utility](#modifying-low_latency-flag-using-setserial-utility)
    - [When using the SPI interface](#when-using-the-spi-interface)
  - [Package Contents](#package-contents)
  - [References](#references)

<!---toc end-->

## What is this repository for?

- This code is a ROS2 package for demonstrating a ROS2 node that configures and publishes IMU messages from a supported Epson IMU.
- This code provides software communication between Epson IMU and ROS using the either the UART or SPI interface.
- For the UART connection, this code uses the standard Unix Terminal I/O library (termios) for communicating either direct or by USB-serial converter such as FTDI USB-UART bridge ICs.
- For the SPI connection, this code uses the [Unofficial wiringPi](https://github.com/WiringPi/WiringPi/) library for accessing GPIO and SPI functions on the Raspberry Pi platform running Ubuntu Linux distro.
- This ROS2 node demonstration software is a ROS C++ wrapper around the Linux C driver software:
  - *src/epson_imu_uart_ros2_node.cpp* is for the UART interface
  - *src/epson_imu_spi_ros2_node.cpp* is for the SPI interface
- The other source files in `src/` are based on the Linux C driver originally released and can be found here:
  [Linux C driver and logger example for EPSON IMU](https://github.com/cubicleguy/imu_linux_example)
- Information about ROS2, and tutorials can be found: [ROS.org](https://docs.ros.org/)

## What kind of hardware or software will I likely need?

- Epson IMU [Epson IMU models](https://global.epson.com/products_and_drivers/sensing_system/imu/)
  - At the time software release:
    - G320PDG0, G320PDGN, G354PDH0, G364PDCA, G364PDC0
    - G365PDC1, G365PDF1, G370PDF1, G370PDS0
    - G330PDG0, G366PDG0, G370PDG0, G370PDT0
    - G570PR20
- ROS2 Foxy or Humble or Jazzy
- This software was developed and tested on the following:

```
  ROS2:        Foxy, Humble, Jazzy
  Description: Ubuntu 20.04 LTS, Ubuntu 22.04 LTS, Ubuntu 24.04 LTS
  Hardware Platform: Core i7 PC, Raspberry Pi 3B+, RaspberryPi 4
```

### For the UART Interface:

- Epson USB evaluation board or equivalent FTDI USB-Serial interface connecting the Epson IMU to ROS host (tty/serial) [See M-G32EV041](https://global.epson.com/products_and_drivers/sensing_system/technical_info/evaluation_tools/)
- Alternatively, a direct connection from the Epson IMU to the ROS platform supporting a 3.3V CMOS compatible UART interface [See M-G32EV031](https://global.epson.com/products_and_drivers/sensing_system/technical_info/evaluation_tools/).

### For the SPI Interface:

- **NOTE:** This software is intended for an embedded Linux host system with 3.3V I/O compatible SPI interface (SCLK, MISO, MOSI) and 3 GPIOs (CS#, RST#, DRDY).
- For Raspberry Pi, if not enabled already the SPI interface can be enabled using `raspi-config` or equivalent.
- This code uses a separate GPIO to manually control CS# chipselect instead of the chipselect assigned to by the RapsberryPi SPI interface.
  - The chipselect assigned by the HW SPI interface should also work, but has not been thoroughly tested.
- Epson Breakout evaluation board or some equivalent is required to connect to the 3.3V CMOS compatible pins of the ROS host (SPI & GPIOs) [See M-G32EV031](https://global.epson.com/products_and_drivers/sensing_system/technical_info/evaluation_tools/)

## How do I use the driver?

- This code assumes that the user is familiar with building ROS2 packages using the `colcon build` process

- This README is *NOT* detailed step by step instructions on how to build and install ROS2 software.

- Please refer to the ROS.org website for more detailed instructions on the ROS package build process. [ROS.org](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

- **NOTE:** At a bare minimum, the user must re-build the package with colcon after modifying the `CMakeLists.txt` to configure any of the following:

  - serial interface type, `INTERFACE=` (UART or SPI)
  - host platform type, `PLATFORM=` (RPI or NONE=PC)

- Changes to IMU settings should be made by editing the `.py` launch files located in the `launch/` folder instead of modifying the `src\` source files

## How do I use the driver if usleep() is not supported for time delays?

- **NOTE:** In the `hcl_linux.c` or `hcl_rpi.c`, there are `seDelayMS()` and `seDelayMicroSecs()` wrapper functions for time delays in millisecond and microseconds, respectively.
- On embedded Linux platforms, the user may need to modify and redirect to platform specific delay routines if `usleep()` is not supported.
- For example on RaspberryPi, the time delay functions for millisecond and microseconds are redirected to WiringPi library `delay()` and `delayMicroseconds()`, respectively.
- If a hardware delay is not available from a library, then a software delay loop is possible but not preferred.

## How do I use the driver with GPIOs to control IMU RESET#, DRDY, EXT pins?

- When connecting the IMU using the UART interface, the use of GPIO pins for connecting to the IMU `RESET#` or `DRDY` is optional, but can be useful on an embedded Linux platforms (such as RapsberryPi).
- When connecting the IMU using the SPI interface, the use of GPIO pins for connecting to the IMU `DRDY` is mandatory (`RESET#` is recommended, `EXT` is optional).
- When using the `time_correction` function for accurate time stamping with an external 3.3V GNSS 1PPS signal, the IMU `EXT` pin must be connected to the 1PPS.
- The user can choose to control the IMU `CS#` pin directly with a designated host GPIO output or connect to the designated chipselect of the SPI interface (i.e. `SPI_CE0` pin on the RapsberryPi)
- Where possible, connecting the `RESET#` and forcing a Hardware Reset during every IMU initialization is recommended for better robustness.
- This code is structured to simplify editing to redirect GPIO control to low-level hardware library GPIO function calls.
- There are no standard methods to implement GPIO connections on embedded Linux platform, but the following source files typically need changing:

```
src/hcl_linux.c
src/hcl_gpio.c
src/hcl_gpio.h
```

- Typically, an external library needs to be invoked to initialize & enable GPIO HW functions on the user's embedded platform.

- This typically requires the following changes to `hcl_linux.c` (Refer to `hcl_rpi.c` as a template):

  - add `#include` to external library near the top of `hcl_linux.c`
  - add the initialization call to the HW library inside the `seInit()` function in `hcl_linux.c`

For example on a Raspberry Pi, the following changes can be made to `hcl_linux.c`:

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

- Typically, the GPIO pins need to be assigned according to embedded HW platform-specific **pin numbering** and requires changes to `hcl_gpio.h`.

**NOTE:** When using the SPI interface, if direct `CS#` control is not by GPIO, then connect IMU `CS#` pin to the RPI `SPI0_CE0 (P1_24)`.

For example on a Raspberry Pi, the changes to `hcl_gpio.h` assume the following pin mapping:

```
Epson IMU                   Raspberry Pi
---------------------------------------------------
EPSON_RESET                 RPI_GPIO_P1_15 (GPIO22) Output
EPSON_DRDY                  RPI_GPIO_P1_18 (GPIO24) Input
```

```
...

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

- The external HW library will typically have GPIO pin control functions such as `set_output()`, `set_input()`, `set()`, `reset()`, `read_pin_level()`, etc...

- The user should redirect the function calls in `hcl_gpio.c` for `gpioInit()`, `gpioRelease()`, `gpioSet()`, `gpioClr()`, `gpioGetPinLevel()` to such equivalent external HW library pin control functions.

- For example on a Raspberry Pi, the following are changes to `hcl_gpio.c` which can be used as a template:

```
#include "hcl.h"
#include "hcl_gpio.h"
#include <wiringPi.h>                         // <== Added external library

...

int gpioInit(void)
{
	pinMode(EPSON_RESET, OUTPUT);               // <== Added external call RESET Output Pin
	pinMode(EPSON_DRDY, INPUT);                 // <== Added external call DRDY Input Pin
	pullUpDnControl(EPSON_DRDY, PUD_OFF) ;      // <== Added external call Disable any internal pullup or pulldown resistance

	return OK;
}

...

int gpioRelease(void)
{
	return OK;
}

...

void gpioSet(uint8_t pin)
{
	digitalWrite(pin, HIGH);                    // <== Added external call set pin HIGH
}

...

void gpioClr(uint8_t pin)
{
	digitalWrite(pin, LOW);                     // <== Added external call set pin LOW
}

...

uint8_t gpioGetPinLevel(uint8_t pin)
{
	return (digitalRead(pin));                  // <== Added external call to return pin state of input pin
}

...
```

## How do I build, install, run this package?

The Epson IMU ROS2 driver is designed for building with the standard ROS2 colcon build environment.
For more information on setting up the ROS2 environment refer to: [Installing and Configuring ROS Environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html).
For more information on setting up the ROS2 colcon environment refer to: [ROS2 Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

1. Place this package (including folders) into a new folder within your colcon workspace `src\` folder.
   For example, we recommend using the folder name `ess_imu_driver2`

```
<colcon_workspace>/src/ess_imu_driver2/
```

2. Modify the `CMakeLists.txt` to select the serial interface type `INTERFACE=` and platform type `PLATFORM=` that matches your ROS2 system and connection to the Epson IMU.
   Refer to the comments in the `CMakeLists.txt` for additional info.
   **NOTE:** You *MUST* re-build using `colcon build` after any changes in the `CMakeLists.txt`.

3. From the colcon workspace folder run `colcon build --symlink-install` to build all ROS2 packages located in the `<colcon_workspace>/src/` folder.
   **NOTE:** Re-run `colcon build --symlink-install` to rebuild the software after making any changes to any of the `.c` or `.cpp` or `.h` source files.

**NOTE:** It is recommended to change IMU settings by editing the parameters in the `.py` launch file, instead of modifying the `.c`, `.cpp`, `.h` source files directly.

```
<colcon_workspace>/colcon build --packages-select ess_imu_driver2 --symlink-install
```

4. Reload the current ROS2 environment variables that may have changed after the colcon build process by entering the following from the `\<colcon_workspace>`:

```
source ./install/setup.bash
```

5. Modify the `.py` launch file in the `launch/` folder to set your desired IMU configure parameter options at runtime:

| Parameter            | Comment                                                                                                                                                     |
| -------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| serial_port          | specifies UART port on the system when built and configured for UART in the CMakeLists.txt                                                                  |
| frame_id             | specifies the value in the IMU message frame_id field                                                                                                       |
| burst_polling_rate   | specifies the driver internal polling rate for the serial port input buffer (typically does not need changing)                                              |
| imu_dout_rate        | specifies the IMU output data rate                                                                                                                          |
| imu_filter_sel       | specifies the IMU filter setting                                                                                                                            |
| quaternion_output_en | enables or disables quaternion output (not supported on all IMU models)                                                                                     |
| atti_profile         | specifies the attitude motion profile (not supported on all IMU models)                                                                                     |
| output_32bit_en      | enables or disables outputting sensor data in 32 or 16 bit resolution                                                                                       |
| time_correction_en   | enables time correction function using IMU counter reset function & external 1PPS connection to IMU GPIO2/EXT pin. **Cannot be used when ext_trigger_en=1** |

**NOTE:** The `.py` launch file passes IMU configuration settings to ROS node at runtime. Therefore, re-building with `colcon build --symlink-install` when changing the `.py` launch file is not required.
**NOTE:** When using this software on older ROS2 versions (Dashing or Eloquent) the `.py` launch file needs to be modified to replace `namespace` parameter from `executable` to `node_executable`

6. Start the Epson IMU ROS2 driver use the appropriate `.py` launch file located in `launch/`. All parameters are described in the in-line comments of the `.py` launch file.

For example, to launch the ROS2 node:

```
<colcon_workspace>/ros2 launch ess_imu_driver2 launch.py
```

| Launch File   | Description                                                                                                                                                                         |
| ------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| launch.py     | Publishes to `imu/data` for IMU models that support `gyro`, `accel`, and `quaternion (orientation)` or `imu/data_raw` for IMU models that do not support `quaternion (orientation)` |
| raw_launch.py | Publishes to `imu/data_raw` and does not enable or update `quaternion (orientation)`                                                                                                |
| tc_launch.py  | Same as launch.py except time correction function is enabled (GNSS 1PPS signal must be connected to IMU EXT pin)                                                                    |

### Example console output of *colcon build*:

```
user@RPI4-RC:~/ros2_ws$ colcon build --symlink-install --packages-select ess_imu_driver2
Starting >>> ess_imu_driver2
Finished <<< ess_imu_driver2 [29.3s]

Summary: 1 package finished [30.1s]
user@RPI4-RC:~/ros2_ws$
```

### Example console output of launching ROS node:

```
user@RPI4-RC:~/ros2_ws$ ros2 launch ess_imu_driver2 launch.py
[INFO] [launch]: All log files can be found below /home/user/.ros/log/2024-07-31-12-52-53-476565-VDC-RPI4-RC-853040
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [ess_imu_driver2_node-1]: process started with pid [853041]
[ess_imu_driver2_node-1] [INFO] [1722455573.931168183] [epson_node]: frame_id:                  imu_link
[ess_imu_driver2_node-1] [INFO] [1722455573.931458588] [epson_node]: time_correction_en:        0
[ess_imu_driver2_node-1] [INFO] [1722455573.931508699] [epson_node]: burst_polling_rate:        4000.0
[ess_imu_driver2_node-1] [WARN] [1722455573.931558883] [epson_node]: Not specified param temperature_topic. Set default value:       /epson_imu/tempc
[ess_imu_driver2_node-1] [WARN] [1722455573.931592180] [epson_node]: Not specified param ext_trigger_en. Set default value:  0
[ess_imu_driver2_node-1] [INFO] [1722455573.931624976] [epson_node]: imu_dout_rate:             4
[ess_imu_driver2_node-1] [INFO] [1722455573.931657012] [epson_node]: imu_filter_sel:            5
[ess_imu_driver2_node-1] [INFO] [1722455573.931688253] [epson_node]: quaternion_output_en:      1
[ess_imu_driver2_node-1] [INFO] [1722455573.931718123] [epson_node]: output_32bit_en:           1
[ess_imu_driver2_node-1] [INFO] [1722455573.931761197] [epson_node]: atti_profile:              0
[ess_imu_driver2_node-1] [INFO] [1722455576.933310504] [epson_node]: Checking sensor power on status...
[ess_imu_driver2_node-1] [INFO] [1722455577.135333655] [epson_node]: Detecting sensor model...
[ess_imu_driver2_node-1] ...Initializing wiringPI library......done....delay for GPIO pins...
[ess_imu_driver2_node-1] Reading device model and serial number...
[ess_imu_driver2_node-1] PRODUCT ID:    G366PDG0
[ess_imu_driver2_node-1] [INFO] [1722455577.139222921] [epson_node]: Initializing Sensor...
[ess_imu_driver2_node-1] [INFO] [1722455577.146912528] [epson_node]: Epson IMU initialized.
[ess_imu_driver2_node-1] SERIAL ID:     T1000062
[ess_imu_driver2_node-1] [INFO] [1722455577.147432672] [epson_node]: Sensor started...


```

## What does this ROS IMU Node publish as messages?

The Epson IMU ROS node will publish messages as convention per [REP 145](http://www.ros.org/reps/rep-0145.html).

- For IMU models that do not support quaternion output, the `.py` launch file will remap IMU messages to `/imu/data_raw` and only update the fields for `angular_velocity` (gyro), and `linear_acceleration` (accel).
- For IMU models that supports and enables quaternion, the `.py` launch file will remap IMU messages to `/imu/data` and update `angular_velocity` (gyro), `linear_acceleration` (accel), and `orientation` field using the internal extended Kalman Filter.
- Temperature sensor data from the IMU will publish on topic `/epson_imu/tempc`

### ROS Topic Message /imu/data

```
---
header:
  stamp:
    sec: 1685644337
    nanosec: 660167310
  frame_id: imu_link
orientation:
  x: 0.9999865293502808
  y: -0.00023142248392105103
  z: 0.002697369083762169
  w: -0.004425106570124626
orientation_covariance:
- -1.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
angular_velocity:
  x: 0.00032043419196270406
  y: -0.0005722396308556199
  z: 0.0002733084256760776
angular_velocity_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
linear_acceleration:
  x: 0.04799626022577286
  y: -0.07659434527158737
  z: -9.828039169311523
linear_acceleration_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
---
```

### ROS Topic Message /imu/data_raw

```
---
header:
  stamp:
    sec: 1685643703
    nanosec: 402378465
  frame_id: imu_link
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0
orientation_covariance:
- -1.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
angular_velocity:
  x: -0.0002180843148380518
  y: 7.883128273533657e-05
  z: 6.580488116014749e-05
angular_velocity_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
linear_acceleration:
  x: 0.07670824229717255
  y: -0.056904129683971405
  z: -9.818550109863281
linear_acceleration_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
---
```

## Why am I seeing inaccurate ROS timestamps, high latencies, or slower than expected IMU data rates?

### ROS timestamps

- By default, the ROS node publishes using the ROS/Unix current timestamp at the time of processing.
- Latencies from system overhead, system loading, and buffering will cause timestamp inaccuracies between the sensor data and this ROS/Unix timestamp.
- To improve timestamp accuracy, the user should consider using the time correction function with a GNSS receiver with 1PPS output to increase clock accuracy on the ROS host system:
  - Consider setting up [chrony](https://chrony-project.org/examples.html) and [gpsd](https://gpsd.io/) with the GNSS receiver's 1PPS signal connected to constrain the host system clock drift
  - Consider connecting the GNSS receiver's 1PPS signal to the Epson IMU GPIO2/EXT with `time_correction` enabled in the `.py` launch file to improve time-stamp accuracy of the IMU sensor messages

### When using USB-UART bridges

- If your connection between the Epson IMU UART interface and the Linux host is by FTDI ICs, the `latency_timer` setting in the FTDI driver may be large i.e. typically 16 (msec).
- This may affect the UART latency and maximum IMU data rates on your host system.
- There are 3 methods listed below to reduce the impact of this latency.

#### Modifying *latency_timer* by udev mechanism

- [udev](https://wiki.debian.org/udev) is a device manager for Linux that can dynamically create and remove devices in *userspace* and run commands when new devices appear or other events
- Create a udev rule to automatically set the `latency_timer` to 1 when an FTDI USB-UART device is plugged in to a USB port.
- For example, the following text file named `99-ftdi_sio.rules` can be put in the `/etc/udev/rules.d` directory

**NOTE:** This requires root (sudo) access to create or copy file in `/etc/udev/rules.d`
**NOTE:** This is the recommended method because it is automatic when device is plugged in, but it affects ALL FTDI USB-UART devices on the system

```
SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
```

#### Modifying *latency_timer* by sysfs mechanism

- The example below reads the `latency_timer` setting for `/dev/ttyUSB0` which returns 16msec.
- Then, it sets the `latency_timer` to 1msec, and confirms it by read back.

**NOTE: This may require root (sudo su) access on your system to execute.**

```
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
16
echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
1
```

#### Modifying *low_latency* flag using *setserial* utility

- The example below sets the `low_latency` flag for `/dev/ttyUSB0`.
- This will have the same effect as setting the `latency_timer` to 1msec.
- This can be confirmed by running the `setserial` command again.

```
user@user:~$ setserial /dev/ttyUSB0
/dev/ttyUSB0, UART: unknown, Port: 0x0000, IRQ: 0

user@user:~$ setserial /dev/ttyUSB0 low_latency

user@user:~$ setserial /dev/ttyUSB0
/dev/ttyUSB0, UART: unknown, Port: 0x0000, IRQ: 0, Flags: low_latency
```

### When using the SPI interface

- latency issues with SPI interface will largely depend on your host system's processing load and capabilities.
- If your ROS platform is running too many ROS node packages or too slow, it may not react fast enough to detect the rising edge of the IMU DRDY signal and
  process the IMU sampling data.
- Try modifying the `dout_rate` and `filter_sel` setting in the `.py` launch file to the slowest setting that can meet your system requirements.
- Try monitoring the DRDY signal on the IMU with a oscilloscope to verify the stability of the IMU DRDY signal and seeing that it matches the expected `dout_rate` frequency.
- Try to set the SPI clock rate to maximum of 1MHz (1000000) in the `_node.cpp` for the `spiInit()` function call:

```
...

    while (rclcpp::ok() && !spiInit(SPI_MODE3, 1000000)) {
      RCLCPP_WARN(this->get_logger(),
                  "Retry to initialize the SPI layer in 1 second period...");
      one_sec.sleep();
    }

...
```

## Package Contents

The Epson IMU ROS2 driver-related sub-folders & root files are:

```
launch/        <== various example launch files
src/           <== source code for ROS2 node C++ wrapper, IMU Linux C driver, and additional README_src.md (specifically for building and using the IMU Linux C driver as stand-alone without ROS support)
CMakeLists.txt <== cmake build script used by colcon
LICENSE.txt    <== description of the applicable licenses
package.xml    <== colcon package description
README.md      <== general README
```

## References

1. https://index.ros.org/doc/ros2/
