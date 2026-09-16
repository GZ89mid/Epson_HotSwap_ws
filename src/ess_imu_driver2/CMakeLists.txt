cmake_minimum_required(VERSION 3.5)
project(ess_imu_driver2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# NOTE: Refer to the README_src.md inside the src folder for more details about the C source files

################################################################
# 此设置是构建前通常唯一需要修改的项

# 1. 选择 NONE 标准 PC, 或 RPI RaspberryPi
set(PLATFORM "NONE")
#set(PLATFORM "RPI")

###############################################################

message(STATUS "---- Building for platform: ${PLATFORM}")

# Define macros variables for compilation
add_definitions(-D${PLATFORM})
add_definitions(-DUART)

# Create file list for C libraries based on PLATFORM
if (PLATFORM STREQUAL "RPI")
  set(lib_sources
    src/hcl_rpi.c
    src/hcl_gpio_rpi.c
  )
elseif (PLATFORM STREQUAL "NONE")
  set(lib_sources
    src/hcl_linux.c
    src/hcl_gpio.c
  )
else()
  message(FATAL_ERROR "**** Invalid Platform")
endif()

# UART 接口的 C 库
# 注意: imu_hub_node 自包含实例化串口/协议实现, 仅复用
#       sensor_epsonCommon.c 中的 epson_sensors[] 型号属性表
set(lib_sources ${lib_sources}
  src/hcl_uart.c
  src/sensor_epsonCommon.c
  src/sensor_epsonUart.c
)

# Declare library for Epson IMU functions
add_library(ess_imu_driver2_lib
  ${lib_sources}
)

# Link any external libraries to Epson IMU Library
if (PLATFORM STREQUAL "RPI")
  # Determine location of wiringPi library on the host system
  # Required for building on Raspberry Pi platform
  find_library(wiringPi_LIB NAMES wiringPi)

  target_link_libraries(ess_imu_driver2_lib
    ${wiringPi_LIB}
  )
endif()


# 主驱动节点: 单进程多 IMU 热插拔 (imu_hub)
#   - 实时热插拔 / 实时话题删减
#   - 支持绑定 ID (config/imu_bindings.yaml)
#   - 只输出 raw 数据 (/<name>/data_raw, /<name>/tempc)
add_executable(ess_imu_driver2_node
  src/imu_hub_node.cpp
)

# Link C libraries to C++ executable ROS node
target_link_libraries(ess_imu_driver2_node
  ess_imu_driver2_lib
  rt
  pthread
)

# Link C++ Library to C++ executable ROS node
ament_target_dependencies(ess_imu_driver2_node
  rclcpp
  sensor_msgs
)

# Mark executables and/or libraries for installation
install(
   TARGETS ess_imu_driver2_node
   DESTINATION lib/${PROJECT_NAME}
 )

# Mark directories for installation
install(
   DIRECTORY launch DESTINATION share/${PROJECT_NAME}
)

# Install config directory (imu_bindings.yaml etc.)
install(
   DIRECTORY config DESTINATION share/${PROJECT_NAME}
)

ament_package()
