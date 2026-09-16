//==============================================================================
//
//  imu_hub_node.cpp - 单进程多 IMU 热插拔驱动 (Epson G354 / G3xx UART)
//
//  功能:
//    - 只启动一个 ROS2 节点即可实时热插拔, 插入/拔出 IMU 无需重启
//    - 每插入一个 IMU 自动发布 /<name>/data_raw (和 /<name>/tempc)
//    - 拔出设备后对应话题自动从 ROS 图中消失 (实时话题删减)
//    - 支持绑定 ID: 通过 YAML 将 CH343/USB 串口序列号绑定到固定名称
//    - 支持无限数量 IMU (每设备独立串口 fd + 独立读取线程)
//    - 只输出 IMU raw 数据 (角速度 + 加速度 + 温度), 不含任何滤波/姿态
//
//  协议说明:
//    Epson G3xx IMU UART 协议:
//      寄存器写:  [addr|0x80, data, 0x0D]
//      寄存器读:  [addr&0x7E, 0x00, 0x0D] -> 响应 [addr, hi, lo, 0x0D]
//      Burst 数据: 0x80 头 + 数据 + 0x0D 尾 (可含校验和)
//    本节点自带实例化串口/协议实现, 不依赖全局单实例的 C 库
//    (hcl_uart.c / sensor_epsonUart.c 使用全局 fd, 无法支持多设备并发)
//
//  [This software is BSD-3
//  licensed.](http://opensource.org/licenses/BSD-3-Clause)
//
//==============================================================================

#include <glob.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <limits.h>
#include <libgen.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include "sensor_epsonCommon.h"  // 寄存器常量 + epson_sensors[] 型号属性表

using namespace std::chrono_literals;

// UART 帧标记 (与 sensor_epsonUart.c 一致)
static const unsigned char UART_HEADER = 0x80;   // Burst 数据帧头
static const unsigned char UART_DELIMITER = 0x0D;  // 帧尾
static const unsigned short EPSON_ID_VAL = 0x5345;  // ADDR_ID 期望值

//=========================================================================
// EpsonUart - 每个 IMU 独立的串口实例 (termios)
//=========================================================================
class EpsonUart {
 public:
  EpsonUart() = default;
  ~EpsonUart() { close(); }
  EpsonUart(const EpsonUart&) = delete;
  EpsonUart& operator=(const EpsonUart&) = delete;

  bool open(const std::string& path, int baud) {
    close();
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) return false;

    speed_t speed;
    switch (baud) {
      case 2000000: speed = B2000000; break;
      case 1500000: speed = B1500000; break;
      case 1000000: speed = B1000000; break;
      case 921600: speed = B921600; break;
      case 460800: speed = B460800; break;
      case 230400: speed = B230400; break;
      default:
        close();
        return false;
    }

    struct termios options;
    if (tcgetattr(fd_, &options) != 0) {
      close();
      return false;
    }
    cfsetospeed(&options, speed);
    cfsetispeed(&options, speed);

    // raw 模式 (与 hcl_uart.c 一致)
    options.c_iflag &=
      ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    options.c_oflag = 0;
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    options.c_cflag &= ~(CSIZE | PARENB);
    options.c_cflag |= CS8;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 20;  // 2 秒超时
    options.c_cflag |= (CLOCAL | CREAD);

    if (tcsetattr(fd_, TCSAFLUSH, &options) != 0) {
      close();
      return false;
    }
    purge();
    return true;
  }

  void close() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  int read(unsigned char* buf, int size) {
    if (fd_ < 0) return -1;
    return ::read(fd_, buf, size);
  }

  int write(const unsigned char* buf, int size) {
    if (fd_ < 0) return -1;
    return ::write(fd_, buf, size);
  }

  int bytesAvailable() {
    int n = 0;
    if (fd_ >= 0 && ioctl(fd_, FIONREAD, &n) == 0) return n;
    return 0;
  }

  void purge() {
    if (fd_ >= 0) tcflush(fd_, TCIOFLUSH);
  }

  // 等待数据可读, 超时 timeout_ms 毫秒; 返回 true 表示有数据可读
  bool waitReadable(int timeout_ms) {
    if (fd_ < 0) return false;
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    int r = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
    return r > 0;
  }

  bool valid() const { return fd_ >= 0; }
  int fd() const { return fd_; }

 private:
  int fd_ = -1;
};

//=========================================================================
// EpsonProtocol - 每个 IMU 独立的协议状态机
//   (逻辑等价于 sensor_epsonUart.c + sensor_epsonCommon.c 的实例化版本)
//=========================================================================
class EpsonProtocol {
 public:
  explicit EpsonProtocol(EpsonUart* uart) : uart_(uart) {}

  void resetState() {
    state_ = 0;
    data_count_ = 0;
  }

  // 串口时序: TSTALL = 134us @ 460800 波特
  static void stall() { usleep(134); }

  // 寄存器写: [addr|0x80, data, 0x0D]
  void writeByte(unsigned char regAddr, unsigned char regByte) {
    unsigned char tx[3] = {static_cast<unsigned char>(regAddr | 0x80),
                           regByte, UART_DELIMITER};
    uart_->write(tx, 3);
    stall();
  }

  // 寄存器写 (带窗口): 先写 WIN_CTRL
  void registerWriteByte(unsigned char winNumber, unsigned char regAddr,
                         unsigned char regByte) {
    writeByte(ADDR_WIN_CTRL, winNumber);
    writeByte(regAddr, regByte);
  }

  // 寄存器读: [addr&0x7E, 0x00, 0x0D] -> [addr, hi, lo, 0x0D]
  unsigned short read16(unsigned char regAddr) {
    unsigned char tx[3] = {static_cast<unsigned char>(regAddr & 0x7E),
                           0x00, UART_DELIMITER};
    unsigned char resp[4] = {0};
    uart_->write(tx, 3);
    stall();
    int size = readExact(resp, 4);
    if (size < 4 || resp[0] != tx[0] || resp[3] != UART_DELIMITER) {
      return 0;
    }
    return (unsigned short)resp[1] << 8 | (unsigned short)resp[2];
  }

  // 寄存器读 (带窗口)
  unsigned short registerRead16(unsigned char winNumber,
                                unsigned char regAddr) {
    writeByte(ADDR_WIN_CTRL, winNumber);
    return read16(regAddr);
  }

  // 读满 size 字节 (带超时)
  int readExact(unsigned char* buf, int size, int timeout_ms = 500) {
    int got = 0;
    auto start = std::chrono::steady_clock::now();
    while (got < size) {
      if (std::chrono::steady_clock::now() - start >
          std::chrono::milliseconds(timeout_ms)) {
        break;
      }
      int n = uart_->read(buf + got, size - got);
      if (n > 0) {
        got += n;
      } else if (n < 0) {
        break;
      } else {
        usleep(1000);
      }
    }
    return got;
  }

  // 上电检查: 进入 Config 模式, 校验 ID 与 DIAG_STAT
  bool powerOn() {
    unsigned short rxData = 0xFFFF;
    unsigned short retryCount = 10;
    do {
      uart_->purge();  // 丢弃残留数据帧, 防止干扰寄存器响应
      registerWriteByte(WIN_ID0, ADDR_MODE_CTRL_HI, CMD_CONFIG);
      rxData = registerRead16(WIN_ID0, ADDR_MODE_CTRL_LO);
      usleep(200000);
      retryCount--;
    } while ((rxData & BIT10) == 0 && retryCount > 0);

    if (retryCount == 0) return false;

    rxData = registerRead16(WIN_ID0, ADDR_ID);
    if (rxData != EPSON_ID_VAL) return false;

    rxData = registerRead16(WIN_ID0, ADDR_DIAG_STAT);
    if (rxData != 0x0000) return false;
    return true;
  }

  // 开始采样
  void start() {
    registerWriteByte(WIN_ID0, ADDR_MODE_CTRL_HI, CMD_SAMPLING);
  }

  // 停止采样
  void stop() {
    writeByte(ADDR_MODE_CTRL_HI, CMD_CONFIG);
    usleep(200000);
    uart_->purge();
  }

  // 检测型号: 读 PROD_ID / SERIAL_NUM (W1), 匹配 epson_sensors[] 属性表
  bool detectModel(struct EpsonProperties& props, std::string& prod_id,
                   std::string& serial_id) {
    char pid[9] = {0};
    char sid[9] = {0};
    unsigned short p1 = registerRead16(WIN_ID1, ADDR_PROD_ID1);
    unsigned short p2 = registerRead16(WIN_ID1, ADDR_PROD_ID2);
    unsigned short p3 = registerRead16(WIN_ID1, ADDR_PROD_ID3);
    unsigned short p4 = registerRead16(WIN_ID1, ADDR_PROD_ID4);
    pid[0] = (char)p1;
    pid[1] = (char)(p1 >> 8);
    pid[2] = (char)p2;
    pid[3] = (char)(p2 >> 8);
    pid[4] = (char)p3;
    pid[5] = (char)(p3 >> 8);
    pid[6] = (char)p4;
    pid[7] = (char)(p4 >> 8);

    unsigned short s1 = registerRead16(WIN_ID1, ADDR_SERIAL_NUM1);
    unsigned short s2 = registerRead16(WIN_ID1, ADDR_SERIAL_NUM2);
    unsigned short s3 = registerRead16(WIN_ID1, ADDR_SERIAL_NUM3);
    unsigned short s4 = registerRead16(WIN_ID1, ADDR_SERIAL_NUM4);
    sid[0] = (char)s1;
    sid[1] = (char)(s1 >> 8);
    sid[2] = (char)s2;
    sid[3] = (char)(s2 >> 8);
    sid[4] = (char)s3;
    sid[5] = (char)(s3 >> 8);
    sid[6] = (char)s4;
    sid[7] = (char)(s4 >> 8);

    prod_id = pid;
    serial_id = sid;

    for (int i = G_EMPTY; i < G_UNKNOWN; i++) {
      if (std::strcmp(epson_sensors[i].product_id, pid) == 0) {
        props = epson_sensors[i];
        return true;
      }
    }
    return false;
  }

  // 应用配置 (逻辑等价于 sensorInitOptions)
  bool applyOptions(const struct EpsonProperties& esensor,
                    const struct EpsonOptions& options) {
    // Disable attitude/quaternion output if not supported
    if (!(esensor.feature_flags & HAS_ATTITUDE_OUTPUT)) {
      if (options.atti_out || options.qtn_out) return false;
    }
    // Disable delta output if not supported
    if (!(esensor.feature_flags & HAS_DLT_OUTPUT)) {
      if (options.gyro_delta_out || options.accel_delta_out) return false;
    }

    // SIG_CTRL
    int sig_ctrl_lo = 0;
    if (esensor.feature_flags & HAS_DLT_OUTPUT) {
      sig_ctrl_lo = (options.accel_delta_out & 0x01) << 2 |
                    (options.accel_delta_out & 0x01) << 3 |
                    (options.accel_delta_out & 0x01) << 4 |
                    (options.gyro_delta_out & 0x01) << 5 |
                    (options.gyro_delta_out & 0x01) << 6 |
                    (options.gyro_delta_out & 0x01) << 7;
    }
    int sig_ctrl_hi =
      (options.accel_out & 0x01) << 1 | (options.accel_out & 0x01) << 2 |
      (options.accel_out & 0x01) << 3 | (options.gyro_out & 0x01) << 4 |
      (options.gyro_out & 0x01) << 5 | (options.gyro_out & 0x01) << 6 |
      (options.temp_out & 0x01) << 7;

    // MSC_CTRL
    int msc_ctrl_lo = 0;
    if (esensor.model == G570PR20) {
      msc_ctrl_lo = ((options.drdy_pol & 0x01) << 1) |
                    ((options.drdy_on & 0x01) << 2);
    } else {
      msc_ctrl_lo = (options.drdy_pol & 0x01) << 1 |
                    (options.drdy_on & 0x01) << 2 |
                    (options.ext_pol & 0x01) << 5 |
                    (options.ext_sel & 0x03) << 6;
    }

    // SMPL_CTRL / FILTER_CTRL
    int smpl_ctrl_hi = (options.dout_rate & 0x0F);
    int filter_ctrl_lo = (options.filter_sel & 0x1F);

    // BURST_CTRL1
    int burst_ctrl1_lo = (options.checksum_out & 0x1) |
                         (options.count_out & 0x1) << 1 |
                         (options.gpio_out & 0x01) << 2;
    int burst_ctrl1_hi = 0;
    if (esensor.feature_flags & HAS_ATTITUDE_OUTPUT) {
      burst_ctrl1_hi |= ((options.atti_out & 0x1) |
                         (options.qtn_out & 0x1) << 1);
    }
    if (esensor.feature_flags & HAS_DLT_OUTPUT) {
      burst_ctrl1_hi |= ((options.accel_delta_out & 0x1) << 2 |
                         (options.gyro_delta_out & 0x01) << 3);
    }
    burst_ctrl1_hi |=
      ((options.accel_out & 0x01) << 4 | (options.gyro_out & 0x01) << 5 |
       (options.temp_out & 0x01) << 6 | (options.flag_out & 0x01) << 7);

    // BURST_CTRL2
    int burst_ctrl2_hi = 0;
    if (esensor.feature_flags & HAS_ATTITUDE_OUTPUT) {
      burst_ctrl2_hi |= ((options.atti_bit & 0x1) |
                         (options.qtn_bit & 0x01) << 1);
    }
    if (esensor.feature_flags & HAS_DLT_OUTPUT) {
      burst_ctrl2_hi |= ((options.accel_delta_bit & 0x01) << 2 |
                         (options.gyro_delta_bit & 0x01) << 3);
    }
    burst_ctrl2_hi |=
      ((options.accel_bit & 0x01) << 4 | (options.gyro_bit & 0x01) << 5 |
       (options.temp_bit & 0x01) << 6);

    // POL_CTRL
    int pol_ctrl_lo =
      (options.invert_zaccel & 0x01) << 1 |
      (options.invert_yaccel & 0x01) << 2 |
      (options.invert_xaccel & 0x01) << 3 |
      (options.invert_zgyro & 0x01) << 4 |
      (options.invert_ygyro & 0x01) << 5 |
      (options.invert_xgyro & 0x01) << 6;

    // DLT_CTRL
    int dlt_ctrl_hi = 0;
    if (esensor.feature_flags & HAS_ARANGE) {
      dlt_ctrl_hi = (options.a_range_ctrl & 0x01);
    }
    int dlt_ctrl_lo = 0;
    if (esensor.feature_flags & HAS_DLT_OUTPUT) {
      dlt_ctrl_lo = (options.dlta_range_ctrl & 0x0F) << 4 |
                    (options.dltv_range_ctrl & 0x0F);
    }

    // ATTI_CTRL
    int atti_ctrl_hi = 0;
    int atti_ctrl_lo = 0;
    if (esensor.feature_flags & HAS_ATTITUDE_OUTPUT) {
      atti_ctrl_hi |= (((options.atti_out | options.qtn_out) & 0x01) << 2 |
                       (options.atti_mode & 0x01) << 3);
      if (!options.qtn_out) {
        atti_ctrl_lo = (options.atti_conv & 0x1f);
      }
    }
    if ((esensor.feature_flags & (HAS_DLT_OUTPUT | HAS_ATTI_ON_REG)) ==
        (HAS_DLT_OUTPUT | HAS_ATTI_ON_REG)) {
      atti_ctrl_hi |=
        (((options.gyro_delta_out & 0x01) | (options.accel_delta_out & 0x01))
         << 1);
    }

    // GLOB_CMD2
    int glob_cmd2_lo = 0;
    if (esensor.feature_flags & HAS_ATTITUDE_OUTPUT) {
      glob_cmd2_lo = (options.atti_profile & 0x03) << 4;
    }

    if (!(esensor.model == G570PR20)) {
      registerWriteByte(WIN_ID1, ADDR_SIG_CTRL_HI, sig_ctrl_hi);
      registerWriteByte(WIN_ID1, ADDR_SIG_CTRL_LO, sig_ctrl_lo);
    }
    registerWriteByte(WIN_ID1, ADDR_MSC_CTRL_LO, msc_ctrl_lo);
    registerWriteByte(WIN_ID1, ADDR_SMPL_CTRL_HI, smpl_ctrl_hi);
    registerWriteByte(WIN_ID1, ADDR_FILTER_CTRL_LO, filter_ctrl_lo);
    usleep(esensor.delay_filter_ms * 1000);

    // 等待 FILTER_BUSY 清零
    unsigned short rxData;
    unsigned short retryCount = 3000;
    do {
      rxData = registerRead16(WIN_ID1, ADDR_FILTER_CTRL_LO);
      retryCount--;
    } while ((rxData & BIT5) && (retryCount > 0));
    if (retryCount == 0) return false;

    // UART 接口必须启用 UART_AUTO 模式
    registerWriteByte(WIN_ID1, ADDR_UART_CTRL_LO, 0x01);

    registerWriteByte(WIN_ID1, ADDR_BURST_CTRL1_LO, burst_ctrl1_lo);
    registerWriteByte(WIN_ID1, ADDR_BURST_CTRL1_HI, burst_ctrl1_hi);
    registerWriteByte(WIN_ID1, ADDR_BURST_CTRL2_HI, burst_ctrl2_hi);
    registerWriteByte(WIN_ID1, ADDR_POL_CTRL_LO, pol_ctrl_lo);

    if (esensor.feature_flags & HAS_ARANGE) {
      registerWriteByte(WIN_ID1, ADDR_DLT_CTRL_HI, dlt_ctrl_hi);
    }
    if (esensor.feature_flags & HAS_DLT_OUTPUT) {
      registerWriteByte(WIN_ID1, ADDR_DLT_CTRL_LO, dlt_ctrl_lo);
    }
    if (esensor.feature_flags & HAS_ATTI_ON_REG) {
      registerWriteByte(WIN_ID1, ADDR_ATTI_CTRL_HI, atti_ctrl_hi);
    }
    if (esensor.feature_flags & HAS_ATTITUDE_OUTPUT) {
      registerWriteByte(WIN_ID1, ADDR_ATTI_CTRL_LO, atti_ctrl_lo);
      registerWriteByte(WIN_ID1, ADDR_GLOB_CMD2_LO, glob_cmd2_lo);
      usleep(esensor.delay_atti_profile_ms * 1000);
      retryCount = 3000;
      do {
        rxData = registerRead16(WIN_ID1, ADDR_GLOB_CMD2_LO);
        retryCount--;
      } while ((rxData & BIT6) && (retryCount > 0));
      if (retryCount == 0) return false;
    }
    return true;
  }

  // Burst 包数据长度 (逻辑等价于 sensorDataByteLength)
  unsigned int burstLength(const struct EpsonProperties& esensor,
                           const struct EpsonOptions& options) const {
    unsigned int length = 0;
    if (options.flag_out) length += 2;
    if (options.temp_out) length += options.temp_bit ? 4 : 2;
    if (options.gyro_out) length += options.gyro_bit ? 12 : 6;
    if (options.accel_out) length += options.accel_bit ? 12 : 6;
    if (esensor.feature_flags & HAS_DLT_OUTPUT) {
      if (options.gyro_delta_out) length += options.gyro_delta_bit ? 12 : 6;
      if (options.accel_delta_out) length += options.accel_delta_bit ? 12 : 6;
    }
    if (esensor.feature_flags & HAS_ATTITUDE_OUTPUT) {
      if (options.qtn_out) length += options.qtn_bit ? 16 : 8;
      if (options.atti_out) length += options.atti_bit ? 12 : 6;
    }
    if (options.gpio_out) length += 2;
    if (options.count_out) length += 2;
    if (options.checksum_out) length += 2;
    length += 2;  // UART START + END 标记
    return length;
  }

  // 读取一帧 Burst 数据 (状态机 + 校验和), 成功返回 true
  bool readBurst(const struct EpsonProperties& esensor,
                 const struct EpsonOptions& options,
                 struct EpsonData& data) {
    unsigned int byte_length = burstLength(esensor, options);
    int data_length = (int)byte_length - 2;
    unsigned char byte;

    while (uart_->waitReadable(100)) {
      if (uart_->read(&byte, 1) <= 0) break;
      switch (state_) {
        case 0:  // START
          if (byte == UART_HEADER) state_ = 1;
          break;
        case 1:  // DATA
          rxBuf_[data_count_++] = byte;
          if (data_count_ == data_length) state_ = 2;
          break;
        case 2:  // END
          data_count_ = 0;
          state_ = 0;
          if (byte == UART_DELIMITER) {
            if (options.checksum_out == 1) {
              unsigned short calc = 0;
              for (int i = 0; i < data_length - 2; i += 2) {
                calc += (rxBuf_[i] << 8) + rxBuf_[i + 1];
              }
              unsigned short epson_checksum =
                (rxBuf_[data_length - 2] << 8) + rxBuf_[data_length - 1];
              if (calc != epson_checksum) {
                return false;  // 校验失败, 丢弃本帧
              }
            }
            scaleData(esensor, options, data);
            return true;
          }
          break;
        default:
          state_ = 0;
          data_count_ = 0;
          break;
      }
    }
    return false;  // 超时或无完整帧
  }

  // 解析 Burst 数据 (逻辑等价于 sensorDataScaling)
  void scaleData(const struct EpsonProperties& esensor,
                 const struct EpsonOptions& options,
                 struct EpsonData& data) {
    int idx = 0;

    if (options.flag_out) {
      unsigned short ndflags = (rxBuf_[idx] << 8) + rxBuf_[idx + 1];
      data.ndflags = ndflags;
      idx += 2;
    }

    if (options.temp_out) {
      if (options.temp_bit) {
        int temp = (rxBuf_[idx] << 8 * 3) + (rxBuf_[idx + 1] << 8 * 2) +
                   (rxBuf_[idx + 2] << 8) + rxBuf_[idx + 3];
        if ((esensor.model == G330PDG0) || (esensor.model == G366PDG0) ||
            (esensor.model == G370PDG0) || (esensor.model == G370PDT0) ||
            (esensor.model == G570PR20)) {
          data.temperature = temp * esensor.tempc_sf_degc / 65536 + 25;
        } else {
          data.temperature = (temp - esensor.tempc_25c_offset * 65536) *
                               esensor.tempc_sf_degc / 65536 +
                             25;
        }
        idx += 4;
      } else {
        short temp = (rxBuf_[idx] << 8) + rxBuf_[idx + 1];
        if ((esensor.model == G330PDG0) || (esensor.model == G366PDG0) ||
            (esensor.model == G370PDG0) || (esensor.model == G370PDT0) ||
            (esensor.model == G570PR20)) {
          data.temperature = temp * esensor.tempc_sf_degc + 25;
        } else {
          data.temperature =
            (temp - esensor.tempc_25c_offset) * esensor.tempc_sf_degc + 25;
        }
        idx += 2;
      }
    }

    if (options.gyro_out) {
      if (options.gyro_bit) {
        int gyro_x = (rxBuf_[idx] << 8 * 3) + (rxBuf_[idx + 1] << 8 * 2) +
                     (rxBuf_[idx + 2] << 8) + rxBuf_[idx + 3];
        int gyro_y = (rxBuf_[idx + 4] << 8 * 3) +
                     (rxBuf_[idx + 5] << 8 * 2) + (rxBuf_[idx + 6] << 8) +
                     rxBuf_[idx + 7];
        int gyro_z = (rxBuf_[idx + 8] << 8 * 3) +
                     (rxBuf_[idx + 9] << 8 * 2) + (rxBuf_[idx + 10] << 8) +
                     rxBuf_[idx + 11];
        data.gyro_x = (esensor.gyro_sf_dps / 65536) * DEG2RAD * gyro_x;
        data.gyro_y = (esensor.gyro_sf_dps / 65536) * DEG2RAD * gyro_y;
        data.gyro_z = (esensor.gyro_sf_dps / 65536) * DEG2RAD * gyro_z;
        idx += 12;
      } else {
        short gyro_x = (rxBuf_[idx] << 8) + rxBuf_[idx + 1];
        short gyro_y = (rxBuf_[idx + 2] << 8) + rxBuf_[idx + 3];
        short gyro_z = (rxBuf_[idx + 4] << 8) + rxBuf_[idx + 5];
        data.gyro_x = esensor.gyro_sf_dps * DEG2RAD * gyro_x;
        data.gyro_y = esensor.gyro_sf_dps * DEG2RAD * gyro_y;
        data.gyro_z = esensor.gyro_sf_dps * DEG2RAD * gyro_z;
        idx += 6;
      }
    }

    if (options.accel_out) {
      if (options.accel_bit) {
        int accel_x = (rxBuf_[idx] << 8 * 3) + (rxBuf_[idx + 1] << 8 * 2) +
                      (rxBuf_[idx + 2] << 8) + rxBuf_[idx + 3];
        int accel_y = (rxBuf_[idx + 4] << 8 * 3) +
                      (rxBuf_[idx + 5] << 8 * 2) + (rxBuf_[idx + 6] << 8) +
                      rxBuf_[idx + 7];
        int accel_z = (rxBuf_[idx + 8] << 8 * 3) +
                      (rxBuf_[idx + 9] << 8 * 2) + (rxBuf_[idx + 10] << 8) +
                      rxBuf_[idx + 11];
        data.accel_x = (esensor.accl_sf_mg / 65536) * MG2MPS2 * accel_x;
        data.accel_y = (esensor.accl_sf_mg / 65536) * MG2MPS2 * accel_y;
        data.accel_z = (esensor.accl_sf_mg / 65536) * MG2MPS2 * accel_z;
        idx += 12;
      } else {
        short accel_x = (rxBuf_[idx] << 8) + rxBuf_[idx + 1];
        short accel_y = (rxBuf_[idx + 2] << 8) + rxBuf_[idx + 3];
        short accel_z = (rxBuf_[idx + 4] << 8) + rxBuf_[idx + 5];
        data.accel_x = esensor.accl_sf_mg * MG2MPS2 * accel_x;
        data.accel_y = esensor.accl_sf_mg * MG2MPS2 * accel_y;
        data.accel_z = esensor.accl_sf_mg * MG2MPS2 * accel_z;
        idx += 6;
      }
    }

    if (options.gyro_delta_out) {
      double da_sf =
        esensor.dlta0_sf_deg * (1 << options.dlta_range_ctrl) * DEG2RAD;
      if (options.gyro_delta_bit) {
        int gyro_delta_x = (rxBuf_[idx] << 8 * 3) +
                           (rxBuf_[idx + 1] << 8 * 2) +
                           (rxBuf_[idx + 2] << 8) + rxBuf_[idx + 3];
        int gyro_delta_y = (rxBuf_[idx + 4] << 8 * 3) +
                           (rxBuf_[idx + 5] << 8 * 2) +
                           (rxBuf_[idx + 6] << 8) + rxBuf_[idx + 7];
        int gyro_delta_z = (rxBuf_[idx + 8] << 8 * 3) +
                           (rxBuf_[idx + 9] << 8 * 2) +
                           (rxBuf_[idx + 10] << 8) + rxBuf_[idx + 11];
        data.gyro_delta_x = gyro_delta_x * da_sf / 65536;
        data.gyro_delta_y = gyro_delta_y * da_sf / 65536;
        data.gyro_delta_z = gyro_delta_z * da_sf / 65536;
        idx += 12;
      } else {
        short gyro_delta_x = (rxBuf_[idx] << 8) + rxBuf_[idx + 1];
        short gyro_delta_y = (rxBuf_[idx + 2] << 8) + rxBuf_[idx + 3];
        short gyro_delta_z = (rxBuf_[idx + 4] << 8) + rxBuf_[idx + 5];
        data.gyro_delta_x = gyro_delta_x * da_sf;
        data.gyro_delta_y = gyro_delta_y * da_sf;
        data.gyro_delta_z = gyro_delta_z * da_sf;
        idx += 6;
      }
    }

    if (options.accel_delta_out) {
      double dv_sf = esensor.dltv0_sf_mps * (1 << options.dltv_range_ctrl);
      if (options.accel_delta_bit) {
        int accel_delta_x = (rxBuf_[idx] << 8 * 3) +
                            (rxBuf_[idx + 1] << 8 * 2) +
                            (rxBuf_[idx + 2] << 8) + rxBuf_[idx + 3];
        int accel_delta_y = (rxBuf_[idx + 4] << 8 * 3) +
                            (rxBuf_[idx + 5] << 8 * 2) +
                            (rxBuf_[idx + 6] << 8) + rxBuf_[idx + 7];
        int accel_delta_z = (rxBuf_[idx + 8] << 8 * 3) +
                            (rxBuf_[idx + 9] << 8 * 2) +
                            (rxBuf_[idx + 10] << 8) + rxBuf_[idx + 11];
        data.accel_delta_x = accel_delta_x * dv_sf / 65536;
        data.accel_delta_y = accel_delta_y * dv_sf / 65536;
        data.accel_delta_z = accel_delta_z * dv_sf / 65536;
        idx += 12;
      } else {
        short accel_delta_x = (rxBuf_[idx] << 8) + rxBuf_[idx + 1];
        short accel_delta_y = (rxBuf_[idx + 2] << 8) + rxBuf_[idx + 3];
        short accel_delta_z = (rxBuf_[idx + 4] << 8) + rxBuf_[idx + 5];
        data.accel_delta_x = accel_delta_x * dv_sf;
        data.accel_delta_y = accel_delta_y * dv_sf;
        data.accel_delta_z = accel_delta_z * dv_sf;
        idx += 6;
      }
    }

    if (options.qtn_out) {
      if (options.qtn_bit) {
        int qtn0 = (rxBuf_[idx] << 8 * 3) + (rxBuf_[idx + 1] << 8 * 2) +
                   (rxBuf_[idx + 2] << 8) + rxBuf_[idx + 3];
        int qtn1 = (rxBuf_[idx + 4] << 8 * 3) +
                   (rxBuf_[idx + 5] << 8 * 2) + (rxBuf_[idx + 6] << 8) +
                   rxBuf_[idx + 7];
        int qtn2 = (rxBuf_[idx + 8] << 8 * 3) +
                   (rxBuf_[idx + 9] << 8 * 2) + (rxBuf_[idx + 10] << 8) +
                   rxBuf_[idx + 11];
        int qtn3 = (rxBuf_[idx + 12] << 8 * 3) +
                   (rxBuf_[idx + 13] << 8 * 2) + (rxBuf_[idx + 14] << 8) +
                   rxBuf_[idx + 15];
        data.qtn0 = (double)qtn0 * esensor.qtn_sf / 65536;
        data.qtn1 = (double)qtn1 * esensor.qtn_sf / 65536;
        data.qtn2 = (double)qtn2 * esensor.qtn_sf / 65536;
        data.qtn3 = (double)qtn3 * esensor.qtn_sf / 65536;
        idx += 16;
      } else {
        short qtn0 = (rxBuf_[idx] << 8) + rxBuf_[idx + 1];
        short qtn1 = (rxBuf_[idx + 2] << 8) + rxBuf_[idx + 3];
        short qtn2 = (rxBuf_[idx + 4] << 8) + rxBuf_[idx + 5];
        short qtn3 = (rxBuf_[idx + 6] << 8) + rxBuf_[idx + 7];
        data.qtn0 = (double)qtn0 * esensor.qtn_sf;
        data.qtn1 = (double)qtn1 * esensor.qtn_sf;
        data.qtn2 = (double)qtn2 * esensor.qtn_sf;
        data.qtn3 = (double)qtn3 * esensor.qtn_sf;
        idx += 8;
      }
    }

    if (options.atti_out) {
      if (options.atti_bit) {
        int ang1 = (rxBuf_[idx] << 8 * 3) + (rxBuf_[idx + 1] << 8 * 2) +
                   (rxBuf_[idx + 2] << 8) + rxBuf_[idx + 3];
        int ang2 = (rxBuf_[idx + 4] << 8 * 3) +
                   (rxBuf_[idx + 5] << 8 * 2) + (rxBuf_[idx + 6] << 8) +
                   rxBuf_[idx + 7];
        int ang3 = (rxBuf_[idx + 8] << 8 * 3) +
                   (rxBuf_[idx + 9] << 8 * 2) + (rxBuf_[idx + 10] << 8) +
                   rxBuf_[idx + 11];
        data.ang1 = (esensor.ang_sf_deg / 65536) * DEG2RAD * ang1;
        data.ang2 = (esensor.ang_sf_deg / 65536) * DEG2RAD * ang2;
        data.ang3 = (esensor.ang_sf_deg / 65536) * DEG2RAD * ang3;
        idx += 12;
      } else {
        short ang1 = (rxBuf_[idx] << 8) + rxBuf_[idx + 1];
        short ang2 = (rxBuf_[idx + 2] << 8) + rxBuf_[idx + 3];
        short ang3 = (rxBuf_[idx + 4] << 8) + rxBuf_[idx + 5];
        data.ang1 = esensor.ang_sf_deg * DEG2RAD * ang1;
        data.ang2 = esensor.ang_sf_deg * DEG2RAD * ang2;
        data.ang3 = esensor.ang_sf_deg * DEG2RAD * ang3;
        idx += 6;
      }
    }

    if (options.gpio_out) {
      unsigned short gpio = (rxBuf_[idx] << 8) + rxBuf_[idx + 1];
      data.gpio = gpio;
      idx += 2;
    }

    if (options.count_out) {
      int count = (rxBuf_[idx] << 8) + rxBuf_[idx + 1];
      if (options.ext_sel == 1)
        data.count = (int)(count * esensor.rstcnt_sf_micros);
      else
        data.count = count;
    }
  }

 private:
  EpsonUart* uart_;
  unsigned char rxBuf_[256];
  int state_ = 0;        // 0=START 1=DATA 2=END
  int data_count_ = 0;
};

//=========================================================================
// 系统工具函数
//=========================================================================

// 自然排序: ttyUSB2 < ttyUSB10
static bool naturalLess(const std::string& a, const std::string& b) {
  std::string ba = a.substr(a.find_last_of('/') + 1);
  std::string bb = b.substr(b.find_last_of('/') + 1);
  size_t ia = 0, ib = 0;
  while (ia < ba.size() && ib < bb.size()) {
    if (std::isdigit(ba[ia]) && std::isdigit(bb[ib])) {
      size_t ja = ia;
      while (ja < ba.size() && std::isdigit(ba[ja])) ja++;
      size_t jb = ib;
      while (jb < bb.size() && std::isdigit(bb[jb])) jb++;
      long na = std::strtol(ba.substr(ia, ja - ia).c_str(), nullptr, 10);
      long nb = std::strtol(bb.substr(ib, jb - ib).c_str(), nullptr, 10);
      if (na != nb) return na < nb;
      ia = ja;
      ib = jb;
    } else {
      if (ba[ia] != bb[ib]) return ba[ia] < bb[ib];
      ia++;
      ib++;
    }
  }
  return ba.size() < bb.size();
}

// 扫描串口设备, realpath 去重 (优先保留 imu_ 链接), 自然排序
static std::vector<std::string> scanDevices(const std::string& glob_patterns) {
  std::map<std::string, std::string> found;  // realpath -> 原始路径
  std::istringstream iss(glob_patterns);
  std::string pat;
  while (iss >> pat) {
    glob_t g;
    if (glob(pat.c_str(), 0, nullptr, &g) == 0) {
      for (size_t i = 0; i < g.gl_pathc; i++) {
        std::string orig = g.gl_pathv[i];
        char real[PATH_MAX];
        std::string rp =
          realpath(orig.c_str(), real) ? std::string(real) : orig;
        std::string base = orig.substr(orig.find_last_of('/') + 1);
        auto it = found.find(rp);
        if (it == found.end() || base.rfind("imu_", 0) == 0) {
          found[rp] = orig;
        }
      }
      globfree(&g);
    }
  }
  std::vector<std::string> devs;
  for (const auto& kv : found) devs.push_back(kv.second);
  std::sort(devs.begin(), devs.end(), naturalLess);
  return devs;
}

// 通过 sysfs 读取 USB 串口设备的硬件序列号 (CH343)
static std::string usbSerialFor(const std::string& device) {
  char real[PATH_MAX];
  if (!realpath(device.c_str(), real)) return "";
  std::string base = ::basename(real);
  std::string sysfs = "/sys/class/tty/" + base + "/device";
  char iface[PATH_MAX];
  if (!realpath(sysfs.c_str(), iface)) return "";
  std::string serial_file = std::string(::dirname(iface)) + "/serial";
  std::ifstream f(serial_file);
  std::string s;
  f >> s;
  return s;
}

// 通过 sysfs 读取 USB 设备 VID (PRODUCT=vid:pid:rev)
static std::string usbVidFor(const std::string& device) {
  char real[PATH_MAX];
  if (!realpath(device.c_str(), real)) return "";
  std::string base = ::basename(real);
  std::string uevent = "/sys/class/tty/" + base + "/device/uevent";
  std::ifstream f(uevent);
  std::string line;
  while (std::getline(f, line)) {
    if (line.rfind("PRODUCT=", 0) == 0) {
      std::string v = line.substr(std::string("PRODUCT=").size());
      return v.substr(0, v.find('/'));
    }
  }
  return "";
}

//=========================================================================
// ImuDevice - 单个 IMU 设备: 串口 + 读取线程 + 发布器
//=========================================================================
class ImuDevice {
 public:
  using ImuPub = rclcpp::Publisher<sensor_msgs::msg::Imu>;
  using TempPub = rclcpp::Publisher<sensor_msgs::msg::Temperature>;

  ImuDevice(rclcpp::Node* node, const std::string& path,
            const std::string& name, const struct EpsonOptions& opts,
            int baud, bool publish_temp)
    : node_(node),
      path_(path),
      name_(name),
      opts_(opts),
      baud_(baud),
      publish_temp_(publish_temp),
      proto_(&uart_) {}

  ~ImuDevice() { stop(); }

  const std::string& path() const { return path_; }
  const std::string& name() const { return name_; }
  const std::string& model() const { return model_; }
  const std::string& imu_serial() const { return imu_serial_; }

  // 打开串口 + 初始化 IMU + 创建发布器 + 启动读取线程
  bool start(rclcpp::Logger logger) {
    if (!uart_.open(path_, baud_)) {
      RCLCPP_ERROR(logger, "[%s] 无法打开串口 %s", name_.c_str(),
                   path_.c_str());
      return false;
    }

    // 上次进程可能未正常停止, IMU 停留在 Sampling 模式持续输出数据:
    // 先拉回 Config 模式并清空残留缓冲, 否则 powerOn 读寄存器
    // 会被串口里的残留数据帧干扰而校验失败
    proto_.resetState();
    proto_.stop();
    proto_.resetState();

    if (!proto_.powerOn()) {
      RCLCPP_ERROR(logger, "[%s] IMU 上电/ID 校验失败 (%s)", name_.c_str(),
                   path_.c_str());
      uart_.close();
      return false;
    }

    std::string prod, ser;
    if (!proto_.detectModel(props_, prod, ser)) {
      RCLCPP_ERROR(logger, "[%s] 无法识别 IMU 型号 (PROD_ID=%s)",
                   name_.c_str(), prod.c_str());
      uart_.close();
      return false;
    }
    model_ = props_.product_id;
    imu_serial_ = ser;

    if (!proto_.applyOptions(props_, opts_)) {
      RCLCPP_ERROR(logger, "[%s] IMU 配置失败 (%s)", name_.c_str(),
                   model_.c_str());
      uart_.close();
      return false;
    }

    proto_.start();
    proto_.resetState();
    uart_.purge();

    pub_raw_ =
      node_->create_publisher<sensor_msgs::msg::Imu>("/" + name_ + "/data_raw", 20);
    if (publish_temp_) {
      pub_temp_ = node_->create_publisher<sensor_msgs::msg::Temperature>(
        "/" + name_ + "/tempc", 20);
    }

    running_ = true;
    thread_ = std::thread(&ImuDevice::loop, this);
    return true;
  }

  // 停止: 停线程 -> 关串口 -> 释放发布器 (话题从图中消失)
  void stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
    if (uart_.valid()) {
      proto_.stop();
      uart_.close();
    }
    pub_raw_.reset();
    pub_temp_.reset();
  }

 private:
  void loop() {
    while (running_ && uart_.valid()) {
      struct EpsonData d = {};
      if (proto_.readBurst(props_, opts_, d)) {
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = node_->now();
        msg.header.frame_id = name_ + "_link";

        // raw 驱动不输出姿态 (REP-145 约定):
        //   orientation_covariance[0] = -1 表示 orientation 无效,
        //   消费者应忽略 orientation 字段
        msg.orientation.w = 1.0;  // 单位四元数 (占位, 无姿态)
        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation_covariance[0] = -1;

        msg.angular_velocity.x = d.gyro_x;
        msg.angular_velocity.y = d.gyro_y;
        msg.angular_velocity.z = d.gyro_z;

        msg.linear_acceleration.x = d.accel_x;
        msg.linear_acceleration.y = d.accel_y;
        msg.linear_acceleration.z = d.accel_z;

        if (pub_raw_) pub_raw_->publish(msg);

        if (pub_temp_) {
          auto t = sensor_msgs::msg::Temperature();
          t.header = msg.header;
          t.temperature = d.temperature;
          t.variance = 0;
          pub_temp_->publish(t);
        }
      }
    }
  }

  rclcpp::Node* node_;
  std::string path_;
  std::string name_;
  struct EpsonOptions opts_;
  int baud_;
  bool publish_temp_;

  EpsonUart uart_;
  EpsonProtocol proto_;
  struct EpsonProperties props_ = {};
  std::string model_;
  std::string imu_serial_;

  std::thread thread_;
  std::atomic<bool> running_{false};

  ImuPub::SharedPtr pub_raw_;
  TempPub::SharedPtr pub_temp_;
};

//=========================================================================
// ImuHubNode - 多 IMU 热插拔管理节点
//=========================================================================
class ImuHubNode : public rclcpp::Node {
 public:
  explicit ImuHubNode(const rclcpp::NodeOptions& op)
    : Node("imu_hub", op) {
    ParseParams();
    scan_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((int)(poll_interval_ * 1000.0)),
      std::bind(&ImuHubNode::scan, this));
    RCLCPP_INFO(this->get_logger(), "IMU Hub 已启动: glob='%s' 扫描周期=%.1fs 波特率=%d",
                glob_patterns_.c_str(), poll_interval_, baud_);
  }

  ~ImuHubNode() override {
    for (auto& kv : devices_) {
      kv.second->stop();
    }
    devices_.clear();
  }

 private:
  void ParseParams() {
    glob_patterns_ = this->declare_parameter<std::string>(
      "device_glob", "/dev/imu_* /dev/ttyUSB* /dev/ttyACM*");
    prefix_ = this->declare_parameter<std::string>("prefix", "imu");
    baud_ = this->declare_parameter<int>("baud_rate", 460800);
    poll_interval_ = this->declare_parameter<double>("poll_interval", 1.0);
    filter_vid_ = this->declare_parameter<std::string>("filter_vid", "");
    publish_temp_ = this->declare_parameter<bool>("publish_temperature", true);
    imu_dout_rate_ = this->declare_parameter<int>("imu_dout_rate", CMD_RATE200);
    imu_filter_sel_ = this->declare_parameter<int>("imu_filter_sel", CMD_FLTAP32);
    std::vector<std::string> binding_pairs =
      this->declare_parameter<std::vector<std::string>>("bindings",
                                                        std::vector<std::string>{});

    // 解析绑定: "A1=5C38170227" -> bindings_[serial] = name
    for (const auto& pair : binding_pairs) {
      auto pos = pair.find('=');
      if (pos == std::string::npos) continue;
      std::string name = pair.substr(0, pos);
      std::string serial = pair.substr(pos + 1);
      if (!name.empty() && !serial.empty()) {
        bindings_[serial] = name;
      }
    }
    // 未绑定设备编号从绑定数量+1 开始 (imu(x+1), 与旧行为一致)
    auto_index_ = (int)bindings_.size() + 1;

    // IMU 配置选项 (只输出 raw: 加速度 + 角速度 + 温度)
    options_.ext_sel = 1;
    options_.drdy_on = true;
    options_.drdy_pol = 1;
    options_.dout_rate = imu_dout_rate_;
    options_.filter_sel = imu_filter_sel_;
    options_.flag_out = true;
    options_.temp_out = true;
    options_.gyro_out = true;
    options_.accel_out = true;
    options_.qtn_out = false;
    options_.count_out = true;
    options_.checksum_out = true;
    options_.atti_mode = 1;
    options_.atti_profile = 0;
  }

  // 扫描 /dev: 添加新设备, 移除消失设备 (实时话题删减)
  void scan() {
    auto found = scanDevices(glob_patterns_);

    // 1. 移除已消失的设备
    for (auto it = devices_.begin(); it != devices_.end();) {
      if (std::find(found.begin(), found.end(), it->first) == found.end()) {
        auto& dev = it->second;
        RCLCPP_INFO(this->get_logger(),
                    "设备移除: %s (%s, /%s/data_raw 话题已删除)", it->first.c_str(),
                    dev->name().c_str(), dev->name().c_str());
        dev->stop();
        used_names_.erase(dev->name());
        it = devices_.erase(it);
      } else {
        ++it;
      }
    }

    // 2. 添加新发现的设备
    auto now = std::chrono::steady_clock::now();
    for (const auto& dev_path : found) {
      if (devices_.count(dev_path)) continue;

      // VID 过滤 (可选)
      if (!filter_vid_.empty() && usbVidFor(dev_path) != filter_vid_) continue;

      // 失败冷却: 10 秒内不重试, 避免日志刷屏
      auto cit = cooldown_.find(dev_path);
      if (cit != cooldown_.end()) {
        if (now - cit->second < std::chrono::seconds(10)) continue;
        cooldown_.erase(cit);
      }

      std::string serial = usbSerialFor(dev_path);
      std::string name = nameFor(serial);
      auto dev = std::make_shared<ImuDevice>(this, dev_path, name, options_,
                                             baud_, publish_temp_);
      if (dev->start(this->get_logger())) {
        devices_[dev_path] = dev;
        RCLCPP_INFO(this->get_logger(),
                    "新 IMU: %s -> 名称 %s, 型号 %s, IMU序列号 %s, USB序列号 %s"
                    " | 话题 /%s/data_raw%s",
                    dev_path.c_str(), name.c_str(), dev->model().c_str(),
                    dev->imu_serial().c_str(), serial.c_str(), name.c_str(),
                    publish_temp_ ? (" + /" + name + "/tempc").c_str() : "");
      } else {
        // 启动失败: 归还已分配的名称 (否则绑定名被自己占用,
        // 冷却后重试会被改成自动编号 imuN)
        used_names_.erase(name);
        cooldown_[dev_path] = now;
      }
    }
  }

  // 名称分配: 绑定命中优先, 否则 prefix + 自动编号
  std::string nameFor(const std::string& serial) {
    auto it = bindings_.find(serial);
    if (it != bindings_.end() && used_names_.count(it->second) == 0) {
      used_names_.insert(it->second);
      RCLCPP_INFO(this->get_logger(), "  ~ USB序列号 %s -> 绑定名称 %s",
                  serial.c_str(), it->second.c_str());
      return it->second;
    }
    std::string n;
    do {
      n = prefix_ + std::to_string(auto_index_++);
    } while (used_names_.count(n));
    used_names_.insert(n);
    return n;
  }

  rclcpp::TimerBase::SharedPtr scan_timer_;
  std::map<std::string, std::shared_ptr<ImuDevice>> devices_;  // 路径 -> 设备
  std::set<std::string> used_names_;
  std::map<std::string, std::chrono::steady_clock::time_point> cooldown_;
  std::map<std::string, std::string> bindings_;  // USB serial -> name

  std::string glob_patterns_;
  std::string prefix_;
  int baud_ = 460800;
  double poll_interval_ = 1.0;
  std::string filter_vid_;
  bool publish_temp_ = true;
  int imu_dout_rate_ = CMD_RATE200;
  int imu_filter_sel_ = CMD_FLTAP32;
  int auto_index_ = 1;
  struct EpsonOptions options_ = {};
};

//=========================================================================
// main
//=========================================================================
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuHubNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
