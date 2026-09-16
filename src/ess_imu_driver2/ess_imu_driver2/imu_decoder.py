"""
Epson IMU UART Burst Protocol Decoder — G365PDC1 / G365PDF1

帧格式 (UART Burst Read, 16-bit):
┌──────┬──────────┬──────┬─────────────┬──────────────┬──────────────┬───────┬──────────┬──────┐
│ 0x80 │ ndflags  │ temp │ gyro x,y,z  │ accel x,y,z  │ qtn q0~q3    │ count │ checksum │ 0x0D │
│  1B  │   2B     │  2B  │    6B       │     6B       │     8B       │  2B   │   2B     │  1B  │
└──────┴──────────┴──────┴─────────────┴──────────────┴──────────────┴───────┴──────────┴──────┘

各字段含义:
  ndflags    — 状态标志位, bit0=ND(新数据), bit1=EA(错误告警)
  temp       — 温度原始值, 换算: (°C) = (raw - 2364) * (-0.0037918) + 25
  gyro x,y,z — 角速度原始值, 换算: (°/s) = raw * (1/66)
  accel x,y,z— 加速度原始值, 换算: (mg)  = raw * (1/6.25)  [PDC1] / raw * (1/2.5)  [PDF1]
  qtn q0~q3  — 四元数原始值, 换算: q = raw / 16384, 然后转为 roll/pitch/yaw
  count      — 采样计数器 (1PPS reset counter)
  checksum   — 校验和 (16-bit 累加)
"""

import struct
import math

RAD2DEG = 180.0 / math.pi

# ========== G365 参数 (PDC1 / PDF1 共用 gyro 和温度) ==========
GYRO_SF_DPS   = 1.0 / 66         # (°/s)/LSB  角速度尺度因子
TEMP_SF       = -0.0037918        # °C/LSB     温度尺度因子
TEMP_OFFSET   = 2364              #            温度 25°C 偏移
QTN_SF        = 1.0 / 16384       #            四元数尺度因子  (2<<13 = 16384)

# G365PDF1: accl_sf_mg = 1/2.5 ≈ 0.40 mg/LSB
ACCL_SF_MG    = 1.0 / 2.5         # mg/LSB     加速度尺度因子
# ============================================================

_buffer = bytearray()


def _int16(b, offset):
    """读取 little-endian 有符号 16-bit"""
    return struct.unpack_from('<h', b, offset)[0]


def _uint16(b, offset):
    """读取 little-endian 无符号 16-bit"""
    return struct.unpack_from('<H', b, offset)[0]


def decode(data: bytes) -> bytes:
    """
    comtool 接收回调: 从串口原始字节流中提取并解析 IMU burst 帧

    每帧 (30 字节):
      [0x80] [ndflags] [temp] [gyro_x gyro_y gyro_z] [accel_x accel_y accel_z]
      [qtn0 qtn1 qtn2 qtn3] [count] [checksum] [0x0D]

    返回一行文本:
      temp:xx.x℃ | gyro(°/s): x.xxx y.yyy z.zzz | accel(mg): x.x y.y z.z | euler(°):r=xx.xx p=yy.yy y=zz.zz
    """
    global _buffer
    _buffer.extend(data)
    lines = []

    while len(_buffer) > 2:
        # 查找帧头 0x80
        h = _buffer.find(0x80)
        if h < 0:
            _buffer.clear()
            break
        if h > 0:
            _buffer = _buffer[h:]

        # 查找帧尾 0x0D (至少在 header 后 1 字节开始找)
        t = _buffer.find(0x0D, 1)
        if t < 0:
            if len(_buffer) > 256:
                _buffer = _buffer[1:]    # 超过 256B 没找到帧尾, 丢弃开头
            break

        # 提取帧体 (去掉头尾)
        frame = bytes(_buffer[1:t])
        _buffer = _buffer[t + 1:]

        # 完整帧至少 28B (ndflags+temp+gyro+accel+qtn+count+chk)
        if len(frame) < 28:
            continue

        try:
            n = len(frame)
            idx = 0
            parts = []

            # 1) ndflags (2B, 跳过不显示)
            #    bit0=0 → 新数据, bit1=1 → IMU 告警
            idx += 2

            # 2) 温度 int16 → °C
            #    公式: t(°C) = (raw - 2364) * (-0.0037918) + 25
            if idx + 2 <= n:
                raw_t = _int16(frame, idx)
                temp_c = (raw_t - TEMP_OFFSET) * TEMP_SF + 25.0
                parts.append(f"temp:{temp_c:.1f}℃")
                idx += 2

            # 3) 角速度 int16 x3 → °/s
            #    公式: g(°/s) = raw * (1/66)
            if idx + 6 <= n:
                gx = _int16(frame, idx)     * GYRO_SF_DPS
                gy = _int16(frame, idx + 2) * GYRO_SF_DPS
                gz = _int16(frame, idx + 4) * GYRO_SF_DPS
                parts.append(f"gyro(°/s):{gx:8.3f} {gy:8.3f} {gz:8.3f}")
                idx += 6

            # 4) 加速度 int16 x3 → mg
            #    PDC1: a(mg) = raw * (1/6.25)
            #    PDF1: a(mg) = raw * (1/2.5)
            if idx + 6 <= n:
                ax = _int16(frame, idx)     * ACCL_SF_MG
                ay = _int16(frame, idx + 2) * ACCL_SF_MG
                az = _int16(frame, idx + 4) * ACCL_SF_MG
                parts.append(f"accel(mg):{ax:8.1f} {ay:8.1f} {az:8.1f}")
                idx += 6

            remaining = n - idx

            # 5) 四元数 int16 x4 → Euler (roll/pitch/yaw)
            #    G365 的四元数定义为 q0~q3 (w, x, y, z)
            #    q = raw / 16384
            #    roll  = atan2(2(wx+yz), 1-2(x²+y²))
            #    pitch = asin(2(wy-zx))
            #    yaw   = atan2(2(wz+xy), 1-2(y²+z²))
            if remaining >= 8:
                qw = _int16(frame, idx)     * QTN_SF
                qx = _int16(frame, idx + 2) * QTN_SF
                qy = _int16(frame, idx + 4) * QTN_SF
                qz = _int16(frame, idx + 6) * QTN_SF

                _2qwqx = 2.0 * qw * qx
                _2qwqy = 2.0 * qw * qy
                _2qwqz = 2.0 * qw * qz
                _2qxqy = 2.0 * qx * qy
                _2qyqz = 2.0 * qy * qz
                _2qxqx = 2.0 * qx * qx
                _2qyqy = 2.0 * qy * qy
                _2qzqz = 2.0 * qz * qz

                roll  = math.atan2(_2qwqx + _2qyqz, 1.0 - _2qxqx - _2qyqy) * RAD2DEG
                pitch = math.asin(max(-1.0, min(1.0, _2qwqy - 2.0 * qz * qx))) * RAD2DEG
                yaw   = math.atan2(_2qwqz + _2qxqy, 1.0 - _2qyqy - _2qzqz) * RAD2DEG

                parts.append(f"euler(°):r={roll:7.2f} p={pitch:7.2f} y={yaw:7.2f}")
                idx += 8
                remaining -= 8

            # 6) count + checksum (各 2B, 跳过不显示)
            #     count 可用于时间同步, checksum 用于数据完整性验证

            lines.append(' | '.join(parts))
        except Exception:
            continue

    return ('\n'.join(lines) + '\n').encode() if lines else b''


def encode(data: bytes) -> bytes:
    return data
