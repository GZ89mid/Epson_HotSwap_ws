# 多 IMU 同时启动指南 (单进程热插拔驱动)

## 背景

多个 Epson IMU 通过 CH343 USB 转串口接入同一台电脑。Linux 默认按插入顺序
分配 `/dev/ttyUSB0 ~ /dev/ttyUSB3`, **插拔或重启后顺序可能变化**。

本方案 (2026-08 重构后) 用**一个驱动进程**解决全部问题:

1. **实时热插拔**: 插入自动发布话题, 拔出自动删除话题 (无需重启)
2. **绑定 ID**: 按 CH343 USB 序列号绑定名称 (换口/拔插不乱名)
3. **无限 IMU 数量**: 绑定 4 块, 插入第 5 块自动命名 `imu5`
4. **只输出 raw**: 无任何滤波/姿态算法, 话题 `/<名称>/data_raw` + `/<名称>/tempc`

## 一、固定设备名 (udev 规则, 可选)

```bash
cd ~/G-Exo/ros2_ws/src/ess_imu_driver2/tools
sudo bash setup_imu_udev.sh
```

> ⚠️ **前提**: 先把所有 IMU 全部插入电脑 (顺序任意)

脚本会:
1. 扫描所有 CH343 设备 (`/dev/ttyUSB*`)
2. 按 **USB 物理端口路径** 排序, 生成 `/etc/udev/rules.d/99-imu.rules`
3. 打印对应关系: `/dev/imuX <-> USB端口`
4. 自动重新加载规则

> udev 规则**不是命名依据**, 只生成 `/dev/imu_XXX` 便捷链接方便调试。
> 名称绑定完全由 `config/imu_bindings.yaml` 中的 **CH343 序列号** 决定。

验证:

```bash
ls -l /dev/imu*
```

## 二、启动驱动 (只此一步)

```bash
cd ~/G-Exo/ros2_ws
source install/setup.bash
ros2 launch ess_imu_driver2 imu_hub_launch.py
```

### 话题与设备名对照

| 名称 | 绑定序列号 | 数据话题 | 温度话题 | frame_id |
|------|-----------|---------|---------|----------|
| A1 | 5C38170227 | /A1/data_raw | /A1/tempc | A1_link |
| A2 | 5C38170221 | /A2/data_raw | /A2/tempc | A2_link |
| B1 | 5C38170222 | /B1/data_raw | /B1/tempc | B1_link |
| B2 | 5C38170220 | /B2/data_raw | /B2/tempc | B2_link |
| imu5+ | (未绑定) | /imu5/data_raw | /imu5/tempc | imu5_link |

### 绑定关系 (config/imu_bindings.yaml)

```yaml
imu_bindings:
  A1: "5C38170227"
  A2: "5C38170221"
  B1: "5C38170222"
  B2: "5C38170220"
```

**插入超过绑定数量的 IMU 时**, 自动按 `imu5, imu6, ...` 命名, **无上限**。
想增加绑定, 编辑 YAML 加入一行即可, 重启 launch 生效。

### 常用参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `baud_rate` | 460800 | 串口波特率 |
| `imu_dout_rate` | 9 (200Hz) | 数据频率: 4=125Hz, 8=400Hz |
| `imu_filter_sel` | 5 (TAP32) | 芯片内部 FIR 滤波 (硬件, 非软件算法) |
| `poll_interval` | 1.0 | 设备扫描周期 (秒) |
| `filter_vid` | (空) | 只接受该 USB VID, 如 `1a86` 只认 CH343 |
| `publish_temperature` | true | 是否发布温度话题 |

示例:

```bash
# 只认 CH343 + 400Hz
ros2 launch ess_imu_driver2 imu_hub_launch.py filter_vid:=1a86 imu_dout_rate:=8
```

## 三、验证

```bash
# 查看所有话题 (每个 IMU 两个)
ros2 topic list | grep imu

# 查看某个 IMU 数据 (有输出即正常)
ros2 topic echo /A1/data_raw --once

# 查看节点 (只有一个 imu_hub)
ros2 node list

# 查看数据频率
ros2 topic hz /A1/data_raw
```

## 四、热插拔行为

| 操作 | 效果 |
|------|------|
| 插入新 IMU | 1 秒内自动识别, 发布对应话题 (按序列号绑定或自动编号) |
| 拔出 IMU | 1 秒内对应话题**实时删除** (publisher 销毁) |
| 再次插入 | 重新识别, 名称不变 (绑定按序列号) |

无需重启驱动, 无需修改任何配置。

## 五、常见问题

| 问题 | 原因 / 解决 |
|------|-------------|
| 启动报错 `Failed to open serial port` | `/dev/ttyACM*` 不存在, 先插 USB; 权限不足用 `sudo usermod -aG dialout $USER` |
| 权限不足 `Permission denied` | udev 规则中已含 `MODE="0666"`, 重新插拔设备 |
| 多个设备只有部分有数据 | 确认每个 IMU 供电正常、波特率一致 |
| 拔出后话题还在 | 等 1 个扫描周期 (默认 1s) 自动删除 |
| 想接超过 4 块 | 直接插, 自动命名 imu5, imu6, ... 无上限 |
| 需要姿态数据 | 本驱动只输出 raw; 上层订阅 data_raw 自行解算 (EKF/互补滤波等) |
