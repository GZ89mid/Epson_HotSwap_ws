# ESS IMU Driver2 — 多 IMU 自动检测与启动 (README)

> 服务器: 192.168.11.66 (Bianbu 4.0.1, riscv64, ROS2 Humble)
> 硬件: 多块 Epson G354PDH0 IMU 载板 (CH343 USB → /dev/ttyACM* / /dev/ttyUSB*)
> 核心: **插入几个 IMU, 自动启动几个节点**, 名称由外部 YAML 绑定

---

## 目录

- [1. 多 IMU 支持逻辑](#1-多-imu-支持逻辑)
- [2. 硬件绑定 (YAML 配置)](#2-硬件绑定-yaml-配置)
- [3. 快速启动](#3-快速启动)
- [4. 常用参数](#4-常用参数)
- [5. 验证数据](#5-验证数据)
- [6. 话题与节点对照](#6-话题与节点对照)
- [7. 物理识别板子](#7-物理识别板子-可选)
- [8. 停止 / 关机](#8-停止--关机)
- [9. 常见问题](#9-常见问题)
- [10. 架构变更记录](#10-架构变更记录)

---

## 1. 多 IMU 支持逻辑

`imu_auto_detect.py` 守护进程每 1 秒扫描串口设备, **无数量上限**:

1. **检测**: 扫描 `/dev/ttyUSB* /dev/ttyACM*` (可自定义 glob), 插入几个就启动几个;
2. **命名**:
   - 设备 **CH343 USB 序列号** 命中 YAML 绑定表 → 使用绑定名称 (如 A1/A2/B1/B2);
   - 未绑定 (多出的设备) → 自动命名 `imu(x+1)`, **x = YAML 绑定数量**;
     例如绑定 4 个时, 第 5/6/7 块自动叫 `imu5 / imu6 / imu7 ...`, 无限递增;
   - 无 YAML 配置 (旧用法) → 回退 `--names` 列表, 用尽后自动 `imuN` 编号;
3. **热插拔**: 插入新 IMU 自动启动对应节点, 拔出自动清理, 无需重启;
4. **自动恢复**: 驱动进程异常退出 → 自动重启 (60s 失败冷却)。

### 命名规则示例

| 设备序列号 | 是否命中 YAML | 命名结果 |
|-----------|--------------|---------|
| 5C38170227 | ✅ A1 | A1 |
| 5C38170221 | ✅ A2 | A2 |
| 5C38170222 | ✅ B1 | B1 |
| 5C38170220 | ✅ B2 | B2 |
| (新板, 未绑定) | ❌ | imu5 (x=4 绑定 → 4+1) |
| (又一块新板) | ❌ | imu6 |

> 绑定 4 个时, 多余设备从 `imu5` 开始; 绑定 2 个时, 第 3 块叫 `imu3`。**x 始终等于 YAML 绑定数量。**

---

## 2. 硬件绑定 (YAML 配置)

IMU 名称与 CH343 硬件 ID 的绑定关系写在**外部 YAML 文件**中 (不写死在代码里):

**文件**: `src/ess_imu_driver2/config/imu_bindings.yaml` (launch 自动加载, 已随包安装)

```yaml
imu_bindings:
  A1: "5C38170227"
  A2: "5C38170221"
  B1: "5C38170222"
  B2: "5C38170220"
```

- 代码通过 sysfs 读取每个设备的 **CH343 USB 序列号** (`/sys/class/tty/<dev>/device/../serial`), 命中 YAML 即用对应名称;
- **修改绑定 = 编辑 YAML 重启 launch**, 无需改代码/重编译;
- udev 规则 (`/etc/udev/rules.d/99-imu.rules`) 仅用于生成 `/dev/imu_XXX` 便捷链接 (非命名依据);
- 验证序列号: `ls /sys/class/tty/ttyACM*/device/../serial`。

---

## 3. 快速启动

```bash
# 0. 开机前检查 (少于 4 块也能启动, 自动按数量工作)
ping 192.168.11.66

# 1. SSH 登录
ssh gexo@192.168.11.66

# 2. 确认设备已识别
ls -l /dev/ttyACM*

# 3. 启动 ROS2 环境
source /opt/ros/humble/setup.bash
cd ~/G-Exo/ros2_ws

# 4. 启动自动检测守护进程 (核心)
ros2 launch ess_imu_driver2 imu_auto_launch.py
```

> ✅ 无需再传 `names:=` — 名称绑定自动从 `config/imu_bindings.yaml` 读取。
> ⚠️ **不要加 `enable_mahony:=true`** — 姿态解算已内嵌进驱动节点 (旧架构遗留)。

启动后终端持续打印 (约每 1 秒):

```
[imu_auto_detect] 21:46:05 + 检测到 /dev/ttyACM0 (序列号 5C38170227) -> YAML 绑定名 A1
[imu_auto_detect] 21:46:05 + 检测到 /dev/ttyACM1 (序列号 5C38170221) -> YAML 绑定名 A2
[imu_auto_detect] 21:46:06 当前 4 个 IMU 在线 | 话题: /A1/ahrs /A2/ahrs /B1/ahrs /B2/ahrs
```

> 该终端保持运行。**插拔 IMU 自动增删节点, 无需重启。**

---

## 4. 常用参数

| 需求 | 命令 |
|------|------|
| 只识别 CH343 (防误认其他串口) | `filter_vid:=1a86` |
| 开启原始数据+温度 | `publish_data_raw:=true publish_temperature:=true` |
| 调整姿态频率 400Hz | `imu_dout_rate:=8` (默认 9=200Hz, 4=125Hz) |
| 自定义扫描设备 | `device_glob:='/dev/ttyUSB*'` |
| 调 mahony 参数 | `mahony_kp:=1.0 mahony_ki:=0.005` |
| 自定义 YAML 绑定文件 | `config:=/path/to/imu_bindings.yaml` |

### ✅ 默认配置 (2026-08-01 优化后)

| 项目 | 默认值 | 说明 |
|------|--------|------|
| 姿态话题 /<名>/ahrs | **开** | Mahony 姿态解算**内嵌进驱动节点** |
| 原始数据 /<名>/data_raw | **关** | `publish_data_raw:=false`, 需要时再开 |
| 温度 /<名>/tempc | **关** | `publish_temperature:=false`, 需要时再开 |
| 姿态发布频率 | **200 Hz** | `imu_dout_rate:=9` |
| 独立 mahony 进程 | **不再启动** | 姿态算法已内嵌 |

---

## 5. 验证数据 (新开终端)

```bash
source /opt/ros/humble/setup.bash

ros2 topic list            # 查看话题
ros2 topic hz /A1/ahrs     # 查看姿态频率 (应 200Hz)
ros2 topic echo /A1/ahrs --once   # 查看姿态 (四元数)
```

> 启动后内嵌 mahony 需约 10~20 秒校准采样 (calib 500 + init 200 + warmup 300) 后才输出姿态。

---

## 6. 话题与节点对照 (默认配置)

| 名称 | 设备 | 驱动节点 | 姿态话题 | frame_id | data_raw/tempc |
|------|------|---------|---------|----------|----------------|
| A1 | /dev/ttyACM0 | A1_node | /A1/ahrs | A1_link | 默认关闭 |
| A2 | /dev/ttyACM1 | A2_node | /A2/ahrs | A2_link | 默认关闭 |
| B1 | /dev/ttyACM3 | B1_node | /B1/ahrs | B1_link | 默认关闭 |
| B2 | /dev/ttyACM2 | B2_node | /B2/ahrs | B2_link | 默认关闭 |
| imu5 (未绑定) | /dev/ttyACM4 | imu5_node | /imu5/ahrs | imu5_link | 默认关闭 |

> 名称按序列号匹配 YAML, 与 ttyACM 编号顺序无关。

---

## 7. 物理识别板子 (可选)

```bash
python3 ~/G-Exo/ros2_ws/src/ess_imu_driver2/tools/monitor_ahrs.py
```

实时显示各板 roll/pitch/yaw, **拿起哪块转动, 对应列变化最大** → 该列名字即这块板。

板子 Epson 序列号: A1=00001368, A2=00001095, B1=00002736, B2=00011726

---

## 8. 停止 / 关机

| 操作 | 方法 |
|------|------|
| 停止算法 | 守护进程终端 `Ctrl+C` (自动清理子节点) |
| 停止后台管理器 | `pkill -f imu_auto_detect.py` |
| 关机 / 重启 | `sudo shutdown -h now` / `sudo reboot` |

---

## 9. 常见问题

| 现象 | 处理 |
|------|------|
| `ls /dev/ttyACM*` 没设备 | 重新插拔 USB; `sudo dmesg \| grep -i usb` |
| 某姿态话题一直不出 | 拔插对应板子; 查看 `/tmp/imu_XXX.log` |
| 想换名称/加新板 | 编辑 `config/imu_bindings.yaml` 加一行, 重启 launch |
| 名称分配错乱 | YAML 按 CH343 序列号绑定不会乱; 检查序列号: `ls /sys/class/tty/ttyACM*/device/../serial` |
| rqt 报 pyqt 错误 | 不影响, 用 rqt_topic 等 Python 插件 |
| 需要原始数据/温度 | `publish_data_raw:=true publish_temperature:=true` |
| 需要更高频率 | `imu_dout_rate:=8` = 400Hz |

---

## 10. 架构变更记录

- **2026-08-02 — YAML 硬件绑定**: 名称↔CH343 序列号绑定从代码/udev 移到外部
  `config/imu_bindings.yaml`; 未绑定设备自动命名 `imu(x+1)` (x=绑定数量); launch 新增 `config` 参数;
  修改文件: `scripts/imu_auto_detect.py`, `launch/imu_auto_launch.py`, `CMakeLists.txt`, `config/imu_bindings.yaml`(新建)。
- **2026-08-01 — 内嵌 Mahony + 默认关闭 raw/tempc**: 姿态解算内嵌驱动节点
  (`src/mahony_embedded.h` 新建); 新增 `publish_data_raw/publish_temperature/publish_ahrs` 参数;
  频率默认 200Hz; 独立 mahony 进程不再默认启动。
