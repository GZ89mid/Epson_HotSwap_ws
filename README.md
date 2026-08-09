# ESS IMU Driver2 — 单进程多 IMU 热插拔驱动

> 硬件: 多块 Epson G354PDH0 IMU 载板 (CH343 USB → /dev/ttyACM* / /dev/ttyUSB*)
> 核心: **一个驱动进程管理任意数量 IMU**, 实时热插拔, 实时话题删减, 只输出 raw 数据

---

## 目录

- [1. 功能一览](#1-功能一览)
- [2. 架构说明](#2-架构说明)
- [3. 硬件绑定 (YAML 配置)](#3-硬件绑定-yaml-配置)
- [4. 快速启动](#4-快速启动)
- [5. 部署到新电脑 (Clone)](#5-部署到新电脑-clone)
- [6. 推送到 GitHub](#6-推送到-github)
- [7. 常用参数](#7-常用参数)
- [8. 验证数据](#8-验证数据)
- [9. 话题与设备对照](#9-话题与设备对照)
- [10. 热插拔行为](#10-热插拔行为)
- [11. 物理识别板子 (可选)](#11-物理识别板子-可选)
- [12. 常见问题](#12-常见问题)

---

## 1. 功能一览

| 功能 | 说明 |
|------|------|
| **单进程驱动** | 只启动 1 个节点 (`imu_hub`), 内部线程管理所有 IMU |
| **实时热插拔** | 插入自动识别并发布话题, 拔出自动删除话题, **无需重启** |
| **实时话题删减** | 设备拔出 → publisher 销毁 → 话题立即从 ROS 图中消失 |
| **绑定 ID** | 按 CH343 USB 序列号绑定名称, 换口/拔插不乱名 |
| **无限 IMU 数量** | 绑定 4 块, 插入第 5 块自动命名 `imu5`, 无上限 |
| **只输出 raw** | 陀螺仪 / 加速度 / 温度原始数据, **不含任何滤波/姿态算法** |

---

## 2. 架构说明

```
┌─────────────────────────────────────────────────┐
│  imu_hub  (单个 ROS2 节点, 1 个进程)              │
│                                                 │
│  扫描定时器 (默认 1s)                             │
│    └─ glob 扫描 /dev/ttyACM* /dev/ttyUSB*        │
│         ├─ 新设备 → 打开串口 → 识别型号 → 发布话题  │
│         │           → 启动数据线程                │
│         └─ 消失设备 → 停止线程 → 销毁 publisher    │
│                                                 │
│  每个 IMU 一个数据线程:                           │
│    /A1/data_raw  /A1/tempc                      │
│    /A2/data_raw  /A2/tempc                      │
│    ... (无限)                                    │
└─────────────────────────────────────────────────┘
```

- **可执行名**: `ess_imu_driver2_node` (`ros2 run ess_imu_driver2 ess_imu_driver2_node`)
- **数据流向**: IMU → UART → 芯片内部 FIR 滤波 → raw 话题, **无软件滤波算法**
- **姿态说明**: `sensor_msgs/Imu` 消息强制包含 `orientation` 字段, raw 驱动按 **REP-145 约定**置 `orientation_covariance[0] = -1` 表示**无姿态**, 消费者应忽略该字段

---

## 3. 硬件绑定 (YAML 配置)

IMU 名称与 CH343 硬件 ID 的绑定关系写在**外部 YAML 文件**中 (不写死在代码里):

**文件**: `src/ess_imu_driver2/config/imu_bindings.yaml` (launch 自动加载, 已随包安装)

```yaml
imu_bindings:
  A1: "5C38170227"
  A2: "5C38170221"
  B1: "5C38170222"
  B2: "5C38170220"
```

- 驱动通过 sysfs 读取每个设备的 **CH343 USB 序列号**, 命中 YAML 即用对应名称;
- **未绑定设备** → 自动命名 `imu(x+1)`, x = YAML 绑定数量 (绑定 4 个时, 第 5 块叫 `imu5`);
- **修改绑定 = 编辑 YAML 重启 launch**, 无需改代码/重编译;
- 验证序列号: `ls /sys/class/tty/ttyACM*/device/../serial`

---

## 4. 快速启动

```bash
# 1. 确认设备已识别
ls -l /dev/ttyACM*

# 2. 启动 ROS2 环境
source /opt/ros/humble/setup.bash
cd ~/Epson_HotSwap_ws
colcon build
# 3. 启动驱动 (核心, 只此一步)
source install/setup.bash
ros2 launch ess_imu_driver2 imu_hub_launch.py
```

启动后终端持续打印 (约每 1 秒扫描一次):

```
[imu_hub]: IMU Hub 已启动: glob='/dev/imu_* /dev/ttyUSB* /dev/ttyACM*' 扫描周期=1.0s 波特率=460800
[imu_hub]:   ~ USB序列号 5C38170221 -> 绑定名称 A2
[imu_hub]: 新 IMU: /dev/ttyACM0 -> 名称 A2, 型号 G354PDH0, IMU序列号 00002736, USB序列号 5C38170221 | 话题 /A2/data_raw + /A2/tempc
```

> 该终端保持运行。**插拔 IMU 自动增删话题, 无需重启。**

---

## 5. 部署到新电脑 (Clone)

在另一台装有 ROS2 Humble 的电脑上:

```bash
# 1. 安装依赖 (若已装可跳过)
sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions
source /opt/ros/humble/setup.bash

# 2. Clone 主仓库
cd ~
git clone https://github.com/GZ89mid/Epson_HotSwap_ws.git
cd Epson_HotSwap_ws

# 3. 拉取核心驱动 (src/ess_imu_driver2 是嵌套的独立仓库,
#    clone 主仓库后该目录为空, 必须单独拉取)
git clone https://github.com/cubicleguy/ess_imu_driver2.git src/ess_imu_driver2

# 4. 编译 (build/ install/ log/ 为产物, 不入库)
colcon build --symlink-install
source install/setup.bash

# 5. 启动驱动
ros2 launch ess_imu_driver2 imu_hub_launch.py
```

> 私有仓库需要先配置 GitHub 访问权限 (SSH key 或 Personal Access Token)。

---

## 6. 推送到 GitHub

```bash
cd ~/Epson_HotSwap_ws

# 1. 首次需关联远端 (已关联可跳过)
git remote add origin https://github.com/GZ89mid/Epson_HotSwap_ws.git
git remote -v            # 确认远端

# 2. 查看改动
git status               # build/ install/ log/ 已被 .gitignore 排除

# 3. 暂存并提交
git add README.md
git commit -m "更新 README: 新增部署说明"

# 4. 推送 (首次用 -u 建立上游跟踪)
git push -u origin main
```

注意事项:

| 事项 | 说明 |
|------|------|
| 编译产物 | `build/ install/ log/` 已被 `.gitignore` 排除, 不要 `git add` 它们 |
| 嵌套仓库 | `src/ess_imu_driver2` 内的改动要在它自己的仓库里单独 `add/commit/push` |
| 推送失败 | 远端有更新时先 `git pull --rebase` 再 `git push` |
| 提交信息 | 写清改了什么, 方便回滚 |

---

## 7. 常用参数

```bash
ros2 launch ess_imu_driver2 imu_hub_launch.py <参数>:=<值>
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `device_glob` | `/dev/imu_* /dev/ttyUSB* /dev/ttyACM*` | 串口设备扫描通配符 (空格分隔) |
| `prefix` | `imu` | 未绑定设备命名前缀 (imu5, imu6, ...) |
| `baud_rate` | `460800` | 串口波特率 |
| `poll_interval` | `1.0` | 设备扫描周期 (秒) |
| `filter_vid` | (空) | 只接受该 USB VID, 如 `1a86` 只认 CH343 |
| `publish_temperature` | `true` | 是否发布温度话题 `/<名>/tempc` |
| `imu_dout_rate` | `9` (200Hz) | 数据频率: 4=125Hz, 8=400Hz |
| `imu_filter_sel` | `5` (TAP32) | IMU 芯片内部 FIR 滤波 (硬件滤波, 非软件算法) |

示例:

```bash
# 只认 CH343 + 400Hz
ros2 launch ess_imu_driver2 imu_hub_launch.py filter_vid:=1a86 imu_dout_rate:=8

# 自定义绑定文件
ros2 launch ess_imu_driver2 imu_hub_launch.py config:=/path/to/imu_bindings.yaml
```

---

## 8. 验证数据 (新开终端)

```bash
source /opt/ros/humble/setup.bash

ros2 topic list                          # 查看话题 (每个 IMU 两个)
ros2 topic echo /A2/data_raw --once      # 查看原始数据
ros2 topic hz /A2/data_raw               # 查看频率 (应 200Hz)
```

`/A2/data_raw` 消息示例:

```
header:
  frame_id: A2_link
orientation:              # 无姿态: 单位四元数占位
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
orientation_covariance:   # -1 表示无姿态 (REP-145 约定)
- -1.0
- 0.0
- ...
angular_velocity:         # rad/s
  x: -0.1265
  y: 0.0396
  z: 0.0080
linear_acceleration:      # m/s²
  x: ...
  y: ...
  z: ...
```

---

## 9. 话题与设备对照

| 名称 | 绑定序列号 | 数据话题 | 温度话题 | frame_id |
|------|-----------|---------|---------|----------|
| A1 | 5C38170227 | /A1/data_raw | /A1/tempc | A1_link |
| A2 | 5C38170221 | /A2/data_raw | /A2/tempc | A2_link |
| B1 | 5C38170222 | /B1/data_raw | /B1/tempc | B1_link |
| B2 | 5C38170220 | /B2/data_raw | /B2/tempc | B2_link |
| imu5+ (未绑定) | — | /imu5/data_raw | /imu5/tempc | imu5_link |

> 名称按序列号匹配 YAML, 与 ttyACM 编号顺序无关。节点只有 1 个: `/imu_hub`。

---

## 10. 热插拔行为

| 操作 | 效果 |
|------|------|
| 插入新 IMU | 1 秒内自动识别, 发布对应话题 (按序列号绑定或自动编号) |
| 拔出 IMU | 1 秒内对应话题**实时删除** (publisher 销毁) |
| 再次插入 | 重新识别, 名称不变 (绑定按序列号) |

无需重启驱动, 无需修改任何配置。

---

## 11. 物理识别板子 (可选)

```bash
python3 ~/Epson_HotSwap_ws/src/ess_imu_driver2/tools/monitor_ahrs.py
```

实时显示各板角速度/加速度/温度, **拿起哪块转动, 对应列角速度变化最大** → 该列名字即这块板。

---

## 12. 常见问题

| 现象 | 处理 |
|------|------|
| `ls /dev/ttyACM*` 没设备 | 重新插拔 USB; `sudo dmesg \| grep -i usb` |
| 某话题一直不出 | 看驱动终端日志 (会打印失败原因); 拔插对应板子 |
| 拔出后话题还在 | 等 1 个扫描周期 (默认 1s) 自动删除 |
| 想换名称/加新板 | 编辑 `config/imu_bindings.yaml` 加一行, 重启 launch |
| 名称分配错乱 | YAML 按 CH343 序列号绑定不会乱; 检查序列号: `ls /sys/class/tty/ttyACM*/device/../serial` |
| 权限不足 | `sudo usermod -aG dialout $USER` 后重新登录 |
| 需要更高频率 | `imu_dout_rate:=8` = 400Hz |
| 需要姿态数据 | 本驱动只输出 raw; 上层订阅 data_raw 自行解算 |
