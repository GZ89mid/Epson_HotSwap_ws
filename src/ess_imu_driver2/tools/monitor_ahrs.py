#!/usr/bin/env python3
"""实时监控多个 IMU 的 raw 数据 — 用于物理识别哪块板子对应哪个名称

用法:
  source /opt/ros/humble/setup.bash
  python3 ~/G-Exo/ros2_ws/src/ess_imu_driver2/tools/monitor_ahrs.py
  然后拿起任意一块板子转动/晃动, 看哪一列角速度变化最大, 那列的名字就是这块板子

参数: --names 'A1 A2 B1 B2' 可自定义名称列表
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature
import math


class ImuMonitor(Node):
    def __init__(self, names):
        super().__init__("imu_monitor")
        self.data = {n: None for n in names}
        for n in names:
            self.create_subscription(Imu, f"/{n}/data_raw", lambda msg, name=n: self.cb(name, msg), 10)
            self.create_subscription(Temperature, f"/{n}/tempc", lambda msg, name=n: self.cb_temp(name, msg), 10)

    def cb(self, name, msg):
        g = msg.angular_velocity  # rad/s
        a = msg.linear_acceleration  # m/s^2
        self.data[name] = (g.x, g.y, g.z, a.x, a.y, a.z)

    def cb_temp(self, name, msg):
        d = self.data[name]
        if d is None:
            d = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self.data[name] = d
        self.data[name] = (d[0], d[1], d[2], d[3], d[4], d[5], msg.temperature)

    def render(self):
        sys.stdout.write("\033[H\033[J")  # 清屏
        print("=" * 100)
        print("  IMU 板识别器 (raw) — 拿起某块板子转动, 对应列角速度变化最大即为该板")
        print("  角速度单位 deg/s | 加速度单位 m/s^2 | 温度单位 degC")
        print("=" * 100)
        print(f"{'名称':<6}{'gx':>9}{'gy':>9}{'gz':>9}  {'ax':>8}{'ay':>8}{'az':>8}  {'temp':>7}  状态")
        print("-" * 100)
        for n in self.data:
            d = self.data[n]
            if d is None:
                print(f"{n:<6}{'--':>9}{'--':>9}{'--':>9}  {'--':>8}{'--':>8}{'--':>8}  {'--':>7}  等待数据...")
                continue
            has_temp = len(d) > 6
            gx, gy, gz, ax, ay, az = d[0], d[1], d[2], d[3], d[4], d[5]
            gx_d, gy_d, gz_d = math.degrees(gx), math.degrees(gy), math.degrees(gz)
            acc_norm = math.sqrt(ax * ax + ay * ay + az * az) / 9.80665
            status = "OK" if 0.5 < acc_norm < 1.5 else "?!"
            temp_s = f"{d[6]:>7.2f}" if has_temp else f"{'--':>7}"
            print(f"{n:<6}{gx_d:>9.1f}{gy_d:>9.1f}{gz_d:>9.1f}  {ax:>8.2f}{ay:>8.2f}{az:>8.2f}  {temp_s}  {status} (|a|={acc_norm:.2f}g)")
        print("-" * 100)
        print("  Ctrl+C 退出 | 静止时角速度应接近 0, 加速度模应接近 1g")
        sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser(description="Monitor IMU attitudes in real-time")
    ap.add_argument("--names", default="A1 A2 B1 B2", help="Space-separated IMU names")
    args = ap.parse_args()
    names = args.names.split()

    rclpy.init()
    mon = ImuMonitor(names)
    try:
        while rclpy.ok():
            rclpy.spin_once(mon, timeout_sec=0.1)
            mon.render()
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    mon.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
