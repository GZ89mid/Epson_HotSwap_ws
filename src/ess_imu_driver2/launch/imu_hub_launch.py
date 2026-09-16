#!/usr/bin/env python3
# ============================================================
# imu_hub_launch.py — 单进程多 IMU 热插拔驱动启动文件
#
# 只启动这一个节点, 即可:
#   - 实时热插拔: 插入 IMU 自动发布话题, 拔出自动删除话题
#   - 支持绑定 ID: 从 config/imu_bindings.yaml 读取 USB 序列号 -> 名称
#   - 无限 IMU 数量
#   - 只输出 raw 数据 (/<name>/data_raw, /<name>/tempc)
#
# 用法:
#   ros2 launch ess_imu_driver2 imu_hub_launch.py
#   ros2 launch ess_imu_driver2 imu_hub_launch.py baud_rate:=921600
# ============================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_bindings(config_path):
    """读取 YAML 绑定文件: {名称: 序列号} -> 参数数组 ["A1=5C38170227", ...]"""
    if not config_path or not os.path.exists(config_path):
        return []
    try:
        import yaml
        with open(config_path) as f:
            data = yaml.safe_load(f) or {}
        binds = data.get("imu_bindings") or {}
        return [f"{name}={serial}" for name, serial in binds.items()]
    except Exception as e:
        print(f"[imu_hub_launch] !! 读取 YAML 绑定失败 {config_path}: {e}", flush=True)
        return []


def generate_launch_description():
    pkg_dir = get_package_share_directory("ess_imu_driver2")
    default_config = os.path.join(pkg_dir, "config", "imu_bindings.yaml")

    # 启动时读取一次绑定文件 (修改 YAML 后重启 launch 生效)
    binding_args = _load_bindings(default_config)

    return LaunchDescription([
        DeclareLaunchArgument(
            "device_glob",
            default_value="/dev/imu_* /dev/ttyUSB* /dev/ttyACM*",
            description="串口设备扫描通配符 (空格分隔)",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="imu",
            description="未绑定设备的命名前缀 (imu5, imu6, ...)",
        ),
        DeclareLaunchArgument(
            "baud_rate",
            default_value="460800",
            description="UART 波特率",
        ),
        DeclareLaunchArgument(
            "imu_dout_rate",
            default_value="9",
            description="IMU 数据输出率 (0~15); 9=200Hz, 4=125Hz",
        ),
        DeclareLaunchArgument(
            "imu_filter_sel",
            default_value="5",
            description="IMU FIR 滤波器 (0~19); 5=TAP32",
        ),
        DeclareLaunchArgument(
            "poll_interval",
            default_value="1.0",
            description="设备扫描周期 (秒)",
        ),
        DeclareLaunchArgument(
            "filter_vid",
            default_value="",
            description="只接受该 USB VID (如 1a86), 空=接受全部",
        ),
        DeclareLaunchArgument(
            "publish_temperature",
            default_value="true",
            description="是否发布温度话题 /<name>/tempc",
        ),
        Node(
            package="ess_imu_driver2",
            executable="ess_imu_driver2_node",
            name="imu_hub",
            output="screen",
            parameters=[{
                "device_glob": LaunchConfiguration("device_glob"),
                "prefix": LaunchConfiguration("prefix"),
                "baud_rate": LaunchConfiguration("baud_rate"),
                "imu_dout_rate": LaunchConfiguration("imu_dout_rate"),
                "imu_filter_sel": LaunchConfiguration("imu_filter_sel"),
                "poll_interval": LaunchConfiguration("poll_interval"),
                "filter_vid": LaunchConfiguration("filter_vid"),
                "publish_temperature": LaunchConfiguration("publish_temperature"),
                "bindings": binding_args,
            }],
        ),
    ])
