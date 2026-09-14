#!/usr/bin/env python3

"""
本文件初始作者为 YixuanQiu。
当前版本已在原始内容基础上做过修改。
"""

import argparse
import struct
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber

# 当前部署目标是 Go2，因此使用 unitree_go 的低层状态消息类型。
# 使用 G1、H1-2 时需要改成对应机型的消息类型。
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

# 遥控器速度映射上限。
MAX_LINEAR_SPEED = 2.5  # 纵向速度上限（m/s）
MAX_LATERAL_SPEED = 1.5  # 侧向速度上限（m/s）
MAX_ANGULAR_SPEED = 3.0  # 角速度上限（rad/s）


class unitreeRemoteController:
    def __init__(self):
        # 摇杆状态。
        self.Lx = 0.0  # 左摇杆 X 轴，控制横移
        self.Ly = 0.0  # 左摇杆 Y 轴，控制前进/后退
        self.Rx = 0.0  # 右摇杆 X 轴，控制转向
        self.Ry = 0.0  # 右摇杆 Y 轴，当前未使用

        # 按键状态。
        self.L1 = 0
        self.L2 = 0
        self.R1 = 0
        self.R2 = 0
        self.A = 0
        self.B = 0
        self.X = 0
        self.Y = 0
        self.Up = 0
        self.Down = 0
        self.Left = 0
        self.Right = 0
        self.Select = 0
        self.F1 = 0
        self.F3 = 0
        self.Start = 0

    def parse_botton(self, data1, data2):
        self.R1 = (data1 >> 0) & 1
        self.L1 = (data1 >> 1) & 1
        self.Start = (data1 >> 2) & 1
        self.Select = (data1 >> 3) & 1
        self.R2 = (data1 >> 4) & 1
        self.L2 = (data1 >> 5) & 1
        self.F1 = (data1 >> 6) & 1
        self.F3 = (data1 >> 7) & 1
        self.A = (data2 >> 0) & 1
        self.B = (data2 >> 1) & 1
        self.X = (data2 >> 2) & 1
        self.Y = (data2 >> 3) & 1
        self.Up = (data2 >> 4) & 1
        self.Right = (data2 >> 5) & 1
        self.Down = (data2 >> 6) & 1
        self.Left = (data2 >> 7) & 1

    def parse_key(self, data):
        lx_offset = 4
        self.Lx = struct.unpack("<f", data[lx_offset : lx_offset + 4])[0]
        rx_offset = 8
        self.Rx = struct.unpack("<f", data[rx_offset : rx_offset + 4])[0]
        ry_offset = 12
        self.Ry = struct.unpack("<f", data[ry_offset : ry_offset + 4])[0]
        ly_offset = 20
        self.Ly = struct.unpack("<f", data[ly_offset : ly_offset + 4])[0]

    def parse(self, remoteData):
        self.parse_key(remoteData)
        self.parse_botton(remoteData[2], remoteData[3])


class WirelessController(Node):
    def __init__(self, ctrl_mode: str, command_topic: str):
        super().__init__("wireless_controller")

        self.low_state = None
        self.remoteController = unitreeRemoteController()

        # 根据控制模式创建 ROS2 发布器，DiffLoco 默认监听 /velocity_command。
        if ctrl_mode == "diffloco":
            topic = command_topic
        elif ctrl_mode == "loco":
            topic = "/high_level_command"
        elif ctrl_mode == "filter":
            topic = "/navigation_vel_cmd"
        else:
            raise ValueError("Invalid control mode. Use 'diffloco', 'loco', or 'filter'.")
        self.command_publisher = self.create_publisher(PointStamped, topic, 10)
        self.command_topic = topic

        # 创建速度指令消息，point.x/y/z 分别对应 vx、vy、wz。
        self.command_msg = PointStamped()

        # 定时发布速度指令。
        self.timer = self.create_timer(0.02, self.publish_command)  # 50 赫兹

        # 摇杆死区，抑制轻微漂移。
        self.deadzone = 0.1

        self.smoothed_forward = 0.0
        self.smoothed_lateral = 0.0
        self.smoothed_angular = 0.0
        self.smooth_factor = 0.5

    def Init(self):
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)

    def apply_deadzone(self, value, deadzone=None):
        """对摇杆输入应用死区。"""
        if deadzone is None:
            deadzone = self.deadzone
        if abs(value) < deadzone:
            return 0.0
        return value

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg
        wireless_remote_data = self.low_state.wireless_remote
        self.remoteController.parse(wireless_remote_data)

    def publish_command(self):
        """根据遥控器输入发布速度指令。"""
        if self.low_state is None:
            return

        # 对摇杆输入应用死区。
        # 左摇杆控制 vx 和 vy。
        forward_input = self.apply_deadzone(self.remoteController.Ly)  # 正值表示前进
        lateral_input = self.apply_deadzone(-self.remoteController.Lx)  # 左摇杆 X 轴控制横移
        
        # 右摇杆 X 轴控制偏航角速度 wz。
        angular_input = self.apply_deadzone(-self.remoteController.Rx)

        # 松杆时立即归零，推杆时做平滑加速，避免速度突变。
        if forward_input == 0.0:
            self.smoothed_forward = 0.0
        else:
            self.smoothed_forward = (
                self.smoothed_forward * (1 - self.smooth_factor) + forward_input * self.smooth_factor
            )

        if lateral_input == 0.0:
            self.smoothed_lateral = 0.0
        else:
            self.smoothed_lateral = (
                self.smoothed_lateral * (1 - self.smooth_factor) + lateral_input * self.smooth_factor
            )

        if angular_input == 0.0:
            self.smoothed_angular = 0.0
        else:
            self.smoothed_angular = (
                self.smoothed_angular * (1 - self.smooth_factor) + angular_input * self.smooth_factor
            )
        


        # 将摇杆输入映射为 vx、vy、wz 速度指令。
        self.command_msg.header.stamp = self.get_clock().now().to_msg()
        self.command_msg.point.x = float(self.smoothed_forward * MAX_LINEAR_SPEED)  # vx
        self.command_msg.point.y = float(self.smoothed_lateral * MAX_LATERAL_SPEED)  # vy
        self.command_msg.point.z = float(self.smoothed_angular * MAX_ANGULAR_SPEED)  # wz
        
        
        self.command_publisher.publish(self.command_msg)

        print(
            f"Velocity - vx: {self.command_msg.point.x:.2f}, vy: {self.command_msg.point.y:.2f}, wz: {self.command_msg.point.z:.2f}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Go2 Wireless Controller")
    parser.add_argument("--net", type=str, default="lo", help="Network interface for wireless communication")
    parser.add_argument(
        "--control",
        type=str,
        choices=["diffloco"],
        default="diffloco",
        help="Control mode: 'diffloco' publishes vx/vy/wz to /velocity_command",
    )
    parser.add_argument(
        "--topic",
        type=str,
        default="/velocity_command",
        help="ROS 2 geometry_msgs/PointStamped topic for DiffLoco vx, vy, wz commands",
    )
    args = parser.parse_args()

    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    print("Controller mapping:")
    print("- Left stick: Forward/Backward (Y-axis) and Strafe Left/Right (X-axis)")
    print("- Right stick: Turn Left/Right - wz (X-axis)")
    print(f"- ROS command topic: {args.topic if args.control == 'diffloco' else args.control}")

    # 初始化 Unitree SDK。
    print(f"Using network interface: {args.net}")
    ChannelFactoryInitialize(0, args.net)

    # 初始化 ROS2。
    rclpy.init()

    # 创建遥控器桥接节点。
    controller = WirelessController(ctrl_mode=args.control, command_topic=args.topic)
    controller.Init()

    try:
        # 运行 ROS2 节点。
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("Keyboard interrupt received, shutting down")
    finally:
        # 退出前发送零速度指令。
        controller.command_msg.header.stamp = controller.get_clock().now().to_msg()
        controller.command_msg.point.x = 0.0
        controller.command_msg.point.y = 0.0
        controller.command_msg.point.z = 0.0
        controller.command_publisher.publish(controller.command_msg)

        rclpy.shutdown()
        print("Wireless controller shutdown complete")
