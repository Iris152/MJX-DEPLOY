#!/usr/bin/env python3

"""
本文件初始作者为 YixuanQiu。
当前版本已在原始内容基础上做过修改。
"""

import argparse
import os
import select
import signal
import sys
import termios
import threading
import time
import tty
from typing import Dict

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# 控制限幅常量
MAX_LINEAR_SPEED = 1.5  # 纵向速度上限（m/s）
MAX_LATERAL_SPEED = 0.8  # 侧向速度上限（m/s）
MAX_ANGULAR_SPEED = 1.5  # 角速度上限（rad/s）
SPEED_INCREMENT = 0.1

# 键盘按键映射
KEY_UP = "\x1b[A"  # 前进，增大 vx
KEY_DOWN = "\x1b[B"  # 后退，减小 vx
KEY_RIGHT = "\x1b[C"  # 向右横移，减小 vy
KEY_LEFT = "\x1b[D"  # 向左横移，增大 vy
KEY_A = "a"  # 左转，增大 wz
KEY_D = "d"  # 右转，减小 wz
KEY_Q = "q"  # 退出
KEY_SPACE = " "  # 停止，清零所有速度指令
KEY_CTRL_C = "\x03"


# 读取单个按键，配合终端原始模式实现非阻塞式控制体验。
def getch():
    """从标准输入读取一个字符，并且不回显到屏幕。"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        # 方向键会发送多个字符，需要一次性拼完整。
        if ch == "\x1b":
            ch = ch + sys.stdin.read(2)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class KeyboardController(Node):
    def __init__(self, ctrl_mode: str):
        super().__init__("keyboard_controller")

        # 当前速度指令状态
        self.forward_speed = 0.0  # vx，纵向速度（m/s）
        self.lateral_speed = 0.0  # vy，侧向速度（m/s）
        self.angular_speed = 0.0  # wz，偏航角速度（rad/s）

        # 创建 ROS2 速度指令发布器，消息中携带 vx、vy、wz。
        if ctrl_mode == "diffloco":
            self.command_publisher = self.create_publisher(
                PointStamped, "/velocity_command", 10
            )
        else:
            raise ValueError("Invalid control mode. Use 'diffloco'.")

        # 创建速度指令消息，point.x/y/z 分别对应 vx、vy、wz。
        self.command_msg = PointStamped()

        # 初始化速度指令为零。
        self.command_msg.point.x = 0.0  # 纵向速度 vx
        self.command_msg.point.y = 0.0  # 侧向速度 vy
        self.command_msg.point.z = 0.0  # 偏航角速度 wz

        # 发布线程状态
        self.running = False
        self.publish_thread = None
        self._cleaned_up = False

        # 注册信号处理，保证终端尺寸变化和退出时状态可恢复。
        signal.signal(signal.SIGWINCH, self.handle_resize)
        signal.signal(signal.SIGINT, self.handle_interrupt)
        signal.signal(signal.SIGTERM, self.handle_interrupt)

    def handle_resize(self, *args):
        """处理终端尺寸变化事件。"""
        # 重新绘制控制界面。
        self.clear_screen()
        self.draw_control_state()

    def handle_interrupt(self, *args):
        """处理退出信号。"""
        self.running = False

    def clear_screen(self):
        """清空终端屏幕。"""
        os.system("clear")

    def draw_control_state(self):
        """在终端绘制当前控制状态。"""
        self.clear_screen()

        # 组装终端提示和当前速度状态。
        instructions = [
            "Go2 Locomotion Control - Terminal Version",
            "",
            "=== VELOCITY CONTROLS ===",
            "↑/↓: Increase/Decrease Forward Speed (vx)",
            "←/→: Increase/Decrease Strafe Speed (vy, Left/Right)",
            "a/d: Increase/Decrease Turn Speed (wz, Left/Right)",
            "",
            "",
            "SPACE: Clear all commands (STOP)",
            "q: Quit",
            "",
            "=== CURRENT MOVEMENT ===",
        ]

        # 根据速度正负显示当前运动方向。
        movement_status = []
        if self.forward_speed > 0:
            movement_status.append(f"FORWARD ({self.forward_speed:.2f} m/s)")
        elif self.forward_speed < 0:
            movement_status.append(f"BACKWARD ({abs(self.forward_speed):.2f} m/s)")

        if self.lateral_speed > 0:
            movement_status.append(f"STRAFING LEFT ({self.lateral_speed:.2f} m/s)")
        elif self.lateral_speed < 0:
            movement_status.append(
                f"STRAFING RIGHT ({abs(self.lateral_speed):.2f} m/s)"
            )

        if self.angular_speed > 0:
            movement_status.append(f"TURNING LEFT ({self.angular_speed:.2f} rad/s)")
        elif self.angular_speed < 0:
            movement_status.append(
                f"TURNING RIGHT ({abs(self.angular_speed):.2f} rad/s)"
            )

        if not movement_status:
            movement_status.append("STOPPED")

        instructions.extend(movement_status)

        instructions.extend(
            [
                "",
                "=== CURRENT STATE ===",
                f"Forward Speed (vx): {self.forward_speed:.2f} m/s",
                f"Lateral Speed (vy): {self.lateral_speed:.2f} m/s",
                f"Angular Speed (wz): {self.angular_speed:.2f} rad/s",
            ]
        )

        # 输出完整终端界面。
        print("\n".join(instructions))

    def clear_all_commands(self):
        """将所有速度指令清零。"""
        self.forward_speed = 0.0
        self.lateral_speed = 0.0
        self.angular_speed = 0.0

    def update_speed_from_key(self, key):
        """根据按键更新速度，并做限幅处理。"""
        # 纵向和侧向速度控制。
        if key == KEY_UP:  # 前进
            self.forward_speed = min(
                self.forward_speed + SPEED_INCREMENT, MAX_LINEAR_SPEED
            )
        elif key == KEY_DOWN:  # 后退
            self.forward_speed = max(
                self.forward_speed - SPEED_INCREMENT, -MAX_LINEAR_SPEED
            )
        elif key == KEY_LEFT:  # 向左横移
            self.lateral_speed = min(
                self.lateral_speed + SPEED_INCREMENT, MAX_LATERAL_SPEED
            )
        elif key == KEY_RIGHT:  # 向右横移
            self.lateral_speed = max(
                self.lateral_speed - SPEED_INCREMENT, -MAX_LATERAL_SPEED
            )
        # 偏航角速度控制。
        elif key == KEY_A:  # 左转，wz 为正
            self.angular_speed = min(
                self.angular_speed + SPEED_INCREMENT, MAX_ANGULAR_SPEED
            )
        elif key == KEY_D:  # 右转，wz 为负
            self.angular_speed = max(
                self.angular_speed - SPEED_INCREMENT, -MAX_ANGULAR_SPEED
            )
        elif key == KEY_SPACE:  # 清零所有指令
            self.clear_all_commands()

    def publish_command(self):
        """向运动控制节点发布速度指令。"""
        # 将当前速度写入 ROS2 消息。
        self.command_msg.header.stamp = self.get_clock().now().to_msg()
        self.command_msg.point.x = float(self.forward_speed)  # vx
        self.command_msg.point.y = float(self.lateral_speed)  # vy
        self.command_msg.point.z = float(self.angular_speed)  # wz

        # 发布速度指令。
        self.command_publisher.publish(self.command_msg)

    def publisher_thread_function(self):
        """发布线程的循环函数。"""
        while self.running and rclpy.ok():
            self.publish_command()
            time.sleep(0.02)  # 50 赫兹

    def cleanup(self):
        """退出前清理终端并发送零速度。"""
        if self._cleaned_up:
            return
        self._cleaned_up = True

        # 退出前发送零速度指令。
        self.forward_speed = 0.0
        self.lateral_speed = 0.0
        self.angular_speed = 0.0
        if rclpy.ok():
            self.publish_command()

        # 恢复终端状态。
        os.system("stty sane")
        print("\033[?25h")  # 显示光标

    def run(self):
        """主控制循环。"""
        self.running = True

        try:
            # 隐藏光标，减少界面闪烁。
            print("\033[?25l")

            # 启动速度指令发布线程。
            self.publish_thread = threading.Thread(
                target=self.publisher_thread_function
            )
            self.publish_thread.daemon = True
            self.publish_thread.start()

            # 绘制初始界面。
            self.draw_control_state()

            # 主输入循环，每次处理一个按键。
            while self.running and rclpy.ok():
                key = getch()

                # 处理退出按键。
                if key == KEY_Q or key == KEY_CTRL_C:  # q 或 Ctrl+C
                    self.running = False
                    break

                # 根据按键更新速度指令。
                if key in [
                    KEY_UP,
                    KEY_DOWN,
                    KEY_LEFT,
                    KEY_RIGHT,
                    KEY_A,
                    KEY_D,
                    KEY_SPACE,
                ]:
                    self.update_speed_from_key(key)
                    self.draw_control_state()

        except KeyboardInterrupt:
            self.running = False
            print("Keyboard interrupt received, shutting down")
        finally:
            self.cleanup()

            if self.publish_thread:
                self.publish_thread.join(timeout=1.0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Go2 Locomotion Control")
    parser.add_argument(
        "--control",
        type=str,
        choices=["diffloco"],
        default="diffloco",
        help="Control mode: 'diffloco' for differential locomotion commands",
    )
    args = parser.parse_args()

    # 初始化 ROS2。
    rclpy.init()

    controller = KeyboardController(ctrl_mode=args.control)

    def spin_controller():
        try:
            rclpy.spin(controller)
        except ExternalShutdownException:
            pass
        except Exception:
            if rclpy.ok():
                raise

    ros_thread = threading.Thread(target=spin_controller)
    ros_thread.start()

    try:
        controller.run()
    except KeyboardInterrupt:
        print("Keyboard interrupt received, shutting down")
    finally:
        controller.running = False
        controller.cleanup()
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join(timeout=1.0)
        controller.destroy_node()
        print("Exiting controller")
