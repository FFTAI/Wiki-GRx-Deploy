"""
Copyright (C) [2024] [Fourier Intelligence Ltd.]

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA

--------------------------------------------------

Demo code for rl walk of the robot

Required packages:
    - zenoh
    - msgpack
    - pygame
    - fourier_grx

Run this script by:
    python demo_xxx.py

这个 demo 要求连接摇杆, 用摇杆控制机器人走路速度

"""

import time
import zenoh
import msgpack
import pygame
import threading

import fourier_grx.sdk.user as fourier_grx

joystick = None
axis_left = (0.0, 0.0)
axis_right = (0.0, 0.0)


def demo_task():
    global joystick, axis_left, axis_right

    prefix = "fourier-grx"

    # 初始化 zenoh 会话
    zenoh_config = zenoh.Config()
    zenoh_session: zenoh.Session = zenoh.open(zenoh_config)

    # 构建发布者
    zenoh_task_publisher = zenoh_session.declare_publisher(
        key_expr=f"{prefix}/dynalink_interface/task/client",  # 目标发布者的 key 表达式
        priority=zenoh.Priority.REAL_TIME,
        congestion_control=zenoh.CongestionControl.DROP,
    )
    zenoh_grx_publisher = zenoh_session.declare_publisher(
        key_expr=f"{prefix}/dynalink_interface/grx/client",  # 目标发布者的 key 表达式
        priority=zenoh.Priority.REAL_TIME,
        congestion_control=zenoh.CongestionControl.DROP,
    )

    # 构建消息
    message = {
        "robot_task_command": fourier_grx.TaskCommand.TASK_RL_WALK,
        "flag_task_command_update": True,
    }

    print("Sending message: ", message)

    # 发布消息
    zenoh_task_publisher.put(msgpack.packb(message))

    # 等待 1s (确保消息被发送)
    time.sleep(1)

    # 创建子线程, 用于监听摇杆输入
    pygame.init()
    pygame.joystick.init()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    thread_joystick_listener = threading.Thread(target=joystick_listener)
    thread_joystick_listener.start()

    # 等待 1s (确保摇杆监听线程启动)
    time.sleep(1)

    # 用摇杆控制机器人走路速度
    try:
        while True:
            # 构建消息
            message = {
                "virtual_joystick_axis_left": axis_left,
                "virtual_joystick_axis_right": axis_right,
            }

            # 发布消息
            zenoh_grx_publisher.put(msgpack.packb(message))

            # 等待 0.02s, 以减少 CPU 占用
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        pass

    # 关闭 zenoh 会话
    zenoh_session.close()

    # 关闭摇杆
    pygame.quit()


def joystick_listener():
    global joystick, axis_left, axis_right

    while True:
        pygame.event.get()

        # 获取摇杆输入
        axis_left = joystick.get_axis(0), joystick.get_axis(1)
        axis_right = joystick.get_axis(3), 0

        # 等待 0.02s, 以减少 CPU 占用
        time.sleep(0.02)


if __name__ == "__main__":
    demo_task()
