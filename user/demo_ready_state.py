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

Demo code for move to ready state of the robot

Run this script by:
    python demo_xxx.py

"""

import time
import zenoh
import msgpack

import fourier_grx.user as fourier_grx


def demo_task():
    prefix = "fourier-grx"

    # 初始化 zenoh 会话
    zenoh_config = zenoh.Config()
    zenoh_session: zenoh.Session = zenoh.open(zenoh_config)

    # 构建发布者
    zenoh_publisher = zenoh_session.declare_publisher(
        key_expr=f"{prefix}/dynalink_interface/task/client",  # 目标发布者的 key 表达式
        priority=zenoh.Priority.REAL_TIME,
        congestion_control=zenoh.CongestionControl.DROP,
    )

    # 构建消息
    message = {
        "robot_task_command": fourier_grx.TaskCommand.TASK_READY_STATE,
        "flag_task_command_update": True,
    }

    print("Sending message: ", message)

    # 发布消息
    zenoh_publisher.put(msgpack.packb(message))

    # 等待 1s (确保消息被发送)
    time.sleep(1)

    # 关闭 zenoh 会话
    zenoh_session.close()


if __name__ == "__main__":
    demo_task()
