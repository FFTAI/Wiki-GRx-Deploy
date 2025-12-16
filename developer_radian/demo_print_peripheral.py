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

Demo code for Fourier robots

Run this script by:
    python demo_xxx.py --config=config_xxx.yaml
    - config_xxx.yaml is the configuration file for the Fourier robots

本示例需要在配置文件中启用外设（手柄或键盘）
"""

from ischedule import run_loop, schedule

import fourier_grx.sdk.developer as fourier_grx

control_system = fourier_grx.ControlSystem()


def demo_task():
    # 设置机器人算法频率
    control_frequency = 1  # 机器人控制频率, 1Hz
    control_period = 1.0 / control_frequency  # 机器人控制周期

    # 切换为开发者模式
    control_system.developer_mode(servo_on=False)

    # 打印版本信息
    print(control_system.get_info())

    # 设置定时任务
    schedule(schedule_task, interval=control_period)

    run_loop()


def schedule_task():
    joystick = fourier_grx.joystick
    keyboard = fourier_grx.keyboard

    if joystick:
        joystick_state = joystick.get_state()
        print(f"Joystick State: {joystick_state}")

    if keyboard:
        keyboard_state = keyboard.get_state()
        print(f"Keyboard State: {keyboard_state}")


if __name__ == "__main__":
    demo_task()
