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

"""

import time

import fourier_grx.sdk.grmini1.developer as fourier_grx

control_system = fourier_grx.ControlSystem()


def main():
    # 切换为开发者模式
    control_system.developer_mode(servo_on=False)

    # 打印版本信息
    print(control_system.get_info())

    # 算法任务
    algorithm()


def algorithm():
    control_system.robot_control_set_task_command(
        task_command=fourier_grx.TaskCommand.TASK_SERVO_ON
    )

    # 等待算法任务执行完成
    time.sleep(1)


if __name__ == "__main__":
    main()
