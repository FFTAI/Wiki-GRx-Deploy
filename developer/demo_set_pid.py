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

import numpy

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
    # 控制参数如不需修改，则只需要发送一次即可
    joint_target_control_mode = numpy.array([
        # left leg
        fourier_grx.JointControlMode.PD, fourier_grx.JointControlMode.PD, fourier_grx.JointControlMode.PD,
        fourier_grx.JointControlMode.PD, fourier_grx.JointControlMode.PD, fourier_grx.JointControlMode.PD,
        # right leg
        fourier_grx.JointControlMode.PD, fourier_grx.JointControlMode.PD, fourier_grx.JointControlMode.PD,
        fourier_grx.JointControlMode.PD, fourier_grx.JointControlMode.PD, fourier_grx.JointControlMode.PD,
        # waist
        fourier_grx.JointControlMode.PD,
        # left arm
        fourier_grx.JointControlMode.PD, fourier_grx.JointControlMode.PD, fourier_grx.JointControlMode.PD,
        fourier_grx.JointControlMode.PD, fourier_grx.JointControlMode.PD,
        # right arm
        fourier_grx.JointControlMode.PD, fourier_grx.JointControlMode.PD, fourier_grx.JointControlMode.PD,
        fourier_grx.JointControlMode.PD, fourier_grx.JointControlMode.PD,
    ])
    joint_target_kp = numpy.array([
        # left leg
        180.0, 120.0, 90.0, 120.0, 45.0, 45.0,
        # right leg
        180.0, 120.0, 90.0, 120.0, 45.0, 45.0,
        # waist
        90.0,
        # left arm
        90.0, 45.0, 45.0, 45.0, 45.0,
        # right arm
        90.0, 45.0, 45.0, 45.0, 45.0,
    ])
    joint_target_kd = numpy.array([
        # left leg
        10.0, 10.0, 8.0, 8.0, 2.5, 2.5,
        # right leg
        10.0, 10.0, 8.0, 8.0, 2.5, 2.5,
        # waist
        8.0,
        # left arm
        8.0, 2.5, 2.5, 2.5, 2.5,
        # right arm
        8.0, 2.5, 2.5, 2.5, 2.5,
    ])

    """
    control:
    - control_mode
    - pd_control_kp
    - pd_control_kd
    """
    control_dict = {
        "control_mode": joint_target_control_mode,
        "pd_control_kp": joint_target_kp,
        "pd_control_kd": joint_target_kd,
    }

    # 输出控制
    control_system.robot_control_loop_set_control(control_dict=control_dict)


if __name__ == "__main__":
    main()
