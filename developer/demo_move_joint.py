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
from ischedule import run_loop, schedule

import fourier_grx.sdk.developer as fourier_grx

control_system = fourier_grx.ControlSystem()

move_count = 0
move_period = 100
joint_start_position = None


def main():
    # 设置机器人算法频率
    target_control_frequency = 50  # 机器人控制频率, 50Hz
    target_control_period_in_s = 1.0 / target_control_frequency  # 机器人控制周期

    # 切换为开发者模式，设置机器人数据更新频率
    control_system.developer_mode(servo_on=True, control_frequency=100)

    # 打印版本信息
    print(control_system.get_info())

    # 设置定时任务
    schedule(algorithm, interval=target_control_period_in_s)

    run_loop()


def algorithm():
    global move_count, move_period, joint_start_position

    # update state
    """
    state:
    - joint (in urdf):
      - position [rad]
      - velocity [rad/s]
      - torque [Nm]
    """
    state_dict = control_system.robot_control_loop_get_state()

    # --------------------------------------------------

    robot_number_of_joint = 2 + 2 + 2 + 2

    # parse state
    joint_position = state_dict.get("joint_position", [0] * robot_number_of_joint)
    joint_velocity = state_dict.get("joint_velocity", [0] * robot_number_of_joint)
    joint_effort = state_dict.get("joint_effort", [0] * robot_number_of_joint)

    joint_measured_position = joint_position

    # algorithm (user customized...)
    if joint_start_position is None:
        joint_start_position = numpy.array(joint_measured_position)
        print("joint_start_position = \n", numpy.round(joint_start_position, 1))

    joint_final_position = \
        numpy.array([
            # left leg (rotary joint)
            -0.2, 0.2,
            # right leg (rotary joint)
            -0.2, 0.2,
            # left leg (prismatic joint)
            0.0, 0.0,
            # right leg (prismatic joint)
            0.0, 0.0,
        ])  # [rad or m]

    # update move ratio
    move_ratio = min(move_count / move_period, 1)

    # update target position
    joint_target_position = joint_start_position \
                            + (joint_final_position - joint_start_position) * move_ratio

    # update count
    move_count += 1

    # print info
    print("move_ratio = ", numpy.round(move_ratio * 100, 1), "%")

    if move_ratio < 1:
        finish_flag = False
    else:
        finish_flag = True

    if finish_flag is True:
        print("move default position movement finish!")
        exit(0)

    # --------------------------------------------------

    # 控制参数如不需修改，则只需要发送一次即可
    joint_target_control_mode = numpy.array([
        # left leg (rotary joint)
        fourier_grx.JointControlMode.PD, fourier_grx.JointControlMode.PD,
        # right leg (rotary joint)
        fourier_grx.JointControlMode.PD, fourier_grx.JointControlMode.PD,
        # left leg (prismatic joint)
        fourier_grx.JointControlMode.NONE, fourier_grx.JointControlMode.NONE,
        # right leg (prismatic joint)
        fourier_grx.JointControlMode.NONE, fourier_grx.JointControlMode.NONE,
    ])
    joint_target_kp = numpy.array([
        # left leg (rotary joint)
        200.0, 200.0,
        # right leg (rotary joint)
        200.0, 200.0,
        # left leg (prismatic joint)
        10.0, 10.0,
        # right leg (prismatic joint)
        10.0, 10.0,
    ])
    joint_target_kd = numpy.array([
        # left leg (rotary joint)
        20.0, 20.0,
        # right leg (rotary joint)
        20.0, 20.0,
        # left leg (prismatic joint)
        1.0, 1.0,
        # right leg (prismatic joint)
        1.0, 1.0,
    ])

    # --------------------------------------------------

    """
    control:
    - control_mode
    - pd_control_kp
    - pd_control_kd
    - position [rad]
    """
    control_dict = {
        "control_mode": joint_target_control_mode,
        "pd_control_kp": joint_target_kp,
        "pd_control_kd": joint_target_kd,
        "position": joint_target_position,
    }

    # output control
    control_system.robot_control_loop_set_control(control_dict=control_dict)


if __name__ == "__main__":
    main()
