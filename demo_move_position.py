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
"""

import sys
import time
import numpy

from robot_rcs_gr.control_system.fi_control_system_gr import ControlSystemGR as ControlSystem


def main(argv):
    # TODO: upgrade to 1000Hz
    """
    control frequency
    request : < 500Hz
    """
    target_control_frequency = 50  # 机器人控制频率, 50Hz
    target_control_period_in_s = 1.0 / target_control_frequency  # 机器人控制周期

    # dev mode
    ControlSystem().developer_mode(servo_on=True)

    # prepare dict
    state_dict = {}
    control_dict = {}

    # control loop
    while True:
        # control loop start time
        time_start_of_robot_control_loop_in_s = time.time()

        # update state
        """
        state:
        - imu:
          - quat
          - euler angle (rpy) [deg]
          - angular velocity [deg/s]
          - linear acceleration [m/s^2]
        - joint (in urdf):
          - position [deg]
          - velocity [deg/s]
          - torque [Nm]
        - base:
          - position xyz [m]
          - linear velocity xyz [m/s]
        """
        state_dict = ControlSystem().robot_control_loop_get_state()
        # print("state_dict = \n", state_dict)

        # parse state
        imu_quat = state_dict["imu_quat"]
        imu_euler_angle = state_dict["imu_euler_angle"]
        imu_angular_velocity = state_dict["imu_angular_velocity"]
        imu_acceleration = state_dict["imu_acceleration"]
        joint_position = state_dict["joint_position"]
        joint_velocity = state_dict["joint_velocity"]
        joint_kinetic = state_dict["joint_kinetic"]
        base_xyz = state_dict["base_estimate_xyz"]
        base_vel_xyz = state_dict["base_estimate_xyz_vel"]

        joint_measured_position = joint_position

        # algorithm (user customized...)
        joint_target_position, finish_flag = algorithm_move_position(joint_measured_position)

        if finish_flag is True:
            print("move default position movement finish!")
            break

        """
        control:
        - control_mode
        - position
        - kp
        - kd
        """
        control_dict.update({
            # control mode:
            # - 4: position control
            # - 5: PD control
            #
            # kp, kd:
            # - in position control mode: kp is position kp, kd is velocity kp
            # - in PD control mode: kp is position kp, kd is velocity kd
            "control_mode": [
                # left leg
                4, 4, 4, 4, 4, 4,
                # right leg
                4, 4, 4, 4, 4, 4,
                # waist
                4, 4, 4,
                # head
                4, 4, 4,
                # left arm
                4, 4, 4, 4, 4, 4, 4,
                # right arm
                4, 4, 4, 4, 4, 4, 4,
            ],
            "kp": [
                # left leg
                0.583, 0.284, 0.583, 0.583, 0.283, 0.283,
                # right leg
                0.583, 0.284, 0.583, 0.583, 0.283, 0.283,
                # waist
                0.25, 0.25, 0.25,
                # head
                0.005, 0.005, 0.005,
                # left arm
                0.2, 0.2, 0.2, 0.2, 0.2, 0.005, 0.005,
                # right arm
                0.2, 0.2, 0.2, 0.2, 0.2, 0.005, 0.005,
            ],
            "kd": [
                # left leg
                0.017, 0.013, 0.273, 0.273, 0.005, 0.005,
                # right leg
                0.017, 0.013, 0.273, 0.273, 0.005, 0.005,
                # waist
                0.14, 0.14, 0.14,
                # head
                0.005, 0.005, 0.005,
                # left arm
                0.02, 0.02, 0.02, 0.02, 0.02, 0.005, 0.005,
                # right arm
                0.02, 0.02, 0.02, 0.02, 0.02, 0.005, 0.005,
            ],
            # position (in urdf):
            # - unit: deg
            "position": joint_target_position
        })

        # print(numpy.round(joint_target_position, 1))

        # output control
        ControlSystem().robot_control_loop_set_control(control_dict)

        # control loop wait time
        time_end_of_robot_control_loop_in_s = time.time()
        time_of_robot_control_loop_in_s = time_end_of_robot_control_loop_in_s \
                                          - time_start_of_robot_control_loop_in_s

        # wait for next control period
        time_to_sleep_in_s = target_control_period_in_s - time_of_robot_control_loop_in_s
        if time_to_sleep_in_s >= 0:
            pass
        else:
            time_to_sleep_in_s = 0

        time.sleep(time_to_sleep_in_s)


move_count = 0
move_period = 100
joint_start_position = None


def algorithm_move_position(joint_measured_position) -> (list, int):
    global move_count, joint_start_position

    if joint_start_position is None:
        joint_start_position = numpy.array(joint_measured_position)
        print("joint_start_position = \n", numpy.round(joint_start_position, 1))

    joint_end_position = numpy.array([
        0.0, 0.0, -0.2618, 0.5236, -0.2618, 0.0,  # left leg (6)
        0.0, 0.0, -0.2618, 0.5236, -0.2618, 0.0,  # right leg (6)
        0.0, 0.0, 0.0,  # waist (3)
        0.0, 0.0, 0.0,  # head (3)
        0.0, 0.2, 0.0, -0.3, 0.0, 0.0, 0.0,  # left arm (7)
        0.0, -0.2, 0.0, -0.3, 0.0, 0.0, 0.0,  # right arm (7)
    ]) / numpy.pi * 180

    # update move ratio
    move_ratio = min(move_count / move_period, 1)

    # update target position
    joint_target_position = joint_start_position + \
                            (joint_end_position - joint_start_position) * move_ratio

    # update count
    move_count += 1

    # print info
    print("move_ratio = ", numpy.round(move_ratio * 100, 1), "%")

    if move_ratio < 1:
        finish_flag = False
    else:
        finish_flag = True

    return joint_target_position, finish_flag


if __name__ == "__main__":
    main(sys.argv)
