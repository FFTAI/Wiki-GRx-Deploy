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

from robot_rcs_gr.control_system.fi_control_system_gr import ControlSystemGR as ControlSystem


def main(argv):
    # TODO: upgrade to 1000Hz
    """
    control frequency
    request : < 500Hz
    """
    target_control_frequency = 50  # 机器人控制频率, 50Hz
    target_control_period_in_s = 1.0 / target_control_frequency  # 机器人控制周期

    # developer mode
    ControlSystem().developer_mode()

    # print version info
    info_dict = ControlSystem().get_info()
    print(info_dict)

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
          - quat (x, y, z, w)
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

        # algorithm (user customized...)
        algorithm_user_customized()

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
                5, 5, 5, 5, 5, 5,
                # right leg
                5, 5, 5, 5, 5, 5,
                # waist
                5, 5, 5,
                # head
                5, 5, 5,
                # left arm
                5, 5, 5, 5, 5, 5, 5,
                # right arm
                5, 5, 5, 5, 5, 5, 5,
            ],
            "kp": [
                # left leg
                200, 200, 200, 200, 200, 200,
                # right leg
                200, 200, 200, 200, 200, 200,
                # waist
                200, 200, 200,
                # head
                200, 200, 200,
                # left arm
                200, 200, 200, 200, 200, 200, 200,
                # right arm
                200, 200, 200, 200, 200, 200, 200,
            ],
            "kd": [
                # left leg
                10, 10, 10, 10, 10, 10,
                # right leg
                10, 10, 10, 10, 10, 10,
                # waist
                10, 10, 10,
                # head
                10, 10, 10,
                # left arm
                10, 10, 10, 10, 10, 10, 10,
                # right arm
                10, 10, 10, 10, 10, 10, 10,
            ],
            # position (in urdf):
            # - unit: degree
            "position": [
                # left leg
                0, 0, 0, 0, 0, 0,
                # right leg
                0, 0, 0, 0, 0, 0,
                # waist
                0, 0, 0,
                # head
                0, 0, 0,
                # left arm
                0, 0, 0, 0, 0, 0, 0,
                # right arm
                0, 0, 0, 0, 0, 0, 0,
            ],
        })

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

        # TODO: allow use more accurate control frequency
        # time_to_sleep_mark_in_s = time.time()
        # while True:
        #     time_offset_in_s = time.time() - time_to_sleep_mark_in_s
        #     if time_offset_in_s >= time_to_sleep_in_s:
        #         break


def algorithm_user_customized():
    # user customized algorithm
    pass


if __name__ == "__main__":
    main(sys.argv)
