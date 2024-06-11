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
    target_control_frequency = 1  # 机器人控制频率, 1Hz
    target_control_period_in_s = 1.0 / target_control_frequency  # 机器人控制周期

    # dev mode
    ControlSystem().developer_mode(servo_on=False)

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

        # print state
        print("#################################################")
        print("imu_quat = \n", numpy.round(imu_quat, 3))
        print("imu_euler_angle = \n", numpy.round(imu_euler_angle, 3))
        print("imu_angular_velocity = \n", numpy.round(imu_angular_velocity, 3))
        print("imu_acceleration = \n", numpy.round(imu_acceleration, 3))
        print("joint_position = \n", numpy.round(joint_position, 3))
        print("joint_velocity = \n", numpy.round(joint_velocity, 3))
        print("joint_kinetic = \n", numpy.round(joint_kinetic, 3))
        print("base_xyz = \n", numpy.round(base_xyz, 3))
        print("base_vel_xyz = \n", numpy.round(base_vel_xyz, 3))

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


if __name__ == "__main__":
    main(sys.argv)
