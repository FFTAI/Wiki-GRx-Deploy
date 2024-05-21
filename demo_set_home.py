"""
Copyright (C) [2024] [Fourier Intelligence Ltd.]

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
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

# import robot_rcs and robot_rcs_gr
from robot_rcs.control_system.fi_control_system import ControlSystem
from robot_rcs_gr.robot.fi_robot_interface import RobotInterface  # Note: must be imported!


def main(argv):
    # TODO: upgrade to 1000Hz
    # control frequency
    # request : < 500Hz
    target_control_frequency = 50  # 机器人控制频率, 50Hz
    target_control_period_in_s = 1.0 / target_control_frequency  # 机器人控制周期

    # dev mode
    ControlSystem().dev_mode()

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

        # algorithm (user customized...)
        algorithm_set_home()

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
                1.04, 0.25, 0.7, 0.7, 0.1, 0.1,
                # right leg
                1.04, 0.25, 0.7, 0.7, 0.1, 0.1,
                # waist
                0.25, 0.25, 0.25,
                # head
                0.005, 0.005, 0.005,
                # left arm
                0.2, 0.2, 0.2, 0.2, 0.005, 0.005, 0.005,
                # right arm
                0.2, 0.2, 0.2, 0.2, 0.005, 0.005, 0.005,
            ],
            "kd": [
                # left leg
                0.04, 0.14, 0.4, 0.4, 0.005, 0.005,
                # right leg
                0.04, 0.14, 0.4, 0.4, 0.005, 0.005,
                # waist
                0.14, 0.14, 0.14,
                # head
                0.005, 0.005, 0.005,
                # left arm
                0.02, 0.02, 0.02, 0.02, 0.005, 0.005, 0.005,
                # right arm
                0.02, 0.02, 0.02, 0.02, 0.005, 0.005, 0.005,
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

        # sleep some time and exit
        time.sleep(10)
        break


robot_set_home_command = 0x020109  # defined inside the RobotInterface() lib.


def algorithm_set_home():
    global robot_set_home_command

    # Notice:
    # Be careful when uncomment this, this will change the home position!!!
    # Besure when running this algorithm, the robot is in home position.

    # RobotInterface().instance.flag_task_state_update = 1  # set update command flag
    # RobotInterface().instance.task_command = robot_set_home_command  # set home command

    pass


if __name__ == "__main__":
    main(sys.argv)
