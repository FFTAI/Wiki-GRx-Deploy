import sys
import time

from robot_rcs.control_system.fi_control_system import ControlSystem
from robot_rcs_gr.robot.fi_robot_interface import RobotInterface


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
        state_dict = RobotInterface().instance.control_loop_intf_get_state()

        # state:
        # - imu: quat, euler angle, angular velocity, linear acceleration
        # - joint: position, velocity, torque (in urdf)
        # - base: linear velocity, height
        # print("state_dict = \n", state_dict)

        # algorithm (user customized...)

        # control:
        # - control_mode
        # - position
        # - kp
        # - kd
        control_dict.update({
            # control mode:
            # - 4: position control
            # - 5: PD control
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
        RobotInterface().instance.control_loop_intf_set_control(control_dict)

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

        time_to_sleep_mark_in_s = time.time()
        while True:
            time_offset_in_s = time.time() - time_to_sleep_mark_in_s
            if time_offset_in_s >= time_to_sleep_in_s:
                break


if __name__ == "__main__":
    main(sys.argv)
