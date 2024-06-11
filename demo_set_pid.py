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
    ControlSystem().developer_mode(servo_on=False)

    control_dict = {}

    # algorithm (user customized...)
    algorithm_set_pid(control_dict)


def algorithm_set_pid(control_dict):
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


if __name__ == "__main__":
    main(sys.argv)
