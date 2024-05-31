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
    # dev mode
    ControlSystem().developer_mode()

    # print version info
    info_dict = ControlSystem().get_info()
    print(info_dict)

    # algorithm (user customized...)
    algorithm_servo_off()


def algorithm_servo_off():
    from robot_rcs.robot.fi_robot_base_task import RobotBaseTask

    ControlSystem().robot_control_set_task_command(task_command=RobotBaseTask.TASK_SERVO_OFF)

    # wait for servo off
    time.sleep(1)


if __name__ == "__main__":
    main(sys.argv)
