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
import time
import zenoh
import msgpack
import json

prefix = "fourier-grx"


def demo_task():
    # 初始化 zenoh 会话
    zenoh_config = zenoh.Config.from_json5(
        json=json.dumps(
            {
                "mode": "peer",
                "transport": {
                    "auth": {
                        "usrpwd": {
                            "user": "fourier-grx",
                            "password": "fourier-grx",
                            "dictionary_file": "./credentials.txt",
                        }
                    },
                },
            }
        )
    )

    zenoh_session: zenoh.Session = zenoh.open(zenoh_config)

    # 构建接收者
    robot_state_zenoh_subscriber = zenoh_session.declare_subscriber(
        key_expr=f"{prefix}/robot/state",  # 目标发布者的 key 表达式
        handler=state_handler,
    )
    task_state_zenoh_subscriber = zenoh_session.declare_subscriber(
        key_expr=f"{prefix}/task/state",  # 目标发布者的 key 表达式
        handler=state_handler,
    )

    # 等待...
    while True:
        time.sleep(1)


def state_handler(sample: zenoh.Sample):
    """
    Update and print state
    """

    """
    Robot States:
    - imu:
      - quat
      - euler angle (rpy) [deg]
      - angular velocity [deg/s]
      - linear acceleration [m/s^2]
    - joint (in urdf):
      - position [deg]
      - velocity [deg/s]
      - torque [Nm]
      
    Task States:
    - task_execute
    - component_execute
    """

    key_expr = sample.key_expr
    key_expr_str = str(key_expr)

    # change from builtins.ZBytes to bytes-like object
    sample_value = sample.payload.to_bytes()

    # get the data_dict from the sample_value
    data_dict = msgpack.unpackb(sample_value)

    # Convert data_dict to a format suitable for Python processing
    for key, value in data_dict.items():
        if isinstance(value, bytes):
            data_dict[key] = value.decode('utf-8')  # 默认用 utf-8 解码

    # check if the key_expr_str is in the zenoh_keys for robot subscribers
    if data_dict:
        if key_expr_str == f"{prefix}/robot/state":
            # print the robot state
            robot_number_of_joint = 6 + 6 + 1 + 5 + 5

            # parse state
            imu_quat = data_dict.get("imu_quat", [0, 0, 0, 1])
            imu_euler_angle = data_dict.get("imu_euler_angle", [0, 0, 0])
            imu_angular_velocity = data_dict.get("imu_angular_velocity", [0, 0, 0])
            imu_acceleration = data_dict.get("imu_acceleration", [0, 0, 0])
            joint_position = data_dict.get("joint_position", [0] * robot_number_of_joint)
            joint_velocity = data_dict.get("joint_velocity", [0] * robot_number_of_joint)
            joint_effort = data_dict.get("joint_effort", [0] * robot_number_of_joint)

            # print state
            print("#################################################")
            print("imu_quat = \n", numpy.round(imu_quat, 3))
            print("imu_euler_angle = \n", numpy.round(imu_euler_angle, 3))
            print("imu_angular_velocity = \n", numpy.round(imu_angular_velocity, 3))
            print("imu_acceleration = \n", numpy.round(imu_acceleration, 3))
            print("joint_position = \n", numpy.round(joint_position, 3))
            print("joint_velocity = \n", numpy.round(joint_velocity, 3))
            print("joint_effort = \n", numpy.round(joint_effort, 3))

        if key_expr_str == f"{prefix}/task/state":
            # print the task state

            # parse state
            task_execute = data_dict.get("task_execute", False)
            component_execute = data_dict.get("component_execute", False)

            # print state
            print("#################################################")
            print("task_execute = ", task_execute)
            print("component_execute = ", component_execute)


if __name__ == "__main__":
    demo_task()
