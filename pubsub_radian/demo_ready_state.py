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
    python demo_xxx.py

"""

import numpy
import time
import zenoh
import msgpack
import json
from ischedule import run_loop, schedule

import fourier_grx.sdk.pubsub as fourier_grx

prefix = "fourier-grx"

robot_state_zenoh_subscriber = None
task_state_zenoh_subscriber = None
robot_control_zenoh_publisher = None
task_control_zenoh_publisher = None

state_dict = {}
control_dict = {}
task_dict = {}

move_count = 0
move_period = 100
joint_start_position = None


def demo_task():
    global robot_state_zenoh_subscriber, task_state_zenoh_subscriber
    global robot_control_zenoh_publisher, task_control_zenoh_publisher

    # 初始化 zenoh 会话
    zenoh_config = zenoh.Config.from_json5(
        json=json.dumps(
            {
                "mode": "peer",
                "transport": {
                    "auth": {
                        "usrpwd": {
                            "user": "fourier-grx",  # 修改为匹配当前通信环境的 username
                            "password": "fourier-grx",  # 修改为匹配当前通信环境的 password
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

    # 构建发布者
    robot_control_zenoh_publisher = zenoh_session.declare_publisher(
        key_expr=f"{prefix}/robot/control",  # 发布者的 key 表达式
        priority=zenoh.Priority.REAL_TIME,
        congestion_control=zenoh.CongestionControl.DROP,
    )
    task_control_zenoh_publisher = zenoh_session.declare_publisher(
        key_expr=f"{prefix}/task/control",  # 发布者的 key 表达式
        priority=zenoh.Priority.REAL_TIME,
        congestion_control=zenoh.CongestionControl.DROP,
    )

    # 设置使能
    task_dict = {
        "task_command": fourier_grx.TaskCommand.TASK_SERVO_ON,
    }

    task_control_zenoh_publisher.put(msgpack.packb(task_dict))

    # 等待一段时间，确保任务切换成功
    time.sleep(1)

    # 设置远程控制
    control_dict = {
        "task_command": fourier_grx.TaskCommand.TASK_REMOTE_CONTROL,
    }

    task_control_zenoh_publisher.put(msgpack.packb(control_dict))

    # 等待一段时间，确保任务切换成功
    time.sleep(1)

    # 设置机器人算法频率
    target_control_frequency = 50  # 机器人控制频率, 50Hz
    target_control_period_in_s = 1.0 / target_control_frequency  # 机器人控制周期

    # 设置定时任务
    schedule(algorithm, interval=target_control_period_in_s)

    run_loop()


def state_handler(sample: zenoh.Sample):
    global state_dict, control_dict, task_dict

    """
    Robot States:
    - imu:
      - quat
      - euler angle (rpy) [rad]
      - angular velocity [rad/s]
      - linear acceleration [m/s^2]
    - joint (in urdf):
      - position [rad]
      - velocity [rad/s]
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

            state_dict["imu_quat"] = imu_quat
            state_dict["imu_euler_angle"] = imu_euler_angle
            state_dict["imu_angular_velocity"] = imu_angular_velocity
            state_dict["imu_acceleration"] = imu_acceleration
            state_dict["joint_position"] = joint_position
            state_dict["joint_velocity"] = joint_velocity
            state_dict["joint_effort"] = joint_effort

        if key_expr_str == f"{prefix}/task/state":
            # print the task state

            # parse state
            task_execute = data_dict.get("task_execute", False)
            component_execute = data_dict.get("component_execute", False)

            state_dict["task_execute"] = task_execute
            state_dict["component_execute"] = component_execute


def algorithm():
    global state_dict, control_dict, task_dict
    global move_count, move_period, joint_start_position

    # --------------------------------------------------

    robot_number_of_joint = 6 + 6 + 1 + 5 + 5

    # parse state
    imu_quat = state_dict.get("imu_quat", [0, 0, 0, 1])
    imu_euler_angle = state_dict.get("imu_euler_angle", [0, 0, 0])
    imu_angular_velocity = state_dict.get("imu_angular_velocity", [0, 0, 0])
    imu_acceleration = state_dict.get("imu_acceleration", [0, 0, 0])
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
            # left leg
            -0.2468, 0.0, 0.0, +0.5181, 0.0, -0.2408,
            # right leg
            -0.2468, 0.0, 0.0, +0.5181, 0.0, -0.2408,
            # waist
            0.0,
            # left arm
            0.0, 0.0, 0.0, 0.0, 0.0,
            # right arm
            0.0, 0.0, 0.0, 0.0, 0.0,
        ])  # [rad]

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

    # --------------------------------------------------

    """
    Robot Control:
    - control_mode
    - kp
    - kd
    - position [rad]
    """
    control_dict = {
        "control_mode": joint_target_control_mode.copy().tolist(),
        "kp": joint_target_kp.copy().tolist(),
        "kd": joint_target_kd.copy().tolist(),
        "position": joint_target_position.copy().tolist(),
    }

    # output control
    robot_control_zenoh_publisher.put(msgpack.packb(control_dict))


if __name__ == "__main__":
    demo_task()
