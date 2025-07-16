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

这个 demo 要求连接摇杆, 用摇杆控制机器人走路速度

"""

import os
import numpy
import torch
from ischedule import run_loop, schedule

import fourier_grx.sdk.developer as fourier_grx

control_system = fourier_grx.ControlSystem()

policy_file_path = None
policy_model = None
policy_action = None
obs_buf_stack = None


def main():
    global policy_file_path, policy_model

    # 设置机器人算法频率
    target_control_frequency = 50  # 机器人控制频率, 50Hz
    target_control_period_in_s = 1.0 / target_control_frequency  # 机器人控制周期

    # 切换为开发者模式，设置机器人数据更新频率
    control_system.developer_mode(servo_on=True, control_frequency=100)

    # 打印版本信息
    print(control_system.get_info())

    # Load Model
    policy_file_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "policy_jit_rl_walk.pt",
    )

    policy_model = torch.jit.load(policy_file_path, map_location=torch.device('cpu'))

    # 设置定时任务
    schedule(algorithm, interval=target_control_period_in_s)

    run_loop()


def algorithm():
    global policy_model, policy_action, obs_buf_stack

    # update state
    """
    state:
    - imu:
      - quat (x, y, z, w)
      - euler angle (rpy) [rad]
      - angular velocity [rad/s]
      - linear acceleration [m/s^2]
    - joint (in urdf):
      - position [rad]
      - velocity [rad/s]
      - torque [Nm]
    """
    state_dict = control_system.robot_control_loop_get_state()

    # --------------------------------------------------

    robot_number_of_joint = 6 + 6 + 1 + 5 + 5

    policy_control_number_of_joint = 6 + 6 + 1  # left leg + right leg + waist
    policy_control_index_of_joints = numpy.array([
        0, 1, 2, 3, 4, 5,  # left leg
        6, 7, 8, 9, 10, 11,  # right leg
        12,  # waist
    ])

    # get states
    imu_measured_quat = state_dict.get("imu_quat", [0, 0, 0, 1])
    imu_measured_angular_velocity = state_dict.get("imu_angular_velocity", [0, 0, 0])
    joint_measured_position = state_dict.get("joint_position", [0] * robot_number_of_joint)
    joint_measured_velocity = state_dict.get("joint_velocity", [0] * robot_number_of_joint)

    # --------------------------------------------------

    # constants
    default_joint_position = numpy.array([
        # left leg
        -0.2468, 0.0, 0.0, 0.5181, 0.0, -0.2408,
        # right leg
        -0.2468, 0.0, 0.0, 0.5181, 0.0, -0.2408,
        # waist
        0.0,
    ])
    gravity_vector = numpy.array([
        0.0, 0.0, -1.0
    ])
    action_clip_max = numpy.array([
        2.618, 1.571, 1.571, 2.356, 0.436, 0.785,  # left leg
        2.618, 0.262, 1.571, 2.356, 0.436, 0.785,  # right leg
        2.618,  # waist
    ]) + numpy.array([
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5,  # left leg
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5,  # right leg
        0.5,  # waist
    ])
    action_clip_min = numpy.array([
        -2.618, -0.262, -1.571, -0.087, -0.436, -0.785,  # left leg
        -2.618, -1.571, -1.571, -0.087, -0.436, -0.785,  # right leg
        -2.618,  # waist
    ]) - numpy.array([
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5,  # left leg
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5,  # right leg
        0.5,  # waist
    ])

    # --------------------------------------------------

    # prepare input

    # 指令速度: (可修改为摇杆控制)
    # [lin_vel_x, lin_vel_y, ang_vel_yaw], unit: m/s, m/s, rad/s
    commands = numpy.array([0.0, 0.0, 0.0, ])

    base_measured_quat = imu_measured_quat
    base_measured_angular_velocity = imu_measured_angular_velocity

    joint_measured_position_for_policy = numpy.zeros(policy_control_number_of_joint)
    joint_measured_velocity_for_policy = numpy.zeros(policy_control_number_of_joint)

    for i in range(policy_control_number_of_joint):
        index = policy_control_index_of_joints[i]
        joint_measured_position_for_policy[i] = joint_measured_position[index]
        joint_measured_velocity_for_policy[i] = joint_measured_velocity[index]

    if policy_action is None:
        policy_action = numpy.zeros(policy_control_number_of_joint)

    # run algorithm
    torch_commands = torch.from_numpy(commands).float().unsqueeze(0)
    torch_base_measured_quat = torch.from_numpy(base_measured_quat).float().unsqueeze(0)
    torch_base_measured_angular_velocity = torch.from_numpy(base_measured_angular_velocity).float().unsqueeze(0)
    torch_joint_measured_position_for_policy = torch.from_numpy(joint_measured_position_for_policy).float().unsqueeze(0)
    torch_joint_measured_velocity_for_policy = torch.from_numpy(joint_measured_velocity_for_policy).float().unsqueeze(0)
    torch_default_joint_position = torch.from_numpy(default_joint_position).float().unsqueeze(0)

    def torch_quat_rotate_inverse(q, v):
        """
        Rotate a vector (tensor) by the inverse of a quaternion (tensor).

        :param q: A quaternion tensor in the form of [x, y, z, w] in shape of [N, 4].
        :param v: A vector tensor in the form of [x, y, z] in shape of [N, 3].
        :return: The rotated vector tensor in shape of [N, 3].
        """
        q_w = q[:, -1:]
        q_vec = q[:, :3]

        # Compute the dot product of q_vec and v
        q_vec_dot_v = torch.bmm(q_vec.view(-1, 1, 3), v.view(-1, 3, 1)).squeeze(-1)

        # Compute the cross product of q_vec and v
        q_vec_cross_v = torch.cross(q_vec, v, dim=-1)

        # Compute the rotated vector
        a = v * (2.0 * q_w ** 2 - 1.0)
        b = q_vec_cross_v * q_w * 2.0
        c = q_vec * q_vec_dot_v * 2.0

        return a - b + c

    torch_gravity_vector = torch.from_numpy(gravity_vector).float().unsqueeze(0)
    torch_base_project_gravity = torch_quat_rotate_inverse(torch_base_measured_quat, torch_gravity_vector)
    torch_measured_position_offset_for_policy = torch_joint_measured_position_for_policy \
                                                - torch_default_joint_position
    torch_action = torch.from_numpy(policy_action).float().unsqueeze(0)

    obs_buf = torch.cat([
        torch_commands,
        torch_base_measured_angular_velocity,
        torch_base_project_gravity,
        torch_measured_position_offset_for_policy,
        torch_joint_measured_velocity_for_policy * 0.1,
        torch_action,
    ], dim=-1)

    obs_len = obs_buf.shape[-1]
    stack_size = 5

    if obs_buf_stack is None:
        obs_buf_stack = torch.cat([obs_buf] * stack_size, dim=1).float()

    obs_buf_stack = torch.cat([
        obs_buf_stack[:, obs_len:],
        obs_buf,
    ], dim=1).float()

    torch_policy_action = policy_model(obs_buf_stack).detach()

    torch_policy_action = torch.clip(
        torch_policy_action,
        min=torch.from_numpy(action_clip_min).float().unsqueeze(0),
        max=torch.from_numpy(action_clip_max).float().unsqueeze(0),
    )

    # 记录上一次的 action
    policy_action = torch_policy_action.numpy().squeeze(0)

    torch_joint_target_position_from_policy = torch_policy_action \
                                              + torch_default_joint_position

    joint_target_position_from_policy = torch_joint_target_position_from_policy.numpy().squeeze(0)  # unit : rad
    joint_target_position_from_policy = joint_target_position_from_policy  # unit : rad

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
    joint_target_position = numpy.zeros(robot_number_of_joint)

    for i in range(policy_control_number_of_joint):
        index = policy_control_index_of_joints[i]
        joint_target_position[index] = joint_target_position_from_policy[i]

    # --------------------------------------------------

    # set control
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
