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

import os
import sys
import time
import numpy
import torch

# import robot_rcs and robot_rcs_gr
from robot_rcs.control_system.fi_control_system import ControlSystem
from robot_rcs_gr.robot.fi_robot_interface import RobotInterface  # Note: must be imported!


def main(argv):
    # TODO: upgrade to 1000Hz
    """
    control frequency
    request : < 500Hz
    """
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

        joint_measured_position = joint_position
        joint_measured_velocity = joint_velocity

        # print("imu_quat", imu_quat)
        # print("imu_angular_velocity", imu_angular_velocity)
        # print("joint_measured_position", joint_measured_position)
        # print("joint_measured_velocity", joint_measured_velocity)

        # algorithm (user customized...)
        joint_target_position = algorithm_rl_walk(imu_quat,
                                                  imu_angular_velocity,
                                                  joint_measured_position,
                                                  joint_measured_velocity)

        # joint_target_position = numpy.array([
        #     0.0, 0.0, -0.2618, 0.5236, -0.2618, 0.0,  # left leg (6)
        #     0.0, 0.0, -0.2618, 0.5236, -0.2618, 0.0,  # right leg (6)
        #     0.0, 0.0, 0.0,  # waist (3)
        #     0.0, 0.0, 0.0,  # head (3)
        #     0.0, 0.2, 0.0, -0.3, 0.0, 0.0, 0.0,  # left arm (7)
        #     0.0, -0.2, 0.0, -0.3, 0.0, 0.0, 0.0,  # right arm (7)
        # ]) / numpy.pi * 180

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
                0.2, 0.2, 0.2, 0.2, 0.005, 0.005, 0.005,
                # right arm
                0.2, 0.2, 0.2, 0.2, 0.005, 0.005, 0.005,
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
                0.02, 0.02, 0.02, 0.02, 0.005, 0.005, 0.005,
                # right arm
                0.02, 0.02, 0.02, 0.02, 0.005, 0.005, 0.005,
            ],
            # position (in urdf):
            # - unit: degree
            "position": joint_target_position
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


# --------------------------------------------------------------------------------------

command = torch.tensor([[0.0, 0.0, 0.0]])
actor = None
last_action = None
action_max = torch.tensor([[
    0.79, 0.7, 0.7, 1.92, 0.52,  # left leg (5), no ankle roll, more simple model
    0.09, 0.7, 0.7, 1.92, 0.52,  # left leg (5), no ankle roll, more simple model
]]) + 60 / 100 * torch.pi / 3
action_min = torch.tensor([[
    -0.09, -0.7, -1.75, -0.09, -1.05,  # left leg (5), no ankle roll, more simple model
    -0.79, -0.7, -1.75, -0.09, -1.05,  # left leg (5), no ankle roll, more simple model
]]) - 60 / 100 * torch.pi / 3
joint_default_position = torch.tensor([[
    0.0, 0.0, -0.2618, 0.5236, -0.2618, 0.0,  # left leg (6)
    0.0, 0.0, -0.2618, 0.5236, -0.2618, 0.0,  # right leg (6)
    0.0, 0.0, 0.0,  # waist (3)
    0.0, 0.0, 0.0,  # head (3)
    0.0, 0.2, 0.0, -0.3, 0.0, 0.0, 0.0,  # left arm (7)
    0.0, -0.2, 0.0, -0.3, 0.0, 0.0, 0.0,  # right arm (7)
]], dtype=torch.float32)
base_height_target = 0.90
gravity_vector = torch.tensor([[0.0, 0.0, -1.0]])

num_joint = 32
num_actor_obs = 39
num_critic_obs = 168
num_actions = 10
index_joint_controlled = [0, 1, 2, 3, 4,
                          6, 7, 8, 9, 10]


def quat_rotate_inverse(q, v):
    shape = q.shape
    q_w = q[:, -1]
    q_vec = q[:, :3]
    a = v * (2.0 * q_w ** 2 - 1.0).unsqueeze(-1)
    b = torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
    c = q_vec * torch.bmm(q_vec.view(shape[0], 1, 3), v.view(shape[0], 3, 1)).squeeze(-1) * 2.0
    return a - b + c


def algorithm_rl_walk(imu_quat,
                      imu_angular_velocity,
                      joint_measured_position,
                      joint_measured_velocity) -> list:
    global command
    global actor, last_action

    # parse to torch
    imu_quat_tensor = torch.tensor(imu_quat, dtype=torch.float32).unsqueeze(0)
    imu_angular_velocity_tensor = torch.tensor(imu_angular_velocity, dtype=torch.float32).unsqueeze(0)
    joint_measured_position_tensor = torch.tensor(joint_measured_position, dtype=torch.float32).unsqueeze(0)
    joint_measured_velocity_tensor = torch.tensor(joint_measured_velocity, dtype=torch.float32).unsqueeze(0)

    imu_angular_velocity_tensor = imu_angular_velocity_tensor / 180.0 * torch.pi  # unit : rad/s
    joint_measured_position_tensor = joint_measured_position_tensor / 180.0 * torch.pi  # unit : rad
    joint_measured_velocity_tensor = joint_measured_velocity_tensor / 180.0 * torch.pi  # unit : rad/s

    joint_offset_position_tensor = joint_measured_position_tensor - joint_default_position

    # load actor
    if actor is None:
        from robot_rcs.rl.rl_actor_critic_mlp import ActorCriticMLP

        model_file_path = os.path.dirname(os.path.abspath(__file__)) + "/data/model_4000.pt"
        print("algorithm_rl_walk model_file_path = ", model_file_path)

        model = torch.load(model_file_path, map_location=torch.device("cpu"))
        model_actor_dict = model["model_state_dict"]

        actor = ActorCriticMLP(num_actor_obs=39,
                               num_critic_obs=168,
                               num_actions=10,
                               actor_hidden_dims=[512, 256, 128],
                               critic_hidden_dims=[512, 256, 128])

        actor.load_state_dict(model_actor_dict)

    # when first run, last_action is set to measured joint position
    if last_action is None:
        last_action = torch.zeros((1, num_actions), dtype=torch.float32)
        for i in range(len(index_joint_controlled)):
            index = index_joint_controlled[i]
            last_action[0, i] = joint_offset_position_tensor[0, index]

    # command
    command = torch.tensor([[0.0, 0.0, 0.0]], dtype=torch.float32)

    # project gravity
    project_gravity_tensor = quat_rotate_inverse(imu_quat_tensor, gravity_vector)

    # joint position and velocity
    joint_controlled_position_tensor = torch.zeros((1, num_actions), dtype=torch.float32)
    joint_controlled_velocity_tensor = torch.zeros((1, num_actions), dtype=torch.float32)
    for i in range(len(index_joint_controlled)):
        index = index_joint_controlled[i]
        joint_controlled_position_tensor[0, i] = joint_offset_position_tensor[0, index]
        joint_controlled_velocity_tensor[0, i] = joint_measured_velocity_tensor[0, index]

    # actor-critic
    observation = torch.cat((
        imu_angular_velocity_tensor,
        project_gravity_tensor,
        command,
        joint_controlled_position_tensor,
        joint_controlled_velocity_tensor,
        last_action), dim=1)

    action = actor(observation)

    # clip action
    action = torch.max(action, action_min)
    action = torch.min(action, action_max)

    # record action
    last_action = action

    joint_target_position = torch.zeros((1, num_joint), dtype=torch.float32)
    for i in range(len(index_joint_controlled)):
        index = index_joint_controlled[i]
        joint_target_position[0, index] = action[0, i]
    joint_target_position += joint_default_position

    # parse to numpy
    joint_target_position = joint_target_position.detach().numpy()
    joint_target_position = joint_target_position / numpy.pi * 180.0
    joint_target_position = joint_target_position[0]

    # print("algorithm_rl_walk joint_target_position = \n", joint_target_position)

    return joint_target_position


# --------------------------------------------------------------------------------------


if __name__ == "__main__":
    main(sys.argv)
