import os
import numpy
import torch
from enum import Enum

from controller import InertialUnit
from controller import Robot
from controller import Keyboard
from controller import Supervisor
from controller import Node

import robot_rcs.robot.fi_robot_tool as fi_robot_tool
from robot_rcs.webots.webots_joystick import WebotsJoystick, WebotsJoystickType
from robot_rcs.webots.webots_robot import WebotsRobot, WebotsRobotTask

from robot_rcs_base.fi_robot_gr1t1_algorithm import RobotGR1T1AlgorithmStandControlModel


class WebotsGR1T1Task(Enum):
    IDLE = WebotsRobotTask.IDLE.value
    STAND = 1


class WebotsGR1T1(WebotsRobot):
    def __init__(self, sim_dt):
        super().__init__(sim_dt=sim_dt)

        self.robot_name = "GR1T1"

        self.base_target_height = 0.90

        self.num_of_legs = 2

        self.num_of_links = 6 + 6 + 3 + 3 + 7 + 7
        self.links_name = [
            # left leg
            "l_thigh_roll", "l_thigh_yaw", "l_thigh_pitch", "l_shank_pitch", "l_foot_pitch", "l_foot_roll",
            # right leg
            "r_thigh_roll", "r_thigh_yaw", "r_thigh_pitch", "r_shank_pitch", "r_foot_pitch", "r_foot_roll",
            # waist
            "waist_yaw", "waist_pitch", "waist_roll",
            # head
            "head_yaw", "head_roll", "head_pitch",
            # left arm
            "l_upper_arm_pitch", "l_upper_arm_roll", "l_upper_arm_yaw",
            "l_lower_arm_pitch",
            "l_wrist_yaw", "l_wrist_roll", "l_wrist_pitch",
            # right arm
            "r_upper_arm_pitch", "r_upper_arm_roll", "r_upper_arm_yaw",
            "r_lower_arm_pitch",
            "r_wrist_yaw", "r_wrist_roll", "r_wrist_pitch",
        ]

        self.num_of_joints = 6 + 6 + 3 + 3 + 7 + 7
        self.joints_name = [
            # left leg
            "l_hip_roll", "l_hip_yaw", "l_hip_pitch", "l_knee_pitch", "l_ankle_pitch", "l_ankle_roll",
            # right leg
            "r_hip_roll", "r_hip_yaw", "r_hip_pitch", "r_knee_pitch", "r_ankle_pitch", "r_ankle_roll",
            # waist
            "waist_yaw", "waist_pitch", "waist_roll",
            # head
            "head_yaw", "head_roll", "head_pitch",
            # left arm
            "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw",
            "l_elbow_pitch",
            "l_wrist_yaw", "l_wrist_roll", "l_wrist_pitch",
            # right arm
            "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw",
            "r_elbow_pitch",
            "r_wrist_yaw", "r_wrist_roll", "r_wrist_pitch",
        ]
        self.joints_kp = [
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0,
        ]
        self.joints_ki = [
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0,
        ]
        self.joints_kd = [
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0,
        ]

        self.num_of_joint_position_sensors = 6 + 6 + 3 + 3 + 7 + 7
        self.joint_position_sensors_name = [
            # left leg
            "l_hip_roll_sensor", "l_hip_yaw_sensor", "l_hip_pitch_sensor", "l_knee_pitch_sensor", "l_ankle_pitch_sensor", "l_ankle_roll_sensor",
            # right leg
            "r_hip_roll_sensor", "r_hip_yaw_sensor", "r_hip_pitch_sensor", "r_knee_pitch_sensor", "r_ankle_pitch_sensor", "r_ankle_roll_sensor",
            # waist
            "waist_yaw_sensor", "waist_pitch_sensor", "waist_roll_sensor",
            # head
            "head_yaw_sensor", "head_roll_sensor", "head_pitch_sensor",
            # left arm
            "l_shoulder_pitch_sensor", "l_shoulder_roll_sensor", "l_shoulder_yaw_sensor",
            "l_elbow_pitch_sensor",
            "l_wrist_yaw_sensor", "l_wrist_roll_sensor", "l_wrist_pitch_sensor",
            # right arm
            "r_shoulder_pitch_sensor", "r_shoulder_roll_sensor", "r_shoulder_yaw_sensor",
            "r_elbow_pitch_sensor",
            "r_wrist_yaw_sensor", "r_wrist_roll_sensor", "r_wrist_pitch_sensor",
        ]

        self.num_of_imus = 1
        self.imus_name = [
            "inertial_unit"
        ]

        self.num_of_gyros = 1
        self.gyros_name = [
            "gyro"
        ]

        self.num_of_accelerometers = 1
        self.accelerometers_name = [
            "accelerometer"
        ]

        # self.tasks
        self.tasks = [
            WebotsGR1T1Task.IDLE,
            WebotsGR1T1Task.STAND,
        ]

        self.task_selected = WebotsGR1T1Task.STAND.value
        self.task_selected_last = self.task_selected
        self.task_assigned = WebotsGR1T1Task.STAND.value
        self.task_assigned_last = self.task_assigned

        # avg
        self.base_measured_quat_to_world_buffer_length = 10
        self.base_measured_quat_to_world_buffer = []
        self.base_measured_quat_to_world_avg = numpy.zeros(4)

        self.base_measured_rpy_vel_to_self_buffer_length = 10
        self.base_measured_rpy_vel_to_self_buffer = []
        self.base_measured_rpy_vel_to_self_avg = numpy.zeros(3)

        self.joint_measured_velocity_value_buffer_length = 10
        self.joint_measured_velocity_value_buffer = []
        self.joint_measured_velocity_value_avg = numpy.zeros(self.num_of_joints)

        # algorithm models
        self.stand_algorithm_model = RobotGR1T1AlgorithmStandControlModel()

        # pd control
        self.joint_pd_control_target = numpy.zeros(self.num_of_joints)
        self.joint_pd_control_output = numpy.zeros(self.num_of_joints)

        self.joint_pd_control_target_buffer = []
        self.joint_pd_control_target_delay = 0

        for i in range(self.joint_pd_control_target_delay + 1):
            self.joint_pd_control_target_buffer.append(numpy.zeros(self.num_of_joints))

        self.joint_pd_control_kp = numpy.array([
            251.625, 362.5214, 200.0, 200.0, 10.9885, 0.25,  # left leg(6)
            251.625, 362.5214, 200.0, 200.0, 10.9885, 0.25,  # right leg(6)
            362.5214, 362.5214, 362.5214,  # waist(3)
            10, 10, 10,  # head(3)
            92.85, 92.85, 112.06, 112.06, 10, 10, 10,  # left arm(7)
            92.85, 92.85, 112.06, 112.06, 10, 10, 10,  # right arm(7)
        ])
        self.joint_pd_control_kd = numpy.array([
            14.72, 10.0833, 11.0, 11.0, 0.5991, 0.01,  # left leg(6)
            14.72, 10.0833, 11.0, 11.0, 0.5991, 0.01,  # right leg(6)
            10.0833, 10.0833, 10.0833,  # waist(3)
            1, 1, 1,  # head(3)
            2.575, 2.575, 3.1, 3.1, 1, 1, 1,  # left arm(7)
            2.575, 2.575, 3.1, 3.1, 1, 1, 1,  # right arm(7)
        ])
        self.joint_pd_control_max = numpy.array([
            100.0, 82.5, 150.0, 150.0, 16.0, 8.0,  # left leg(6)
            100.0, 82.5, 150.0, 150.0, 16.0, 8.0,  # right leg(6)
            82.5, 82.5, 82.5,  # waist(3)
            10.0, 3.95, 3.95,  # head(3)
            38.0, 38.0, 30.0, 30.0, 10.0, 10.0, 10.0,  # left arm(7)
            38.0, 38.0, 30.0, 30.0, 10.0, 10.0, 10.0,  # right arm(7)
        ])
        self.joint_pd_control_min = numpy.array([
            -100.0, -82.5, -150.0, -150.0, -16.0, -8.0,  # left leg(6)
            -100.0, -82.5, -150.0, -150.0, -16.0, -8.0,  # right leg(6)
            -82.5, -82.5, -82.5,  # waist(3)
            -10.0, -3.95, -3.95,  # head(3)
            -38.0, -38.0, -30.0, -30.0, -10.0, -10.0, -10.0,  # left arm(7)
            -38.0, -38.0, -30.0, -30.0, -10.0, -10.0, -10.0,  # right arm(7)
        ])

        # record
        self.flag_record = False
        self.record_tick = 0
        self.record_start = 0
        self.record_length = 5000

    def control_loop_update_joystick_state_print_selected_task(self):
        for item in WebotsGR1T1Task:
            if self.tasks[self.task_selected] == item:
                print("task_selected = ", item)
                break

    def control_loop_update_joystick_state_print_assigned_task(self):
        for item in WebotsGR1T1Task:
            if self.tasks[self.task_assigned] == item:
                print("task_assigned = ", item)
                break

    def before_control_loop(self):
        super().before_control_loop()

        # assign control model
        self.task_algorithm_model = self.stand_algorithm_model

    def control_loop_update_task_state(self):
        if self.tasks[self.task_assigned] == WebotsGR1T1Task.IDLE \
                and self.tasks[self.task_assigned_last] != WebotsGR1T1Task.IDLE:
            print("Info: robot task IDLE")
            for joint in self.joints:
                joint.setPosition(0)

        elif self.tasks[self.task_assigned] == WebotsGR1T1Task.STAND \
                and self.tasks[self.task_assigned_last] != WebotsGR1T1Task.STAND:
            print("Info: robot task STAND")
            for joint in self.joints:
                joint.setPosition(0)

            self.task_algorithm_model = self.stand_algorithm_model

        else:
            pass

        self.set_task_assigned(self.task_assigned)

    def control_loop_algorithm(self):
        if self.tasks[self.task_assigned] == WebotsGR1T1Task.IDLE:
            pass

        elif self.tasks[self.task_assigned] == WebotsGR1T1Task.STAND:

            self.stand_algorithm_model.run(
                joint_measured_position_value=self.joint_measured_position_value,
                joint_measured_velocity_value=self.joint_measured_velocity_value_avg
            )

            self.joint_pd_control_target = self.stand_algorithm_model.output_joint_position

        else:
            pass

    def control_loop_output(self):

        self.joint_pd_control_target_to_sim = self.joint_pd_control_target

        # PD Control
        if self.tasks[self.task_assigned] != WebotsGR1T1Task.IDLE:
            # pd control
            self.joint_pd_control_output = \
                self.joint_pd_control_kp * (self.joint_pd_control_target_to_sim
                                            - self.joint_measured_position_value) \
                - self.joint_pd_control_kd * (self.joint_measured_velocity_value)

            self.joint_pd_control_output = \
                numpy.clip(self.joint_pd_control_output,
                           self.joint_pd_control_min,
                           self.joint_pd_control_max)

            # output
            for i in range(self.num_of_joints):
                self.joints[i].setTorque(self.joint_pd_control_output[i])
