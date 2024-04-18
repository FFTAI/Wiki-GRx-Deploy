import os
import math
import time

import numpy
import json
from enum import Enum
from typing import List

from robot_rcs.logger.fi_logger import Logger
from robot_rcs.robot_config.fi_robot_config import gl_robot_config
from robot_rcs.callback.fi_callback import CallbackSystemExit

from robot_rcs.predefine.fi_function_result import FunctionResult
from robot_rcs.predefine.fi_flag_state import FlagState
from robot_rcs.predefine.fi_joint_control_mode import JointControlMode
from robot_rcs.predefine.fi_robot_type import RobotType
from robot_rcs.predefine.fi_robot_work_space import RobotWorkSpace
from robot_rcs.predefine.fi_task_stage import TaskStage

from robot_rcs.comm.fi_dynalinkhs_interface import DynalinkHSInterface
from robot_rcs.operator.fi_operator_joystick_interface import OperatorJoystickInterface

from robot_rcs.sensor.fi_sensor_usb_imu_hipnuc import SensorUSBIMUHipnuc
from robot_rcs.sensor.fi_sensor_fi_fse import SensorFIFSE
from robot_rcs.actuator.fi_actuator_fi_fsa import ActuatorFIFSA
from robot_rcs.actuator.fi_actuator_fi_fsa_group import ActuatorFIFSAGroup
from robot_rcs.joint.fi_joint_rotary import JointRotary
from robot_rcs.joint.fi_joint_group import JointGroup

import robot_rcs.robot.fi_robot_tool as fi_robot_tool
from robot_rcs.robot.fi_robot_base import RobotBaseTasks

from parallel_ankle import ParallelAnkle
from .fi_robot_gr1t1 import RobotGR1T1
from .fi_robot_gr1t1_customize_algorithm import RobotGR1T1AlgorithmCustomizeControlModel


class RobotGR1Tasks(Enum):
    ############################################
    # NOTE:
    # Add your own task related code here
    TASK_CUSTOMIZE = 0xCCCCCC
    ############################################


class RobotGR1T1Customize(RobotGR1T1):

    def __init__(self):
        super(RobotGR1T1Customize, self).__init__()

        # task
        self.tasks.extend([
            ############################################
            # NOTE:
            # Add your own task related code here
            ############################################
            RobotGR1Tasks.TASK_CUSTOMIZE,
        ])

        ############################################
        # NOTE:
        # Add your own task related code here
        self.algorithm_customize_control_model = RobotGR1T1AlgorithmCustomizeControlModel()
        ############################################

    def control_loop_update_command(self):
        func_result = super(RobotGR1T1Customize, self).control_loop_update_command()

        if func_result == FunctionResult.SUCCESS:
            return FunctionResult.SUCCESS
        else:
            pass

        # update command and state...
        if self.flag_task_command_update == FlagState.SET:

            ############################################
            # NOTE:
            # Add your own task related code here
            if self.task_command == RobotGR1Tasks.TASK_CUSTOMIZE:
                if self.task_state != self.task_command:
                    self.algorithm_customize_control_model.flag_inited = False
            ############################################

            else:
                return FunctionResult.FAIL

            # update state information
            self.task_state_last = self.task_state
            self.task_state = self.task_command

            # update command information
            self.task_command_last = self.task_command
            self.task_command = RobotBaseTasks.TASK_NONE

            # clear flag
            self.flag_task_command_update = FlagState.CLEAR

        else:
            pass

        return FunctionResult.SUCCESS

    def control_loop_algorithm(self):
        super(RobotGR1T1Customize, self).control_loop_algorithm()

        ############################################
        # NOTE:
        # Add your own task related code here
        if self.task_state == RobotGR1Tasks.TASK_CUSTOMIZE:
            self.flag_task_in_process = FlagState.SET
            self.algorithm_customize_control()
        ############################################

        else:
            pass

        return FunctionResult.SUCCESS

    def control_loop_update_communication_joystick_button_triangle(self):
        # ^ button press
        if OperatorJoystickInterface().instance.get_button_triangle() == 1:
            ############################################
            # NOTE:
            # Add your own task related code here
            self.task_command = RobotGR1Tasks.TASK_CUSTOMIZE
            ############################################

            self.flag_task_command_update = FlagState.SET

            Logger().print_trace("task command: " + str(self.task_command.name))

    def algorithm_customize_control(self):

        ############################################
        # NOTE:
        # Add your own task related code here
        joint_measured_position_value = self.joint_group_measured_position.copy() * numpy.pi / 180.0  # unit : rad
        joint_measured_velocity_value = self.joint_group_measured_velocity.copy() * numpy.pi / 180.0  # unit : rad/s

        work_space, control_mode, target_position = \
            self.algorithm_stand_control_model.run(
                joint_measured_position_value,
                joint_measured_velocity_value
            )
        ############################################

        target_position = target_position / numpy.pi * 180.0  # unit : degree
        # print("target1: ",target_position )
        try:
            target_position[4], \
                target_position[5], \
                _, _, _, _ = \
                self.parallel_ankle_left.inverse(
                    ankle_position_pitch=target_position[4] * numpy.pi / 180,
                    ankle_position_roll=target_position[5] * numpy.pi / 180)
        except Exception as e:
            Logger().print_trace_warning(
                "algorithm_nowhla_rl_stair_encoder_control() self.parallel_ankle_left.inverse error")

            print("joint_target_position_value = \n", target_position)

            target_position[4] = joint_measured_position_value[4]
            target_position[5] = joint_measured_position_value[5]

            # calculate error, switch back to servo_off
            Logger().print_trace("calculate error, switch back to TASK_SERVO_OFF")
            self.task_command = RobotBaseTasks.TASK_SERVO_OFF
            self.flag_task_command_update = FlagState.SET

        try:
            target_position[10], \
                target_position[11], \
                _, _, _, _ = \
                self.parallel_ankle_right.inverse(
                    ankle_position_pitch=target_position[10] * numpy.pi / 180,
                    ankle_position_roll=target_position[11] * numpy.pi / 180)
        except Exception as e:
            Logger().print_trace_warning(
                "algorithm_nowhla_rl_stair_encoder_control() self.parallel_ankle_left.inverse error")

            print("joint_target_position_value = \n", target_position)

            target_position[11] = joint_measured_position_value[11]
            target_position[10] = joint_measured_position_value[10]

            # calculate error, switch back to servo_off
            Logger().print_trace("calculate error, switch back to TASK_SERVO_OFF")
            self.task_command = RobotBaseTasks.TASK_SERVO_OFF
            self.flag_task_command_update = FlagState.SET

        target_position[4] = target_position[4] / numpy.pi * 180
        target_position[5] = target_position[5] / numpy.pi * 180
        target_position[10] = target_position[10] / numpy.pi * 180
        target_position[11] = target_position[11] / numpy.pi * 180
        # print("target2: ",target_position )

        if work_space == RobotWorkSpace.ACTUATOR_SPACE:
            self.work_space = work_space
            for i in range(self.number_of_actuator):
                self.actuators[i].set_target_control_mode(control_mode[i])

        elif work_space == RobotWorkSpace.JOINT_SPACE:
            self.work_space = work_space
            for i in range(self.number_of_joint):
                self.joints[i].set_target_control_mode(control_mode[i])
                self.joints[i].set_target_position(target_position[i])
                self.joints[i].set_target_velocity(0)
                self.joints[i].set_target_acceleration(0)
                self.joints[i].set_target_kinetic(0)

        else:
            Logger().print_trace_error("algorithm_stand_control() unknown work_space")

        return FunctionResult.SUCCESS

