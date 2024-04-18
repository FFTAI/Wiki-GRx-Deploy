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
from robot_rcs.predefine.fi_robot_work_space import RobotWorkSpace

from robot_rcs.comm.fi_dynalinkhs_interface import DynalinkHSInterface
from robot_rcs.operator.fi_operator_joystick_interface import OperatorJoystickInterface

import robot_rcs.robot.fi_robot_tool as fi_robot_tool
from robot_rcs.robot.fi_robot_base import RobotBase
from robot_rcs.robot.fi_robot_base import RobotBaseTasks


class RobotFFTAI(RobotBase):

    def __init__(self):
        super(RobotFFTAI, self).__init__()

        # robot
        self.robot_type = "FFTAI"

        # task
        self.tasks.extend([
        ])

    def init(self) -> int:
        # Note 2024-01-26:
        # download PD value should before init() actuator,
        # init() actuator will create child thread to handle receive message if using non-block mode.
        Logger().print_trace("RobotFFTAI init() download joint pd value")
        for i in range(self.number_of_joint):
            if self.joints[i] is not None:
                self.joints[i].download_control_pid()

        super().init()

        # sensor
        for i in range(self.number_of_sensor_fi_fse):
            if self.sensor_fi_fse[i] is not None:
                self.sensor_fi_fse[i].init()

        for i in range(self.number_of_sensor_fi_fse):
            if self.sensor_fi_fse[i] is not None:
                self.sensor_fi_fse[i].comm()

        # actuator
        for i in range(self.number_of_actuator):
            if self.actuators[i] is not None:
                self.actuators[i].comm(enable=gl_robot_config.parameters["actuator"]["comm_enable"][i],
                                       block=gl_robot_config.parameters["actuator"]["comm_block"][i],
                                       use_fast=gl_robot_config.parameters["actuator"]["comm_use_fast"][i])

        return FunctionResult.SUCCESS

    def control_loop_update_state_sensor(self):

        # Note 2024-01-26:
        # no need to read fi_fse sensor in every loop
        # fi_fse
        # for i in range(self.number_of_sensor_fi_fse):
        #     self.sensor_fi_fse[i].upload()
        #
        # print("self.sensor_fi_fse = ", self.sensor_fi_fse[5].measured_angle)

        # imu
        for i in range(self.number_of_sensor_usb_imu):
            self.sensor_usb_imus[i].upload()

        # IMU x 轴朝向 x，y 轴朝向 y
        # roll x, pitch y
        for i in range(self.number_of_sensor_usb_imu):
            self.sensor_usb_imu_group_measured_angle[0 + i * 3: 3 + i * 3] = \
                self.sensor_usb_imus[i].get_measured_angle() * self.sensor_usb_imus_angle_direction[i]
            self.sensor_usb_imu_group_measured_angular_velocity[0 + i * 3: 3 + i * 3] = \
                self.sensor_usb_imus[i].get_measured_angular_velocity() * self.sensor_usb_imus_angular_velocity_direction[i]
            self.sensor_usb_imu_group_measured_linear_acceleration[0 + i * 3: 3 + i * 3] = \
                self.sensor_usb_imus[i].get_measured_acceleration()

            # ypr -> quat
            self.sensor_usb_imu_group_measured_quat[0 + i * 4: 4 + i * 4] = \
                fi_robot_tool.radian_ypr_to_quat_continuous(self.sensor_usb_imu_group_measured_angle[0 + i * 3: 3 + i * 3] * numpy.pi / 180,
                                                            self.sensor_usb_imu_group_measured_quat[0 + i * 4: 4 + i * 4])

        return FunctionResult.SUCCESS

    def control_loop_update_state_actuator(self):

        self.actuator_group.upload()

        for i in range(self.number_of_actuator):
            self.actuator_group_measured_position[i] = self.actuators[i].measured_position
            self.actuator_group_measured_velocity[i] = self.actuators[i].measured_velocity
            self.actuator_group_measured_kinetic[i] = self.actuators[i].measured_kinetic
            self.actuator_group_measured_current[i] = self.actuators[i].measured_current

        return FunctionResult.SUCCESS

    def control_loop_update_state_robot(self):

        self.joint_group.update()

        for i in range(self.number_of_joint):
            self.joint_group_measured_position[i] = self.joints[i].measured_position
            self.joint_group_measured_velocity[i] = self.joints[i].measured_velocity
            self.joint_group_measured_kinetic[i] = self.joints[i].measured_kinetic

        self.joint_group_measured_position_radian = self.joint_group_measured_position * numpy.pi / 180.0
        self.joint_group_measured_velocity_radian = self.joint_group_measured_velocity * numpy.pi / 180.0

        return FunctionResult.SUCCESS

    def control_loop_update_command(self):
        func_result = super(RobotFFTAI, self).control_loop_update_command()

        if func_result == FunctionResult.SUCCESS:
            return FunctionResult.SUCCESS
        else:
            return FunctionResult.FAIL

    def control_loop_algorithm(self):
        super(RobotFFTAI, self).control_loop_algorithm()

        return FunctionResult.SUCCESS

    def control_loop_protection_check(self):
        return FunctionResult.SUCCESS

    def control_loop_protection_do(self):
        return FunctionResult.SUCCESS

    def control_loop_output(self):

        # send command to joint
        if self.work_space == RobotWorkSpace.NONE:
            pass
        elif self.work_space == RobotWorkSpace.ACTUATOR_SPACE:
            self.actuator_group.download()
        elif self.work_space == RobotWorkSpace.JOINT_SPACE or \
                self.work_space == RobotWorkSpace.TASK_SPACE:
            self.joint_group.download()
        else:
            self.work_space = RobotWorkSpace.NONE

        return FunctionResult.SUCCESS

    def control_loop_update_communication(self):

        # update from joystick ----------------------------------------------------

        if OperatorJoystickInterface() is not None:
            OperatorJoystickInterface().instance.upload()  # 更新 joystick 的状态数据

            if OperatorJoystickInterface().instance.get_button_l1() == 1 != OperatorJoystickInterface().instance.get_button_l1_last():
                self.task_select = self.task_select + self.task_select_direction

                if self.task_select >= len(self.tasks):
                    self.task_select = 0
                elif self.task_select < 0:
                    self.task_select = len(self.tasks) - 1
                else:
                    pass

                Logger().print_trace("task select: " + str(self.tasks[self.task_select].name))
            else:
                pass

            if OperatorJoystickInterface().instance.get_button_l2() == 1 != OperatorJoystickInterface().instance.get_button_l2_last():
                self.task_command = self.tasks[self.task_select]
                self.flag_task_command_update = FlagState.SET

                Logger().print_trace("task command: " + str(self.tasks[self.task_select].name))

            # task select direction
            if OperatorJoystickInterface().instance.get_button_share() == 1 != OperatorJoystickInterface().instance.get_button_share_last():
                if self.task_select_direction == 1:
                    self.task_select_direction = -1
                elif self.task_select_direction == -1:
                    self.task_select_direction = 1
                else:
                    self.task_select_direction = 1

                print("task select direction: " + str(self.task_select_direction))

            # 快捷键
            self.control_loop_update_communication_joystick_button_logo()
            self.control_loop_update_communication_joystick_button_triangle()
            self.control_loop_update_communication_joystick_button_circle()
            self.control_loop_update_communication_joystick_button_square()
            self.control_loop_update_communication_joystick_button_cross()

            # left axis 和 right axis 同时按下，立即杀死所有任务
            if OperatorJoystickInterface().instance.get_button_axis_left() == 1 \
                    and OperatorJoystickInterface().instance.get_button_axis_right() == 1:
                CallbackSystemExit().flag = FlagState.SET
                Logger().print_trace("system exit")

        # update from joystick ----------------------------------------------------

        return FunctionResult.SUCCESS

    def control_loop_update_communication_joystick_button_logo(self):
        # logo 按钮按下立即停止
        if OperatorJoystickInterface().instance.get_button_logo() == 1:
            self.task_command = RobotBaseTasks.TASK_SERVO_OFF
            self.flag_task_command_update = FlagState.SET

            Logger().print_trace("task command: " + str(self.task_command.name))

    def control_loop_update_communication_joystick_button_triangle(self):
        pass

    def control_loop_update_communication_joystick_button_circle(self):
        pass

    def control_loop_update_communication_joystick_button_square(self):
        pass

    def control_loop_update_communication_joystick_button_cross(self):
        # x button press
        if OperatorJoystickInterface().instance.get_button_cross() == 1:
            self.task_command = RobotBaseTasks.TASK_CLEAR_FAULT
            self.flag_task_command_update = FlagState.SET

            Logger().print_trace("task command: " + str(self.task_command.name))

    def algorithm_servo_on(self):
        super(RobotFFTAI, self).algorithm_servo_on()
        return FunctionResult.SUCCESS

    def algorithm_servo_off(self):
        super(RobotFFTAI, self).algorithm_servo_off()
        return FunctionResult.SUCCESS

    def algorithm_find_home(self):
        self.work_space = RobotWorkSpace.NONE
        return FunctionResult.SUCCESS
