import os
import time

import numpy
import json
from enum import Enum

from robot_rcs.logger.fi_logger import Logger
from robot_rcs.robot_config.fi_robot_config import gl_robot_config

from robot_rcs.predefine.fi_function_result import FunctionResult
from robot_rcs.predefine.fi_flag_state import FlagState
from robot_rcs.predefine.fi_robot_type import RobotType
from robot_rcs.predefine.fi_robot_work_space import RobotWorkSpace
from robot_rcs.predefine.fi_task_stage import TaskStage

from robot_rcs.operator.fi_operator_joystick_interface import OperatorJoystickInterface

from robot_rcs.sensor.fi_sensor_usb_imu_hipnuc import SensorUSBIMUHipnuc
from robot_rcs.sensor.fi_sensor_fi_fse import SensorFIFSE
from robot_rcs.actuator.fi_actuator_fi_fsa import ActuatorFIFSA
from robot_rcs.actuator.fi_actuator_fi_fsa_group import ActuatorFIFSAGroup
from robot_rcs.joint.fi_joint_rotary import JointRotary
from robot_rcs.joint.fi_joint_group import JointGroup

import robot_rcs.robot.fi_robot_tool as fi_robot_tool
from robot_rcs.robot.fi_robot_fftai import RobotFFTAI

from parallel_ankle import ParallelAnkle

from .fi_robot_gr1t1_algorithm import RobotGR1T1AlgorithmStandControlModel


class RobotGR1Tasks(Enum):
    TASK_NONE = 0x000000
    TASK_SET_HOME = 0xAAAAAA
    TASK_STAND = 0xBBBBBB


class RobotGR1T1(RobotFFTAI):

    def __init__(self):
        super(RobotGR1T1, self).__init__()

        # sensor
        self.number_of_sensor_usb_imu = 1
        self.sensor_usb_imus = []
        self.sensor_usb_imus_angle_direction = []
        self.sensor_usb_imus_angular_velocity_direction = []
        for i in range(self.number_of_sensor_usb_imu):
            self.sensor_usb_imus.append(
                SensorUSBIMUHipnuc(usb=gl_robot_config.parameters["sensor_usb_imu"]["usb"][i])
            )
            self.sensor_usb_imus_angle_direction.append(
                numpy.array(gl_robot_config.parameters["sensor_usb_imu"]["angle_direction"][i])
            )
            self.sensor_usb_imus_angular_velocity_direction.append(
                numpy.array(gl_robot_config.parameters["sensor_usb_imu"]["angular_velocity_direction"][i])
            )

        self.sensor_usb_imu_group_measured_quat = numpy.zeros(self.number_of_sensor_usb_imu * 4)
        self.sensor_usb_imu_group_measured_angle = numpy.zeros(self.number_of_sensor_usb_imu * 3)
        self.sensor_usb_imu_group_measured_angular_velocity = numpy.zeros(self.number_of_sensor_usb_imu * 3)
        self.sensor_usb_imu_group_measured_linear_acceleration = numpy.zeros(self.number_of_sensor_usb_imu * 3)

        self.number_of_sensor_fi_fse = 6 + 6 + 3
        self.sensor_fi_fse_direction = numpy.array([
            1.0, 1.0, -1.0, 1.0, 1.0, 1.0,  # left leg
            1.0, 1.0, 1.0, -1.0, -1.0, 1.0,  # right leg
            1.0, 1.0, 1.0,  # waist
        ])
        self.sensor_fi_fse_reduction_ratio = numpy.array([
            2.0, 2.77, 2.514, 1.0, 1.0, 1.0,  # left leg
            2.0, 2.77, 2.514, 1.0, 1.0, 1.0,  # right leg
            4.08, 1.0, 1.0,  # waist
        ])
        # Jason 2024-01-22:
        # If we change the installation direction of GR1T1 IMU sensor, the offset can be all set to zero.
        self.sensor_fi_fse_sensor_offset = numpy.array([
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # left leg
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # right leg
            0.0, 0.0, 0.0,  # waist
        ])
        self.sensor_fi_fse = []
        for i in range(self.number_of_sensor_fi_fse):
            self.sensor_fi_fse.append(
                SensorFIFSE(ip=gl_robot_config.parameters["sensor_abs_encoder"]["ip"][i],
                            direction=self.sensor_fi_fse_direction[i],
                            reduction_ratio=self.sensor_fi_fse_reduction_ratio[i],
                            sensor_offset=self.sensor_fi_fse_sensor_offset[i])
            )

        # actuator
        self.number_of_actuator = 6 + 6 + 3 + 3 + 7 + 7
        self.actuators = []
        for i in range(self.number_of_actuator):
            self.actuators.append(
                ActuatorFIFSA(ip=gl_robot_config.parameters["actuator"]["ip"][i])
            )

        self.actuator_group = ActuatorFIFSAGroup(self.actuators, use_fast=True)
        self.actuator_group_measured_position = [0.0] * self.number_of_actuator
        self.actuator_group_measured_velocity = [0.0] * self.number_of_actuator
        self.actuator_group_measured_kinetic = [0.0] * self.number_of_actuator
        self.actuator_group_measured_current = [0.0] * self.number_of_actuator

        # joint
        self.number_of_joint = 6 + 6 + 3 + 3 + 7 + 7
        self.joints_direction = numpy.array([
            -1.0, 1.0, 1.0, -1.0, -1.0, 1.0,  # left leg
            -1.0, 1.0, -1.0, 1.0, 1.0, -1.0,  # right leg
            -1.0, -1.0, -1.0,  # waist
            -1.0, -1.0, -1.0,  # head
            -1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0,  # left arm
            1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # right arm
        ])
        self.joint_home_position = numpy.array([
            0, 0, 0, 0, 0, 0,  # left leg
            0, 0, 0, 0, 0, 0,  # right leg
            0, 0, 0,  # waist
            0, 0, 0,  # head
            0, 0, 0, 0, 0, 0, 0,  # left arm
            0, 0, 0, 0, 0, 0, 0,  # right arm
        ]) * 180.0 / numpy.pi  # unit : degree
        self.joint_min_position = numpy.array([
            -0.09, -0.7, -1.75, -0.09, -1.05, -0.44,  # left leg(6)
            -0.09, -0.7, -1.75, -0.09, -1.05, -0.44,  # right leg(6)
            -1.05, -0.52, -0.7,  # waist(3)
            -2.71, -0.35, -0.52,  # head(3)
            -2.79, -0.57, -2.97, -2.27, -2.97, -0.61, -0.61,  # left arm(7)
            -2.79, -0.57, -2.97, -2.27, -2.97, -0.61, -0.61,  # right arm(7)
        ]) * 180.0 / numpy.pi  # unit : degree
        self.joint_max_position = numpy.array([
            0.79, 0.7, 0.7, 1.92, 0.52, 0.44,  # left leg(6)
            0.09, 0.7, 0.7, 1.92, 0.52, 0.44,  # right leg(6)
            1.05, 1.22, 0.7,  # waist(3)
            2.71, 0.35, 0.35,  # head(3)
            1.92, 3.27, 2.97, 2.27, 2.97, 0.61, 0.61,  # left arm(7)
            1.92, 0.57, 2.97, 2.27, 2.97, 0.61, 0.61,  # right arm(7)
        ]) * 180.0 / numpy.pi  # unit : degree
        self.joint_reduction_ratio = numpy.array([
            # 31, 51, 7, 7, 36, 36,  # left leg
            # 31, 51, 7, 7, 36, 36,  # right leg
            # 51, 51, 51,  # waist
            # 51, 51, 51,  # head
            # 80, 80, 100, 100, 31, 31, 31,  # left arm
            # 80, 80, 100, 100, 31, 31, 31,  # right arm

            # Jason 2024-01-19:
            # FSA 输出的是执行器末端角度，已经计算过减速比了
            1, 1, 1, 1, 1, 1,  # left leg
            1, 1, 1, 1, 1, 1,  # right leg
            1, 1, 1,  # waist
            1, 1, 1,  # head
            1, 1, 1, 1, 1, 1, 1,  # left arm
            1, 1, 1, 1, 1, 1, 1,  # right arm
        ])
        self.joint_kinematic_reduction_ratio = \
            self.joint_reduction_ratio
        self.joint_kinetic_reduction_ratio = numpy.array([
            0.121, 0.067, 0.26, 0.26, 0.06, 0.06,  # left leg
            0.121, 0.067, 0.26, 0.26, 0.06, 0.06,  # right leg
            0.238, 0.238, 0.238,  # waist
            0.238, 0.238, 0.238,  # head
            0.238, 0.238, 0.238, 0.238, 0.238, 0.238, 0.238,  # left arm
            0.238, 0.238, 0.238, 0.238, 0.238, 0.238, 0.238,  # right arm
        ]) * self.joint_reduction_ratio
        self.joint_pd_control_kp = numpy.array([
            1.04, 0.25, 0.7, 0.7, 0.1, 0.1,  # left leg(6)
            1.04, 0.25, 0.7, 0.7, 0.1, 0.1,  # right leg(6)
            0.25, 0.25, 0.25,  # waist(3)
            0.005, 0.005, 0.005,  # head(3)
            0.2, 0.2, 0.2, 0.2, 0.005, 0.005, 0.005,  # left arm(7)
            0.2, 0.2, 0.2, 0.2, 0.005, 0.005, 0.005,  # right arm(7)
        ])
        self.joint_pd_control_kd = numpy.array([
            0.04, 0.14, 0.4, 0.4, 0.005, 0.005,  # left leg(6)
            0.04, 0.14, 0.4, 0.4, 0.005, 0.005,  # right leg(6)
            0.14, 0.14, 0.14,  # waist(3)
            0.005, 0.005, 0.005,  # head(3)
            0.02, 0.02, 0.02, 0.02, 0.005, 0.005, 0.005,  # left arm(7)
            0.02, 0.02, 0.02, 0.02, 0.005, 0.005, 0.005,  # right arm(7)
        ])
        self.joints = []
        for i in range(self.number_of_joint):
            self.joints.append(
                JointRotary(actuator=self.actuators[i],
                            direction=self.joints_direction[i],
                            home_position=self.joint_home_position[i],
                            min_position=self.joint_min_position[i],
                            max_position=self.joint_max_position[i],
                            kinematic_reduction_ratio=self.joint_kinematic_reduction_ratio[i],
                            kinetic_reduction_ratio=self.joint_kinetic_reduction_ratio[i],
                            position_control_kp=self.joint_pd_control_kp[i],
                            velocity_control_kp=self.joint_pd_control_kd[i],
                            velocity_control_ki=0.0)
            )

        self.joint_group = JointGroup(self.joints, self.actuator_group)
        self.joint_group_measured_position = numpy.array([0.0] * self.number_of_joint)
        self.joint_group_measured_velocity = numpy.array([0.0] * self.number_of_joint)
        self.joint_group_measured_kinetic = numpy.array([0.0] * self.number_of_joint)

        self.joint_group_measured_position_radian = numpy.array([0.0] * self.number_of_joint)
        self.joint_group_measured_velocity_radian = numpy.array([0.0] * self.number_of_joint)

        # joint -> joint urdf
        self.parallel_ankle_left = ParallelAnkle("left")
        self.parallel_ankle_right = ParallelAnkle("right")

        self.joint_urdf_group_measured_position = numpy.array([0.0] * self.number_of_joint)
        self.joint_urdf_group_measured_velocity = numpy.array([0.0] * self.number_of_joint)
        self.joint_urdf_group_measured_kinetic = numpy.array([0.0] * self.number_of_joint)

        self.joint_urdf_group_measured_position_radian = numpy.array([0.0] * self.number_of_joint)
        self.joint_urdf_group_measured_velocity_radian = numpy.array([0.0] * self.number_of_joint)

        # link
        self.number_of_link = 0
        self.links = None

        # end effector
        self.number_of_end_effector = 0
        self.end_effectors = None

        self.end_effector_group_measured_position = numpy.array([[0.0] * 6] * self.number_of_end_effector)
        self.end_effector_group_measured_velocity = numpy.array([[0.0] * 6] * self.number_of_end_effector)
        self.end_effector_group_measured_kinetic = numpy.array([[0.0] * 6] * self.number_of_end_effector)

        # base and legs
        self.number_of_legs = 2
        self.legs_connect_position = [
            [0.3822, 0.2353, 0.0],  # left front leg
            [-0.3822, 0.2353, 0.0],  # left back leg
        ]

        # robot
        self.robot_type = RobotType.GR1

        # task
        self.tasks = [
            RobotGR1Tasks.TASK_NONE,
            RobotGR1Tasks.TASK_SET_HOME,
            RobotGR1Tasks.TASK_STAND,
        ]

        # control algorithm
        self.algorithm_stand_control_model = RobotGR1T1AlgorithmStandControlModel()

    def init(self) -> int:
        # sensor
        # 调用脚本，设置 usb(/dev/ttyUSB0) 权限
        for i in range(self.number_of_sensor_usb_imu):
            Logger().print_trace("sudo chmod 777 " + gl_robot_config.parameters["sensor_usb_imu"]["usb"][i])
            os.system("sudo chmod 777 " + gl_robot_config.parameters["sensor_usb_imu"]["usb"][i])

        for i in range(self.number_of_sensor_usb_imu):
            if self.sensor_usb_imus[i] is not None:
                self.sensor_usb_imus[i].init()

        for i in range(self.number_of_sensor_usb_imu):
            if self.sensor_usb_imus[i] is not None:
                self.sensor_usb_imus[i].comm(enable=gl_robot_config.parameters["sensor_usb_imu"]["comm_enable"][i],
                                             frequency=gl_robot_config.parameters["sensor_usb_imu"]["comm_frequency"][i])

        for i in range(self.number_of_sensor_fi_fse):
            if self.sensor_fi_fse[i] is not None:
                self.sensor_fi_fse[i].init()

        for i in range(self.number_of_sensor_fi_fse):
            if self.sensor_fi_fse[i] is not None:
                self.sensor_fi_fse[i].comm()

        # Note 2024-01-26:
        # download PD value should before init() actuator,
        # init() actuator will create child thread to handle receive message if using non-block mode.
        Logger().print_trace("RobotFFTAI init() download joint pd value")
        for i in range(self.number_of_joint):
            if self.joints[i] is not None:
                self.joints[i].download_control_pid()

        # actuator
        for i in range(self.number_of_actuator):
            if self.actuators[i] is not None:
                self.actuators[i].init()

        for i in range(self.number_of_actuator):
            if self.actuators[i] is not None:
                self.actuators[i].comm(enable=gl_robot_config.parameters["actuator"]["comm_enable"][i],
                                       block=gl_robot_config.parameters["actuator"]["comm_block"][i],
                                       use_fast=gl_robot_config.parameters["actuator"]["comm_use_fast"][i])

        return FunctionResult.SUCCESS

    def prepare(self) -> int:
        Logger().print_trace("RobotGR1 prepare()")

        # read stored abs encoder angle
        Logger().print_trace("RobotGR1 prepare() read stored abs encoder calibrate value")
        map_fi_fses_values_stored = None
        if os.path.exists(gl_robot_config.parameters["sensor_abs_encoder"]["data_path"]):
            Logger().print_trace("home_angle file exists")

            with open(gl_robot_config.parameters["sensor_abs_encoder"]["data_path"], "r") as file:
                json_fi_fses_angle_value = file.read()
                map_fi_fses_values_stored = json.loads(json_fi_fses_angle_value)

            Logger().print_trace("home_angle = \n", str(json_fi_fses_angle_value))

        else:
            Logger().print_trace_warning(gl_robot_config.parameters["sensor_abs_encoder"]["data_path"],
                                         " file not exists, please do TASK_SET_HOME first")

        # load file config
        if map_fi_fses_values_stored is not None:
            for i in range(self.number_of_sensor_fi_fse):
                if self.sensor_fi_fse[i] is not None:
                    ip = self.sensor_fi_fse[i].ip
                    sensor_offset = map_fi_fses_values_stored[ip]

                    self.sensor_fi_fse[i].sensor_offset = sensor_offset
                else:
                    pass

        # sensor
        Logger().print_trace("RobotGR1 prepare() upload fi_fse value")
        for i in range(self.number_of_sensor_fi_fse):
            if self.sensor_fi_fse[i] is not None:
                self.sensor_fi_fse[i].upload()

        # get abs encoder value offset as offset
        absencoder_angle_offset = numpy.zeros(shape=self.number_of_sensor_fi_fse)
        absencoder_radian_offset = numpy.zeros(shape=self.number_of_sensor_fi_fse)
        for i in range(self.number_of_sensor_fi_fse):
            if self.sensor_fi_fse[i] is not None:
                absencoder_angle_offset[i] = self.sensor_fi_fse[i].measured_angle
                absencoder_radian_offset[i] = self.sensor_fi_fse[i].measured_radian

        # parallel ankle
        joint_urdf_angle_offset = numpy.zeros(shape=self.number_of_joint)
        joint_urdf_radian_offset = numpy.zeros(shape=self.number_of_joint)
        joint_urdf_angle_offset[0:self.number_of_sensor_fi_fse] = \
            absencoder_angle_offset[0:self.number_of_sensor_fi_fse].copy()
        joint_urdf_radian_offset[0:self.number_of_sensor_fi_fse] = \
            absencoder_radian_offset[0:self.number_of_sensor_fi_fse].copy()

        print("joint_urdf_angle_offset111111 = \n",
              numpy.round(joint_urdf_angle_offset, 3))

        joint_urdf_radian_offset[4], \
            joint_urdf_radian_offset[5], \
            _, _, _, _ = \
            self.parallel_ankle_left.inverse(ankle_position_pitch=joint_urdf_radian_offset[4],
                                             ankle_position_roll=joint_urdf_radian_offset[5])
        joint_urdf_radian_offset[10], \
            joint_urdf_radian_offset[11], \
            _, _, _, _ = \
            self.parallel_ankle_right.inverse(ankle_position_pitch=joint_urdf_radian_offset[10],
                                              ankle_position_roll=joint_urdf_radian_offset[11])

        joint_angle_offset = joint_urdf_radian_offset / numpy.pi * 180.0

        # joint_angle_offset[6+6+0] = 0
        print("joint_angle_offset2222 = \n",
              numpy.round(joint_angle_offset, 3))

        # read current joint position
        # Jason 2024-01-26:
        # need to wait and send twice to allow data to upload
        self.actuator_group.upload()
        time.sleep(0.01)
        self.joint_group.update()
        time.sleep(0.01)
        self.actuator_group.upload()
        time.sleep(0.01)
        self.joint_group.update()

        for i in range(self.number_of_joint):
            self.joint_group_measured_position[i] = self.joints[i].measured_position
            self.joint_group_measured_velocity[i] = self.joints[i].measured_velocity
            self.joint_group_measured_kinetic[i] = self.joints[i].measured_kinetic

        print("joint_group_measured_position = \n",
              numpy.round(self.joint_group_measured_position, 3))

        joint_home_position = numpy.zeros(self.number_of_joint)
        joint_home_position[0:self.number_of_sensor_fi_fse] = \
            (-1) * (joint_angle_offset[0:self.number_of_sensor_fi_fse]
                    - self.joint_group_measured_position[0:self.number_of_sensor_fi_fse])

        print("joint_home_position = \n",
              numpy.round(joint_home_position, 3))

        # change joint home position, based on encoder measured value
        for i in range(self.number_of_joint):
            if self.joints[i] is not None:
                # use negative value to calibrate joint position
                self.joints[i].home_position = joint_home_position[i]
            else:
                pass  # use default home position are home position

        Logger().print_trace("Finish joint home position calibration !")

        return FunctionResult.SUCCESS

    def control_loop_update_state(self) -> int:
        # sensor
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

        # actuator
        self.actuator_group.upload()

        for i in range(self.number_of_actuator):
            self.actuator_group_measured_position[i] = self.actuators[i].measured_position
            self.actuator_group_measured_velocity[i] = self.actuators[i].measured_velocity
            self.actuator_group_measured_kinetic[i] = self.actuators[i].measured_kinetic
            self.actuator_group_measured_current[i] = self.actuators[i].measured_current

        # joint
        self.joint_group.update()

        for i in range(self.number_of_joint):
            self.joint_group_measured_position[i] = self.joints[i].measured_position
            self.joint_group_measured_velocity[i] = self.joints[i].measured_velocity
            self.joint_group_measured_kinetic[i] = self.joints[i].measured_kinetic

        self.joint_group_measured_position_radian = self.joint_group_measured_position * numpy.pi / 180.0
        self.joint_group_measured_velocity_radian = self.joint_group_measured_velocity * numpy.pi / 180.0

        # joint -> joint urdf
        self.joint_urdf_group_measured_position = self.joint_group_measured_position.copy()
        self.joint_urdf_group_measured_velocity = self.joint_group_measured_velocity.copy()

        self.joint_urdf_group_measured_position_radian = self.joint_urdf_group_measured_position * numpy.pi / 180.0
        self.joint_urdf_group_measured_velocity_radian = self.joint_urdf_group_measured_velocity * numpy.pi / 180.0

        # parallel ankle
        self.joint_urdf_group_measured_position_radian[4], \
            self.joint_urdf_group_measured_position_radian[5], \
            self.joint_urdf_group_measured_velocity_radian[4], \
            self.joint_urdf_group_measured_velocity_radian[5], \
            self.joint_urdf_group_measured_kinetic[4], \
            self.joint_urdf_group_measured_kinetic[5] = \
            self.parallel_ankle_left.forward(joint_position_up=self.joint_group_measured_position_radian[4],
                                             joint_position_down=self.joint_group_measured_position_radian[5],
                                             joint_velocity_up=self.joint_group_measured_velocity_radian[4],
                                             joint_velocity_down=self.joint_group_measured_velocity_radian[5],
                                             joint_torque_up=self.joint_group_measured_kinetic[4],
                                             joint_torque_down=self.joint_group_measured_kinetic[5])

        self.joint_urdf_group_measured_position_radian[10], \
            self.joint_urdf_group_measured_position_radian[11], \
            self.joint_urdf_group_measured_velocity_radian[10], \
            self.joint_urdf_group_measured_velocity_radian[11], \
            self.joint_urdf_group_measured_kinetic[10], \
            self.joint_urdf_group_measured_kinetic[11] = \
            self.parallel_ankle_right.forward(joint_position_up=self.joint_group_measured_position_radian[10],
                                              joint_position_down=self.joint_group_measured_position_radian[11],
                                              joint_velocity_up=self.joint_group_measured_velocity_radian[10],
                                              joint_velocity_down=self.joint_group_measured_velocity_radian[11],
                                              joint_torque_up=self.joint_group_measured_kinetic[10],
                                              joint_torque_down=self.joint_group_measured_kinetic[11])

        self.joint_urdf_group_measured_position = self.joint_urdf_group_measured_position_radian / numpy.pi * 180.0
        self.joint_urdf_group_measured_velocity = self.joint_urdf_group_measured_velocity_radian / numpy.pi * 180.0

        # print("self.joint_group_measured_position = \n",
        #       numpy.round(self.joint_group_measured_position, 3))
        # print("self.joint_urdf_group_measured_position = \n",
        #       numpy.round(self.joint_urdf_group_measured_position, 3))

        # print("self.joint_group_measured_position = \n",
        #       numpy.round(self.joint_group_measured_position[5], 2))
        # print("self.joint_urdf_group_measured_position = \n",
        #       numpy.round(self.joint_urdf_group_measured_position[6 + 6 + 3 + 3 + 1], 2))

        return FunctionResult.SUCCESS

    def control_loop_algorithm(self) -> int:
        if self.task_command == RobotGR1Tasks.TASK_NONE:
            pass

        elif self.task_command == RobotGR1Tasks.TASK_SET_HOME:
            self.algorithm_set_home()

        elif self.task_command == RobotGR1Tasks.TASK_STAND:
            self.algorithm_stand_control()

        else:
            pass

        return FunctionResult.SUCCESS

    def control_loop_output(self) -> int:

        # send command to joint
        if self.work_space == RobotWorkSpace.ACTUATOR_SPACE:
            self.actuator_group.download()

        if self.work_space == RobotWorkSpace.JOINT_SPACE or \
                self.work_space == RobotWorkSpace.TASK_SPACE:
            self.joint_group.download()

        return FunctionResult.SUCCESS

    def control_loop_update_communication(self) -> int:

        if OperatorJoystickInterface() is not None:
            if OperatorJoystickInterface().instance.get_button_triangle() == 1:
                pass

            if OperatorJoystickInterface().instance.get_button_circle() == 1:
                self.task_command = RobotGR1Tasks.TASK_STAND

                Logger().print_trace("task command: " + str(self.task_command.name))

            if OperatorJoystickInterface().instance.get_button_square() == 1:
                self.task_command = RobotGR1Tasks.TASK_SET_HOME

                Logger().print_trace("task command: " + str(self.task_command.name))

            if OperatorJoystickInterface().instance.get_button_cross() == 1:
                pass

        return FunctionResult.SUCCESS

    def algorithm_set_home(self):
        Logger().print_trace("algorithm_set_home")

        sensor_fi_fses_sensor_offsets = []
        map_fi_fses_angle_value = {}

        for i in range(self.number_of_sensor_fi_fse):
            if self.sensor_fi_fse[i] is not None:
                ip = self.sensor_fi_fse[i].ip
                sensor_offset = self.sensor_fi_fse[i].set_home()

                map_fi_fses_angle_value[ip] = sensor_offset

        Logger().print_trace("map_fi_fses_angle_value = \n", str(map_fi_fses_angle_value))

        # 保存 fi_fse 的角度值到文件
        Logger().print_trace("save fi_fse value to file")
        json_fi_fses_angle_value = json.dumps(map_fi_fses_angle_value, indent=4, separators=(',', ': '))

        with open(gl_robot_config.parameters["sensor_abs_encoder"]["data_path"], "w") as file:
            file.write(json_fi_fses_angle_value)

        # change task_command and task_state
        self.task_command = RobotGR1Tasks.TASK_NONE

        return FunctionResult.SUCCESS

    def algorithm_stand_control(self):

        joint_measured_position_value = self.joint_group_measured_position.copy() * numpy.pi / 180.0  # unit : rad
        joint_measured_velocity_value = self.joint_group_measured_velocity.copy() * numpy.pi / 180.0  # unit : rad/s

        work_space, control_mode, target_position = \
            self.algorithm_stand_control_model.run(
                joint_measured_position_value,
                joint_measured_velocity_value
            )

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

        target_position[4] = target_position[4] / numpy.pi * 180
        target_position[5] = target_position[5] / numpy.pi * 180
        target_position[10] = target_position[10] / numpy.pi * 180
        target_position[11] = target_position[11] / numpy.pi * 180

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
