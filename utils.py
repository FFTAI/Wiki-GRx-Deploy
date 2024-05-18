from dataclasses import dataclass, asdict, replace
from enum import IntEnum, Enum
from typing import Any
from collections import namedtuple


def import_rcs(dev=True):
    # version information
    from robot_rcs.version.version import version as robot_rcs_version
    from robot_rcs_gr.version.version import version as robot_rcs_gr_version

    print("robot_rcs_version = ", robot_rcs_version)
    print("robot_rcs_gr_version = ", robot_rcs_gr_version)

    # import robot_rcs and robot_rcs_gr
    from robot_rcs.control_system.fi_control_system import ControlSystem
    from robot_rcs_gr.robot.fi_robot_interface import RobotInterface

    if dev:
        ControlSystem().dev_mode()

    return ControlSystem, RobotInterface


class JointIndex(IntEnum):
    L_LEG_0 = 0
    L_LEG_1 = 1
    L_LEG_2 = 2
    L_LEG_3 = 3
    L_LEG_4 = 4
    L_LEG_5 = 5
    R_LEG_0 = 6
    R_LEG_1 = 7
    R_LEG_2 = 8
    R_LEG_3 = 9
    R_LEG_4 = 10
    R_LEG_5 = 11
    WAIST_0 = 12
    WAIST_1 = 13
    WAIST_2 = 14
    HEAD_0 = 15
    HEAD_1 = 16
    HEAD_2 = 17
    L_ARM_0 = 18
    L_ARM_1 = 19
    L_ARM_2 = 20
    L_ARM_3 = 21
    L_ARM_4 = 22
    L_ARM_5 = 23
    L_ARM_6 = 24
    R_ARM_0 = 25
    R_ARM_1 = 26
    R_ARM_2 = 27
    R_ARM_3 = 28
    R_ARM_4 = 29
    R_ARM_5 = 30
    R_ARM_6 = 31


class ControlMode(IntEnum):
    POSITION = 4
    PD = 5


class PositionModeKey(str, Enum):
    """in position control mode: kp is position kp, kd is velocity kp"""

    POSITION_KP = "kp"
    VELOCITY_kp = "kd"


class PDModeKey(str, Enum):
    """in PD control mode: kp is position kp, kd is velocity kd"""

    KP = "kp"
    KD = "kd"


# todo
# @dataclass
# class State:
#     imu_quat: list
#     imu_euler_angle: list
#     imu_angular_velocity: list
#     imu_acceleration: list
#     joint_position: list
#     joint_velocity: list
#     joint_kinetic: list
#     base_xyz: list
#     base_vel_xyz: list


ControlItem = namedtuple("ControlItem", ["control_mode", "kp", "kd", "position"])


@dataclass
class ControlDict:
    # control mode:
    # - 4: position control
    # - 5: PD control
    control_mode: list[ControlMode] = []
    # kp, kd:
    # - in position control mode: kp is position kp, kd is velocity kp
    # - in PD control mode: kp is position kp, kd is velocity kd
    kp: list[float] = []
    kd: list[float] = []
    # position (in urdf):
    # - unit: degree
    position: list[float] = []

    def asdict(self):
        return asdict(self)

    def update(self, **kwargs):
        return replace(self, **kwargs)

    def _set_key_by_index(self, key: str, indices: list[JointIndex], values: list[Any]):
        if key == "control_mode":
            for i, v in zip(indices, values):
                self.control_mode[i] = v
        elif key == "kp":
            for i, v in zip(indices, values):
                self.kp[i] = v
        elif key == "kd":
            for i, v in zip(indices, values):
                self.kd[i] = v
        elif key == "position":
            for i, v in zip(indices, values):
                self.position[i] = v
        else:
            raise ValueError(f"key={key} is not supported")

    def set_postion(self, indices: list[JointIndex], values: list[Any]):
        """Set joint positions"""
        self._set_key_by_index("position", indices, values)
