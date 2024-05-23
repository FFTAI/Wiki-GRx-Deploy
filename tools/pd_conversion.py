import numpy
from enum import Enum


class FSAType(Enum):
    FSATYPE_802030 = 0
    FSATYPE_601750 = 1
    FSATYPE_1307E = 2
    FSATYPE_36B36E = 3
    FSATYPE_361480 = 4
    FSATYPE_3611100 = 5


def cal_802030(pd_kp, pd_kd):
    gw = 21 * 30 / 360
    velocity_kp = pd_kd / (gw * 0.11 * 30 * 180 / numpy.pi)
    position_kp = pd_kp / (velocity_kp * 30 * 0.11 * 30 * 180 / numpy.pi)
    return position_kp, velocity_kp


def cal_601750(pd_kp, pd_kd):
    gw = 10 * 50 / 360
    velocity_kp = pd_kd / (gw * 0.0846 * 50 * 180 / numpy.pi)
    position_kp = pd_kp / (velocity_kp * 50 * 0.0826 * 50 * 180 / numpy.pi)
    return position_kp, velocity_kp


def cal_1307e(pd_kp, pd_kd):
    gw = 21 * 7 / 360
    velocity_kp = pd_kd / (gw * 0.255 * 7 * 180 / numpy.pi)
    position_kp = pd_kp / (velocity_kp * 7 * 0.255 * 7 * 180 / numpy.pi)
    return position_kp, velocity_kp


def cal_36b36e(pd_kp, pd_kd):
    gw = 10 * 36 / 360
    velocity_kp = pd_kd / (gw * 0.0688 * 36 * 180 / numpy.pi)
    position_kp = pd_kp / (velocity_kp * 36 * 0.0688 * 36 * 180 / numpy.pi)
    return position_kp, velocity_kp


def cal_361480(pd_kp, pd_kd):
    gw = 10 * 80 / 360
    velocity_kp = pd_kd / (gw * 0.067 * 80 * 180 / numpy.pi)
    position_kp = pd_kp / (velocity_kp * 80 * 0.067 * 80 * 180 / numpy.pi)
    return position_kp, velocity_kp


def cal_3611100(pd_kp, pd_kd):
    gw = 10 * 100 / 360
    velocity_kp = pd_kd / (gw * 0.05 * 100 * 180 / numpy.pi)
    position_kp = pd_kp / (velocity_kp * 100 * 0.05 * 100 * 180 / numpy.pi)
    return position_kp, velocity_kp


def pd_conversion(pd_dict: dict) -> dict:
    pid_dict_converted = {}

    for key, value in pd_dict.items():
        if value[0] == FSAType.FSATYPE_802030:
            pid_dict_converted[key] = cal_802030(value[1], value[2])
        elif value[0] == FSAType.FSATYPE_601750:
            pid_dict_converted[key] = cal_601750(value[1], value[2])
        elif value[0] == FSAType.FSATYPE_1307E:
            pid_dict_converted[key] = cal_1307e(value[1], value[2])
        elif value[0] == FSAType.FSATYPE_36B36E:
            pid_dict_converted[key] = cal_36b36e(value[1], value[2])
        elif value[0] == FSAType.FSATYPE_361480:
            pid_dict_converted[key] = cal_361480(value[1], value[2])
        elif value[0] == FSAType.FSATYPE_3611100:
            pid_dict_converted[key] = cal_3611100(value[1], value[2])

    return pid_dict_converted


def GR1T1_pd_dict() -> dict:
    pd_dict = {
        "l_hip_roll": [FSAType.FSATYPE_802030, 251.625, 14.72],
        "l_hip_yaw": [FSAType.FSATYPE_601750, 362.52, 10.0833],
        "l_hip_pitch": [FSAType.FSATYPE_1307E, 200, 11],
        "l_knee_pitch": [FSAType.FSATYPE_1307E, 200, 11],
        "l_ankle_pitch": [FSAType.FSATYPE_36B36E, 10.98, 0.6],
        "l_ankle_roll": [FSAType.FSATYPE_36B36E, 10.98, 0.6],
        "r_hip_roll": [FSAType.FSATYPE_802030, 251.625, 14.72],
        "r_hip_yaw": [FSAType.FSATYPE_601750, 362.52, 10.0833],
        "r_hip_pitch": [FSAType.FSATYPE_1307E, 200, 11],
        "r_knee_pitch": [FSAType.FSATYPE_1307E, 200, 11],
        "r_ankle_pitch": [FSAType.FSATYPE_36B36E, 10.98, 0.6],
        "r_ankle_roll": [FSAType.FSATYPE_36B36E, 10.98, 0.6],
        "waist_yaw": [FSAType.FSATYPE_601750, 362.52, 10.0833],
        "waist_pitch": [FSAType.FSATYPE_601750, 362.52, 10.0833],
        "waist_roll": [FSAType.FSATYPE_601750, 362.52, 10.0833],
        "l_shoulder_pitch": [FSAType.FSATYPE_361480, 92.85, 2.575],
        "l_shoulder_roll": [FSAType.FSATYPE_361480, 92.85, 2.575],
        "l_shoulder_yaw": [FSAType.FSATYPE_3611100, 112.06, 3.1],
        "l_elbow_pitch": [FSAType.FSATYPE_3611100, 112.06, 3.1],
        "r_shoulder_pitch": [FSAType.FSATYPE_361480, 92.85, 2.575],
        "r_shoulder_roll": [FSAType.FSATYPE_361480, 92.85, 2.575],
        "r_shoulder_yaw": [FSAType.FSATYPE_3611100, 112.06, 3.1],
        "r_elbow_pitch": [FSAType.FSATYPE_3611100, 112.06, 3.1],
    }

    return pd_dict


def GR1T2_pd_dict() -> dict:
    pd_dict = {
        "l_hip_roll": [FSAType.FSATYPE_802030, 251.625, 14.72],
        "l_hip_yaw": [FSAType.FSATYPE_601750, 362.52, 10.0833],
        "l_hip_pitch": [FSAType.FSATYPE_1307E, 200, 11],
        "l_knee_pitch": [FSAType.FSATYPE_1307E, 200, 11],
        "l_ankle_pitch": [FSAType.FSATYPE_36B36E, 10.98, 0.6],
        "l_ankle_roll": [FSAType.FSATYPE_36B36E, 10.98, 0.6],
        "r_hip_roll": [FSAType.FSATYPE_802030, 251.625, 14.72],
        "r_hip_yaw": [FSAType.FSATYPE_601750, 362.52, 10.0833],
        "r_hip_pitch": [FSAType.FSATYPE_1307E, 200, 11],
        "r_knee_pitch": [FSAType.FSATYPE_1307E, 200, 11],
        "r_ankle_pitch": [FSAType.FSATYPE_36B36E, 10.98, 0.6],
        "r_ankle_roll": [FSAType.FSATYPE_36B36E, 10.98, 0.6],
        "waist_yaw": [FSAType.FSATYPE_601750, 362.52, 10.0833],
        "waist_pitch": [FSAType.FSATYPE_601750, 362.52, 10.0833],
        "waist_roll": [FSAType.FSATYPE_601750, 362.52, 10.0833],
        "l_shoulder_pitch": [FSAType.FSATYPE_361480, 92.85, 2.575],
        "l_shoulder_roll": [FSAType.FSATYPE_361480, 92.85, 2.575],
        "l_shoulder_yaw": [FSAType.FSATYPE_3611100, 112.06, 3.1],
        "l_elbow_pitch": [FSAType.FSATYPE_3611100, 112.06, 3.1],
        "r_shoulder_pitch": [FSAType.FSATYPE_361480, 92.85, 2.575],
        "r_shoulder_roll": [FSAType.FSATYPE_361480, 92.85, 2.575],
        "r_shoulder_yaw": [FSAType.FSATYPE_3611100, 112.06, 3.1],
        "r_elbow_pitch": [FSAType.FSATYPE_3611100, 112.06, 3.1],
    }

    return pd_dict


def GR1T1_soft_pd_dict() -> dict:
    pd_dict = {
        "l_hip_roll": [FSAType.FSATYPE_802030, 57, 5.7],
        "l_hip_yaw": [FSAType.FSATYPE_601750, 43, 4.3],
        "l_hip_pitch": [FSAType.FSATYPE_1307E, 114, 11.4],
        "l_knee_pitch": [FSAType.FSATYPE_1307E, 114, 11.4],
        "l_ankle_pitch": [FSAType.FSATYPE_36B36E, 15.3, 1.5],
        "l_ankle_roll": [FSAType.FSATYPE_36B36E, 15.3, 1.5],
        "r_hip_roll": [FSAType.FSATYPE_802030, 57, 5.7],
        "r_hip_yaw": [FSAType.FSATYPE_601750, 43, 4.3],
        "r_hip_pitch": [FSAType.FSATYPE_1307E, 114, 11.4],
        "r_knee_pitch": [FSAType.FSATYPE_1307E, 114, 11.4],
        "r_ankle_pitch": [FSAType.FSATYPE_36B36E, 15.3, 1.5],
        "r_ankle_roll": [FSAType.FSATYPE_36B36E, 15.3, 1.5],
    }

    return pd_dict


if __name__ == "__main__":
    pid_dict_converted = pd_conversion(GR1T1_soft_pd_dict())
    for key, value in pid_dict_converted.items():
        print(key, numpy.array(value))
