from robot_rcs.tools.pd_conversion import *


def GR1T1_pd_dict() -> dict:
    pd_dict = {
        "l_hip_roll": [FSAType.FSA_TYPE_802030, 251.625, 14.72],
        "l_hip_yaw": [FSAType.FSA_TYPE_601750, 362.52, 10.0833],
        "l_hip_pitch": [FSAType.FSA_TYPE_1307E, 200, 11],
        "l_knee_pitch": [FSAType.FSA_TYPE_1307E, 200, 11],
        "l_ankle_pitch": [FSAType.FSA_TYPE_36B36E, 10.98, 0.599],
        "l_ankle_roll": [FSAType.FSA_TYPE_36B36E, 10.98, 0.599],
        "r_hip_roll": [FSAType.FSA_TYPE_802030, 251.625, 14.72],
        "r_hip_yaw": [FSAType.FSA_TYPE_601750, 362.52, 10.0833],
        "r_hip_pitch": [FSAType.FSA_TYPE_1307E, 200, 11],
        "r_knee_pitch": [FSAType.FSA_TYPE_1307E, 200, 11],
        "r_ankle_pitch": [FSAType.FSA_TYPE_36B36E, 10.98, 0.599],
        "r_ankle_roll": [FSAType.FSA_TYPE_36B36E, 10.98, 0.599],
        "waist_yaw": [FSAType.FSA_TYPE_601750, 362.52, 10.0833],
        "waist_pitch": [FSAType.FSA_TYPE_601750, 362.52, 10.0833],
        "waist_roll": [FSAType.FSA_TYPE_601750, 362.52, 10.0833],
        "head_yaw": [FSAType.FSA_TYPE_3611100, 112.06, 3.1],
        "head_pitch": [FSAType.FSA_TYPE_3611100, 112.06, 3.1],
        "head_roll": [FSAType.FSA_TYPE_3611100, 112.06, 3.1],
        "l_shoulder_pitch": [FSAType.FSA_TYPE_361480, 92.85, 2.575],
        "l_shoulder_roll": [FSAType.FSA_TYPE_361480, 92.85, 2.575],
        "l_shoulder_yaw": [FSAType.FSA_TYPE_3611100, 112.06, 3.1],
        "l_elbow_pitch": [FSAType.FSA_TYPE_3611100, 112.06, 3.1],
        "r_shoulder_pitch": [FSAType.FSA_TYPE_361480, 92.85, 2.575],
        "r_shoulder_roll": [FSAType.FSA_TYPE_361480, 92.85, 2.575],
        "r_shoulder_yaw": [FSAType.FSA_TYPE_3611100, 112.06, 3.1],
        "r_elbow_pitch": [FSAType.FSA_TYPE_3611100, 112.06, 3.1],
    }

    return pd_dict


def GR1T2_pd_dict() -> dict:
    pd_dict = {
        "l_hip_roll": [FSAType.FSA_TYPE_802030, 251.625, 14.72],
        "l_hip_yaw": [FSAType.FSA_TYPE_601750, 362.52, 10.0833],
        "l_hip_pitch": [FSAType.FSA_TYPE_1307E, 200, 11],
        "l_knee_pitch": [FSAType.FSA_TYPE_1307E, 200, 11],
        "l_ankle_pitch": [FSAType.FSA_TYPE_36B36E, 10.98, 0.6],
        "l_ankle_roll": [FSAType.FSA_TYPE_36B36E, 10.98, 0.6],
        "r_hip_roll": [FSAType.FSA_TYPE_802030, 251.625, 14.72],
        "r_hip_yaw": [FSAType.FSA_TYPE_601750, 362.52, 10.0833],
        "r_hip_pitch": [FSAType.FSA_TYPE_1307E, 200, 11],
        "r_knee_pitch": [FSAType.FSA_TYPE_1307E, 200, 11],
        "r_ankle_pitch": [FSAType.FSA_TYPE_36B36E, 10.98, 0.6],
        "r_ankle_roll": [FSAType.FSA_TYPE_36B36E, 10.98, 0.6],
        "waist_yaw": [FSAType.FSA_TYPE_601750, 362.52, 10.0833],
        "waist_pitch": [FSAType.FSA_TYPE_601750, 362.52, 10.0833],
        "waist_roll": [FSAType.FSA_TYPE_601750, 362.52, 10.0833],
        "l_shoulder_pitch": [FSAType.FSA_TYPE_361480, 92.85, 2.575],
        "l_shoulder_roll": [FSAType.FSA_TYPE_361480, 92.85, 2.575],
        "l_shoulder_yaw": [FSAType.FSA_TYPE_3611100, 112.06, 3.1],
        "l_elbow_pitch": [FSAType.FSA_TYPE_3611100, 112.06, 3.1],
        "r_shoulder_pitch": [FSAType.FSA_TYPE_361480, 92.85, 2.575],
        "r_shoulder_roll": [FSAType.FSA_TYPE_361480, 92.85, 2.575],
        "r_shoulder_yaw": [FSAType.FSA_TYPE_3611100, 112.06, 3.1],
        "r_elbow_pitch": [FSAType.FSA_TYPE_3611100, 112.06, 3.1],
    }

    return pd_dict


def GR1T1_low_stiffness_pd_dict() -> dict:
    pd_dict = {
        "l_hip_roll": [FSAType.FSA_TYPE_802030, 57, 5.7],
        "l_hip_yaw": [FSAType.FSA_TYPE_601750, 43, 4.3],
        "l_hip_pitch": [FSAType.FSA_TYPE_1307E, 114, 11.4],
        "l_knee_pitch": [FSAType.FSA_TYPE_1307E, 114, 11.4],
        "l_ankle_pitch": [FSAType.FSA_TYPE_36B36E, 15.3, 1.5],
        "l_ankle_roll": [FSAType.FSA_TYPE_36B36E, 15.3, 1.5],
        "r_hip_roll": [FSAType.FSA_TYPE_802030, 57, 5.7],
        "r_hip_yaw": [FSAType.FSA_TYPE_601750, 43, 4.3],
        "r_hip_pitch": [FSAType.FSA_TYPE_1307E, 114, 11.4],
        "r_knee_pitch": [FSAType.FSA_TYPE_1307E, 114, 11.4],
        "r_ankle_pitch": [FSAType.FSA_TYPE_36B36E, 15.3, 1.5],
        "r_ankle_roll": [FSAType.FSA_TYPE_36B36E, 15.3, 1.5],
    }

    return pd_dict


def GR1T1_dic_pd_dict() -> dict:
    pd_dict = {
        "l_hip_roll": [FSAType.FSA_TYPE_802030, 76.4, 76.4 / 10 * 2.5],
        "l_hip_yaw": [FSAType.FSA_TYPE_601750, 57.3, 57.3 / 10 * 2.5],
        "l_hip_pitch": [FSAType.FSA_TYPE_1307E, 165.5, 165.5 / 10 * 2.5],
        "l_knee_pitch": [FSAType.FSA_TYPE_1307E, 165.5, 165.5 / 10 * 2.5],
        "l_ankle_pitch": [FSAType.FSA_TYPE_36B36E, 20.4, 20.4 / 10 * 2.5],
        "l_ankle_roll": [FSAType.FSA_TYPE_36B36E, 20.4, 20.4 / 10 * 2.5],
        "r_hip_roll": [FSAType.FSA_TYPE_802030, 76.4, 76.4 / 10 * 2.5],
        "r_hip_yaw": [FSAType.FSA_TYPE_601750, 57.3, 57.3 / 10 * 2.5],
        "r_hip_pitch": [FSAType.FSA_TYPE_1307E, 165.5, 165.5 / 10 * 2.5],
        "r_knee_pitch": [FSAType.FSA_TYPE_1307E, 165.5, 165.5 / 10 * 2.5],
        "r_ankle_pitch": [FSAType.FSA_TYPE_36B36E, 20.4, 20.4 / 10 * 2.5],
        "r_ankle_roll": [FSAType.FSA_TYPE_36B36E, 20.4, 20.4 / 10 * 2.5],
    }

    return pd_dict


if __name__ == "__main__":
    pid_dict_converted = pd_conversion(GR1T1_dic_pd_dict())
    for key, value in pid_dict_converted.items():
        print(key, numpy.array(value))
