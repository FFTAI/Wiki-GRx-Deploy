import numpy

# 常量定义
ROBOT_JOINT_COUNT = 6 + 6 + 1 + 5 + 5  # 机器人关节总数


def parse_robot_state(state_dict):
    """
    Parse the robot state from a dictionary containing sensor and joint data.

    The function extracts various state information related to the IMU and robot
    joints. The IMU data includes orientation (quaternion and Euler angles),
    angular velocity, and linear acceleration. The joint data includes position,
    velocity, and effort (torque) for each joint in the robot.

    Parameters:
    state_dict (dict): A dictionary containing the robot's state data. It may
                       include keys for IMU and joint states such as "imu_quat",
                       "imu_euler_angle", "imu_angular_velocity",
                       "imu_acceleration", "joint_position", "joint_velocity", and
                       "joint_effort". Missing keys will be replaced with default
                       values.

    Returns:
    tuple: A tuple containing the following elements in order:
           - imu_quat (list[float]): IMU orientation as a quaternion (x, y, z, w).
           - imu_euler_angle (list[float]): IMU orientation as Euler angles (roll,
             pitch, yaw) in radians.
           - imu_angular_velocity (list[float]): Angular velocity of the IMU in
             radians per second.
           - imu_acceleration (list[float]): Linear acceleration of the IMU in
             meters per second squared.
           - joint_position (list[float]): Position of each joint in radians.
           - joint_velocity (list[float]): Velocity of each joint in radians per
             second.
           - joint_effort (list[float]): Effort (torque) applied at each joint in
             newton-meters.

    Raises:
    KeyError: If an unexpected key is encountered or if required data cannot be
              extracted due to incorrect dictionary structure.
    """
    imu_quat = state_dict.get("imu_quat", [0, 0, 0, 1])
    imu_euler_angle = state_dict.get("imu_euler_angle", [0, 0, 0])
    imu_angular_velocity = state_dict.get("imu_angular_velocity", [0, 0, 0])
    imu_acceleration = state_dict.get("imu_acceleration", [0, 0, 0])
    joint_position = state_dict.get("joint_position", [0] * ROBOT_JOINT_COUNT)
    joint_velocity = state_dict.get("joint_velocity", [0] * ROBOT_JOINT_COUNT)
    joint_effort = state_dict.get("joint_effort", [0] * ROBOT_JOINT_COUNT)

    return (
        imu_quat, imu_euler_angle, imu_angular_velocity, imu_acceleration,
        joint_position, joint_velocity, joint_effort,
    )


def print_robot_state(
        state_dict
):
    """
    Prints the state of a robot by parsing and displaying various sensor and joint data.

    The function extracts data from a dictionary representing the robot's state, formats it,
    and prints detailed information about the IMU (Inertial Measurement Unit) and joint states.
    All numerical values are rounded to three decimal places for readability.

    Parameters
    ----------
    state_dict : dict
        A dictionary containing raw data of the robot's state. This is expected to have
        specific keys corresponding to IMU and joint measurements.

    Returns
    -------
    None

    Raises
    ------
    ValueError
        If the input `state_dict` does not contain the required keys or if parsing fails.

    Notes
    -----
    This function relies on the `parse_robot_state` function to extract individual components
    such as IMU quaternion, Euler angles, angular velocity, acceleration, joint positions,
    velocities, and efforts. Ensure that `parse_robot_state` is correctly implemented and
    available in the same scope.

    The output is formatted for human readability and includes units for each measurement.
    """
    imu_quat, imu_euler_angle, imu_angular_velocity, imu_acceleration, \
        joint_position, joint_velocity, joint_effort = \
        parse_robot_state(
            state_dict
        )

    print("#################################################")
    print("IMU Quaternion: \n", numpy.round(imu_quat, 3))
    print("IMU Euler Angle (rad): \n", numpy.round(imu_euler_angle, 3))
    print("IMU Angular Velocity (rad/s): \n", numpy.round(imu_angular_velocity, 3))
    print("IMU Linear Acceleration (m/s^2): \n", numpy.round(imu_acceleration, 3))
    print("Joint Positions (rad): \n", numpy.round(joint_position, 3))
    print("Joint Velocities (rad/s): \n", numpy.round(joint_velocity, 3))
    print("Joint Efforts (Nm): \n", numpy.round(joint_effort, 3))
