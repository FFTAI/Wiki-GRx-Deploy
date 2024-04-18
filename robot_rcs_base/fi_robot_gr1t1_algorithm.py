import numpy

from robot_rcs.logger.fi_logger import Logger
from robot_rcs.predefine.fi_robot_work_space import RobotWorkSpace
from robot_rcs.predefine.fi_task_stage import TaskStage
from robot_rcs.predefine.fi_actuator_control_mode import ActuatorControlMode
from robot_rcs.predefine.fi_joint_control_mode import JointControlMode


class RobotGR1T1AlgorithmStandControlModel:
    def __init__(self):
        # robot model
        self.num_of_joints = 6 + 6 + 3 + 3 + 7 + 7
        self.joint_default_position = numpy.array([
            0.0, 0.0, -0.2618, 0.5236, -0.2618, 0.0,  # left leg (6)
            0.0, 0.0, -0.2618, 0.5236, -0.2618, 0.0,  # right leg (6)
            0.0, 0.0, 0.0,  # waist (3)
            0.0, 0.0, 0.0,  # head (3)
            0.0, 0.2, 0.0, -0.3, 0.0, 0.0, 0.0,  # left arm (7)
            0.0, -0.2, 0.0, -0.3, 0.0, 0.0, 0.0,  # right arm (7)
        ])

        self.target_velocity = 0.0
        self.target_direction = 0.0

        self.variable_stage = 0

        self.variable_joint_start_position = numpy.zeros(self.num_of_joints)
        self.variable_joint_target_position = numpy.zeros(self.num_of_joints)

        self.variable_transform_move_count = 0
        self.variable_transform_move_count_max = 200
        self.variable_transform_move_ratio = 0

        self.output_work_space = RobotWorkSpace.NONE

        self.output_actuator_control_mode = numpy.zeros(self.num_of_joints)
        self.output_actuator_position = numpy.zeros(self.num_of_joints)
        self.output_actuator_velocity = numpy.zeros(self.num_of_joints)
        self.output_actuator_kinetic = numpy.zeros(self.num_of_joints)

        self.output_joint_control_mode = numpy.zeros(self.num_of_joints)
        self.output_joint_position = numpy.zeros(self.num_of_joints)
        self.output_joint_velocity = numpy.zeros(self.num_of_joints)
        self.output_joint_kinetic = numpy.zeros(self.num_of_joints)

    def run(self,
            joint_measured_position_value,  # rad
            joint_measured_velocity_value,  # rad/s
            ):

        # use measured position as default pd target
        work_space = RobotWorkSpace.NONE
        control_mode = 0
        joint_pd_control_target = joint_measured_position_value

        if self.variable_stage == TaskStage.STAGE_0:

            self.variable_transform_move_ratio = 0
            self.variable_transform_move_count = 0

            self.variable_joint_start_position = joint_measured_position_value

            self.variable_joint_target_position = numpy.array([
                0.0, 0.0, -0.2618, 0.5236, -0.2618, 0.0,  # left leg (6)
                0.0, 0.0, -0.2618, 0.5236, -0.2618, 0.0,  # right leg (6)
                0.0, 0.0, 0.0,  # waist (3)
                0.0, 0.0, 0.0,  # head (3)
                0.0, 0.2, 0.0, -0.3, 0.0, 0.0, 0.0,  # left arm (7)
                0.0, -0.2, 0.0, -0.3, 0.0, 0.0, 0.0,  # right arm (7)
            ])

            self.output_work_space = RobotWorkSpace.ACTUATOR_SPACE
            self.output_actuator_control_mode = ActuatorControlMode.SERVO_ON * numpy.ones(self.num_of_joints)

            # output
            work_space = self.output_work_space
            control_mode = self.output_actuator_control_mode

            self.variable_stage = TaskStage.STAGE_1

        elif self.variable_stage == TaskStage.STAGE_1:

            self.variable_transform_move_ratio = \
                self.variable_transform_move_count / \
                self.variable_transform_move_count_max

            self.output_work_space = RobotWorkSpace.JOINT_SPACE
            self.output_joint_control_mode = numpy.ones(self.num_of_joints) * JointControlMode.POSITION
            self.output_joint_position = numpy.zeros(self.num_of_joints)
            self.output_joint_velocity = numpy.zeros(self.num_of_joints)
            self.output_joint_kinetic = numpy.zeros(self.num_of_joints)

            self.output_joint_position = \
                self.variable_joint_start_position \
                + (self.variable_joint_target_position - self.variable_joint_start_position) \
                * self.variable_transform_move_ratio

            # add count
            self.variable_transform_move_count += 1

            if self.variable_transform_move_count >= self.variable_transform_move_count_max:
                self.variable_transform_move_count = 0
                self.variable_stage = TaskStage.STAGE_2

            Logger().print_trace(
                "transform_move_ratio = ",
                round(self.variable_transform_move_ratio * 100, 1),
                "%")

            # output
            work_space = self.output_work_space
            control_mode = self.output_joint_control_mode
            joint_pd_control_target = self.output_joint_position

        elif self.variable_stage == TaskStage.STAGE_2:

            self.output_work_space = RobotWorkSpace.JOINT_SPACE
            self.output_joint_control_mode = numpy.ones(self.num_of_joints) * JointControlMode.POSITION
            self.output_joint_position = self.variable_joint_target_position
            self.output_joint_velocity = numpy.zeros(self.num_of_joints)

            # output
            work_space = self.output_work_space
            control_mode = self.output_joint_control_mode
            joint_pd_control_target = self.output_joint_position

        else:
            self.variable_stage = TaskStage.STAGE_1

            # output
            work_space = self.output_work_space
            control_mode = self.output_joint_control_mode
            joint_pd_control_target = self.output_joint_position

        return work_space, control_mode, joint_pd_control_target
