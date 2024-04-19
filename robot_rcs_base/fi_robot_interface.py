from robot_rcs.logger.fi_logger import Logger
from robot_rcs.config.fi_robot_config import gl_robot_config
from robot_rcs.predefine.fi_robot_type import RobotType

from robot_rcs.robot.fi_robot_interface import RobotInterface

# from .fi_robot_gr1t1 import RobotGR1T1
from .fi_robot_gr1t1_customize import RobotGR1T1Customize as RobotGR1T1

# 根据配置文件中的机器人类型，初始化 RobotInterface
RobotInterface().type = "GR1T1"
RobotInterface().instance = RobotGR1T1()

# 如果 RobotInterface().instance 不为 None，则打印 RobotInterface().instance 的信息
if RobotInterface().instance is not None:
    RobotInterface().instance.log_info()
else:
    Logger().print_trace_warning("robot_rcs_gr.robot.fi_robot_interface.py RobotInterface().instance is None")
