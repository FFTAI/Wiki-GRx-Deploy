import os
import subprocess
import socket
import sys
import time
from threading import Thread

from robot_rcs.callback.fi_callback import CallbackSystemExit
from robot_rcs.control_system.fi_control_system import ControlSystem

from robot_rcs.predefine.fi_flag_state import FlagState
from robot_rcs.predefine.fi_function_result import FunctionResult
from robot_rcs.predefine.fi_control_system_work_state import ControlSystemWorkState

from robot_rcs.logger.fi_logger import Logger
from robot_rcs.config.fi_robot_config import gl_robot_config

from robot_rcs.process.fi_process_comm import process_comm_init
from robot_rcs.process.fi_process_comm import process_comm_parent_to_children_update
from robot_rcs.process.fi_process_comm import process_comm_children_to_parent_update

from robot_rcs.process.fi_process_ota import process_ota_init

from robot_rcs_base.fi_robot_interface import RobotInterface


def main(argv):
    # 调用脚本，设置 /dev/ttyUSB0 权限
    # Logger().print_trace("sudo chmod 777 /dev/ttyUSB0")
    # os.system("sudo chmod 777 /dev/ttyUSB0")

    # 初始化通信接口
    dynalinkhs_init()

    # 控制环子线程
    trcl = Thread(target=thread_robot_control, args=(1,))
    trcl.start()

    # 通信缓存更新子线程 (将机器人数据保存到通信中间缓冲区)
    tcmu = Thread(target=thread_comm_manager_update, args=(1,))
    tcmu.start()

    # 退出指令回调子线程
    tcbse = Thread(target=thread_callback_system_exit, args=(1,))
    tcbse.start()

    # 通信子进程
    if gl_robot_config.parameters["comm"]["enable"] is True:
        process_comm_init()

    # OTA子进程
    if gl_robot_config.parameters["ota"]["enable"] is True:
        process_ota_init()

    # 主进程
    while True:
        time.sleep(9999)


def dynalinkhs_init():
    pass


def thread_robot_control(args):
    Logger().print_trace("thread_robot_control start...")

    time_in_rcsos_in_s = time.time()  # 获取当前时间戳
    last_time_in_rcsos_in_s = time.time()  # 获取当前时间戳

    target_control_period_in_s = gl_robot_config.parameters["robot"]["control_period"]  # 机器人控制周期
    target_input_and_calculate_period_in_s = gl_robot_config.parameters["robot"]["input_and_calculate_period"]  # Algorithm Period

    ControlSystem().run_module_robot_control_loop_before()
    ControlSystem().time_of_robot_control_loop_input_and_calculate_target_s = target_input_and_calculate_period_in_s

    count = 0

    while True:
        if RobotInterface().instance is None:
            Logger().print_trace_error("main.py thread_robot_control The robot model has not been initialized!!!")
            break

        # control period calculation
        time_in_rcsos_in_s = time.time()
        RobotInterface().instance.control_period = time_in_rcsos_in_s - last_time_in_rcsos_in_s
        last_time_in_rcsos_in_s = time_in_rcsos_in_s

        # control loop
        time_start_of_robot_control_loop_in_s = time.time()

        ControlSystem().run_module_robot_control_loop()

        time_end_of_robot_control_loop_in_s = time.time()
        time_of_robot_control_loop_in_s = time_end_of_robot_control_loop_in_s \
                                          - time_start_of_robot_control_loop_in_s

        # Logger().print_trace("robot control period = ",
        #                       RobotInterface().instance.control_period)
        # if RobotInterface().instance.control_period > 0.0025:
        #     Logger().print_trace_error("robot control period too large: ",
        #                                 RobotInterface().instance.control_period)
        # break

        count += 1
        if (count * target_control_period_in_s) > 2:  # 2 秒打印一次
            count = 0
            Logger().print_trace("robot control period = ", RobotInterface().instance.control_period)
            Logger().print_trace("robot control elapse = ", time_of_robot_control_loop_in_s)

        # wait for next control period
        time_to_sleep_in_s = target_control_period_in_s - time_of_robot_control_loop_in_s
        if time_to_sleep_in_s >= 0:
            pass
        else:
            time_to_sleep_in_s = 0

        time_to_sleep_mark_in_s = time.time()
        while True:
            time_offset_in_s = time.time() - time_to_sleep_mark_in_s
            if time_offset_in_s >= time_to_sleep_in_s:
                break

        # handle ctrl+c
        if CallbackSystemExit().flag == FlagState.SET:
            break

    Logger().print_trace("thread_robot_control stop...")


def thread_comm_manager_update(args):
    Logger().print_trace("thread_comm_manager_update start...")

    target_comm_manager_update_period_in_s = 0.01  # 通信管理器更新周期 100Hz
    # target_comm_manager_update_period_in_s = 0.02  # 通信管理器更新周期 50Hz
    # target_comm_manager_update_period_in_s = 1  # 通信管理器更新周期 1Hz

    while True:
        # update
        time_start_of_comm_manager_update_in_s = time.time()

        process_comm_parent_to_children_update()  # 控制进程数据 -> 通信进程数据 (response)
        process_comm_children_to_parent_update()  # 通信进程数据 -> 控制进程数据 (request)

        time_end_of_comm_manager_update_in_s = time.time()
        time_of_comm_manager_update_in_s = time_end_of_comm_manager_update_in_s \
                                           - time_start_of_comm_manager_update_in_s

        # wait for next comm update period
        time_to_sleep_in_s = target_comm_manager_update_period_in_s - time_of_comm_manager_update_in_s
        if time_to_sleep_in_s >= 0:
            pass
        else:
            time_to_sleep_in_s = 0
        time.sleep(time_to_sleep_in_s)

        # handle ctrl+c
        if CallbackSystemExit().flag == FlagState.SET:
            break

    Logger().print_trace("thread_comm_manager_update stop...")


def thread_callback_system_exit(args):
    # global gl_s_raw, gl_s_json

    Logger().print_trace("thread_callback_system_exit start...")

    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            CallbackSystemExit().flag = FlagState.SET

        # print("CallbackSystemExit().flag = ", CallbackSystemExit().flag)

        if CallbackSystemExit().flag == FlagState.SET:
            # if gl_s_raw is None:
            #     pass
            # else:
            #     gl_s_raw.close()
            #
            # if gl_s_json is None:
            #     pass
            # else:
            #     gl_s_json.close()

            # 退出循环
            break

    Logger().print_trace("thread_callback_system_exit stop...")
    # Logger().print_trace("poweroff ...")
    # if CallbackSystemExit().shutdown == FlagState.SET:
    #     try:
    #         p = subprocess.Popen("sudo poweroff", shell=True, close_fds=True, stdin=subprocess.PIPE,
    #                              stdout=subprocess.PIPE)
    #         p.stdin.write("pi\n")
    #     except:
    #         Logger().print_trace_warning('poweroff')
    #         try:
    #             p.stdin.write("pi\n")
    #         except:
    #             pass
    #     else:
    #         pass
    # 退出程序
    os._exit(0)


if __name__ == "__main__":
    Logger().print_trace("main program start...")
    Logger().print_file_trace("main program start...")

    Logger().print_trace("main program input args = ", sys.argv)

    if gl_robot_config.parameters["system"] == "LINUX":
        main(sys.argv)
    else:
        Logger().print_trace_error("unsupported system platform.")

    Logger().print_trace("main program stop...")
    Logger().print_file_trace("main program stop...")
