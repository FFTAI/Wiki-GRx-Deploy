import os
import math
import numpy
import time
import serial
import serial.tools.list_ports
from threading import Thread
from multiprocessing import Process, Manager, Lock
from queue import Queue

from .fi_function_result import FunctionResult
from .fi_flag_state import FlagState

from .fi_logger import Logger

from .fi_imu_protocol import *

# 对象字典，用于存储设备的全部 IMU 信息，缓存数据，方便机器人快速获取
usb_imu_fi_shared_od = {}
usb_imu_fi_shared_manager = Manager()
usb_imu_fi_od = {}

usb_imu_read_data_period = 0.002  # update frequency 500Hz
# usb_imu_read_data_period = 0.0025  # update frequency 400Hz
# usb_imu_read_data_period = 0.005  # update frequency 200Hz
# usb_imu_read_data_period = 0.01  # update frequency 100Hz


# ---------------------------------------------------------------------------------------------------------------------


def process_usb_imu_fi_comm(usb_imu, parent_process_od_value):
    global usb_imu_fi_shared_od, usb_imu_read_data_period

    Logger().print_trace("子进程 usb_imu = ", usb_imu)
    Logger().print_trace("子进程 od_value = ", parent_process_od_value)

    usb_imu_fi_shared_od = dict(parent_process_od_value)

    # 打开串口
    try:
        usb_imu_fi_serial = serial.Serial(port=usb_imu, baudrate=921600, timeout=1.0)
    except Exception as e:
        Logger().print_trace_error("USB IMU FI ", usb_imu, " init error: ", e)
        return FunctionResult.FAIL

    if usb_imu_fi_serial is not None:
        if usb_imu_fi_serial.is_open:
            Logger().print_trace("USB IMU FI ", usb_imu, " is already opened.")
        else:
            usb_imu_fi_serial.open()
    else:
        Logger().print_trace_error("USB IMU FI ", usb_imu, " is None")
        return FunctionResult.FAIL

    # 配置缓冲区
    usb_imu_fi_binbuffer = []
    usb_imu_fi_fifobuffer = Queue()
    usb_imu_fi_report_datatype = {
        "imusol": True,
        "gwsol": True,
        "id": True,
        "acc": True,
        "gyr": True,
        "mag": False,
        "euler": True,
        "quat": True,
        "imusol_raw": False,
        "gwsol_raw": False
    }

    # 配置延时器
    time_in_read_data_in_s = time.time()
    last_time_in_read_data_in_s = time.time()
    target_read_data_period_in_s = usb_imu_read_data_period  # update frequency

    # 数据解析
    while True:
        if usb_imu_fi_shared_od.get(usb_imu).get("thread_kill_flag") == FlagState.SET:
            break

        # 记录数据采集时间
        time_in_read_data_in_s = time.time()
        read_data_period = time_in_read_data_in_s - last_time_in_read_data_in_s
        last_time_in_read_data_in_s = time_in_read_data_in_s

        # --------------------------------------------------------------------------------------------------------------
        time_start_of_read_data_in_s = time.time()

        # 读取数据
        try:
            usb_imu_fi_serial_buff_count = usb_imu_fi_serial.in_waiting
            # print("usb_imu_fi_serial_buff_count = ", usb_imu_fi_serial_buff_count)
        except Exception as e:
            Logger().print_trace_error("USB IMU FI ", usb_imu, " inWaiting error: ", e)
            break
        else:
            if usb_imu_fi_serial_buff_count > 0:
                usb_imu_fi_serial_buff = usb_imu_fi_serial.read(usb_imu_fi_serial_buff_count)
                usb_imu_fi_binbuffer.extend(usb_imu_fi_serial_buff)

                # print(usb_imu_fi_serial_buff_count)  # 测试采集数据的发送频率

                # 解析数据
                try:
                    while True:
                        # 嘗試查找完整幀,若失敗會拋出異常
                        headerpos, endpos = intercept_one_complete_frame(usb_imu_fi_binbuffer)

                        # 解析完整幀
                        extraction_information_from_frame(usb_imu_fi_binbuffer[headerpos:endpos + 1],
                                                          usb_imu_fi_fifobuffer,
                                                          usb_imu_fi_report_datatype)

                        usb_imu_fi_binbuffer = usb_imu_fi_binbuffer[endpos + 1:]

                except FIFrame_NotCompleted_Exception as NotCompleted:
                    # 接收進行中
                    pass

                except FIFrame_ErrorFrame_Exception as e:
                    print(e)
                    # 目前幀有幀頭，但是為錯誤幀，跳過錯誤幀
                    headerpos = find_frameheader(usb_imu_fi_binbuffer)
                    usb_imu_fi_binbuffer = usb_imu_fi_binbuffer[headerpos + 1:]
                    pass

                # finally:
                #     pass

            else:
                # 没有数据的时候，不要更新共享变量 od_value，减少资源消耗
                pass

        # --------------------------------------------------------------------------------------------------------------

        if usb_imu_fi_fifobuffer.empty() is False:
            usb_imu_fi_fifobuffer_value = usb_imu_fi_fifobuffer.get(block=True, timeout=1)
            # print("usb_imu_fi_fifobuffer_value = \n", usb_imu_fi_fifobuffer_value)

            # 数据覆盖
            usb_imu_fi_shared_od.get(usb_imu).update({
                "quat": [usb_imu_fi_fifobuffer_value["quat"][0]["X"],
                         usb_imu_fi_fifobuffer_value["quat"][0]["Y"],
                         usb_imu_fi_fifobuffer_value["quat"][0]["Z"],
                         usb_imu_fi_fifobuffer_value["quat"][0]["W"]],
                "angle_degree": [usb_imu_fi_fifobuffer_value["euler"][0]["Roll"],
                                 usb_imu_fi_fifobuffer_value["euler"][0]["Pitch"],
                                 usb_imu_fi_fifobuffer_value["euler"][0]["Yaw"]],
                "angular_velocity": [usb_imu_fi_fifobuffer_value["gyr"][0]["X"],
                                     usb_imu_fi_fifobuffer_value["gyr"][0]["Y"],
                                     usb_imu_fi_fifobuffer_value["gyr"][0]["Z"]],
                "acceleration": [usb_imu_fi_fifobuffer_value["acc"][0]["X"],
                                 usb_imu_fi_fifobuffer_value["acc"][0]["Y"],
                                 usb_imu_fi_fifobuffer_value["acc"][0]["Z"]],
            })

            parent_process_od_value.update({usb_imu: usb_imu_fi_shared_od.get(usb_imu)})
            # print("子进程 usb_imu_fi_shared_od = ", usb_imu_fi_shared_od, "\n")

        time_end_of_read_data_in_s = time.time()
        time_of_read_data_in_s = time_end_of_read_data_in_s - time_start_of_read_data_in_s

        # --------------------------------------------------------------------------------------------------------------

        # wait for enough time for date update, and data to be ready
        time_to_sleep_in_s = target_read_data_period_in_s - time_of_read_data_in_s
        if time_to_sleep_in_s >= 0:
            pass
        else:
            time_to_sleep_in_s = 0

        time_to_sleep_mark_in_s = time.time()
        while True:
            time_offset_in_s = time.time() - time_to_sleep_mark_in_s
            if time_offset_in_s >= time_to_sleep_in_s:
                break

        # break

        # print("read_data_period: ", read_data_period)
        # print("fi: ", numpy.round(usb_imu_fi_shared_od.get(usb_imu).get("angle_degree"), 2))

    # 关闭串口
    if usb_imu_fi_serial is not None:
        if usb_imu_fi_serial.is_open:
            usb_imu_fi_serial.close()
        else:
            Logger().print_trace("USB IMU FI ", usb_imu, " is already closed.")
    else:
        Logger().print_trace_error("USB IMU FI ", usb_imu, " is None")
        return FunctionResult.FAIL

    return FunctionResult.SUCCESS


# ---------------------------------------------------------------------------------------------------------------------


def init(usb_imu):
    global usb_imu_fi_shared_od, usb_imu_fi_od, usb_imu_fi_upload_thread

    # 初始化字典 (shared_od)
    usb_imu_fi_shared_od.update({usb_imu: {}})

    # 准备初始化数据 (shared_od)
    usb_imu_fi_shared_od.get(usb_imu).update({
        "quat": [0, 0, 0, 0],
        "angle_degree": [0, 0, 0],
        "angular_velocity": [0, 0, 0],
        "acceleration": [0, 0, 0],
    })

    # 初始化字典 (od)
    usb_imu_fi_od.update({usb_imu: {}})

    # 准备初始化数据 (od)
    usb_imu_fi_od.get(usb_imu).update({
        "quat": [0, 0, 0, 0],
        "angle_degree": [0, 0, 0],
        "angular_velocity": [0, 0, 0],
        "acceleration": [0, 0, 0],
    })

    return FunctionResult.SUCCESS


def enable(usb_imu):
    global usb_imu_fi_shared_od, usb_imu_fi_shared_manager

    usb_imu_fi_shared_od = usb_imu_fi_shared_manager.dict(usb_imu_fi_shared_od)

    # 创建子线程
    # usb_imu_fi_comm_thread = Thread(target=thread_usb_imu_fi_comm, args=(usb_imu,))
    # usb_imu_fi_comm_thread.start()
    # usb_imu_fi_shared_od.get(usb_imu).update({"thread": usb_imu_fi_comm_thread})
    # usb_imu_fi_shared_od.get(usb_imu).update({"thread_kill_flag": FlagState.CLEAR})

    # 创建子进程
    usb_imu_fi_comm_process = Process(target=process_usb_imu_fi_comm,
                                          args=(usb_imu, usb_imu_fi_shared_od))
    usb_imu_fi_comm_process.start()

    Logger().print_trace("父进程 usb_imu_fi_shared_od_ = ", usb_imu_fi_shared_od)

    value = usb_imu_fi_shared_od.get(usb_imu)
    # value.update({"process": usb_imu_fi_comm_process})  # process 不允许加入字典，用于共享内存
    value.update({"process_kill_flag": FlagState.CLEAR})
    usb_imu_fi_shared_od.update({usb_imu: value})  # multiprocessing.Manager 只允许从最外层的字典中更新数据

    # while True:
    #     time.sleep(1)
    #     print("父进程 usb_imu_fi_shared_od = ", usb_imu_fi_shared_od, "\n")

    return FunctionResult.SUCCESS


def disable(usb_imu):
    global usb_imu_fi_shared_od

    # 杀死子线程
    # usb_imu_fi_shared_od.get(usb_imu).update({"thread_kill_flag": FlagState.SET})
    # usb_imu_fi_thread = usb_imu_fi_shared_od.get(usb_imu).get("thread")
    # usb_imu_fi_thread.join()

    # 杀死子进程
    value = usb_imu_fi_shared_od.get(usb_imu)
    value.update({"process_kill_flag": FlagState.SET})
    usb_imu_fi_shared_od.update({usb_imu: value})  # multiprocessing.Manager 只允许从最外层的字典中更新数据

    return FunctionResult.SUCCESS


def upload(usb_imu):
    global usb_imu_fi_od

    temp_usb_imu_fi_shared_od_usb_imu = usb_imu_fi_shared_od.get(usb_imu).copy()

    usb_imu_fi_od.get(usb_imu).update({
        # "quat": temp_usb_imu_fi_shared_od_usb_imu.get("quat"),
        "angle_degree": temp_usb_imu_fi_shared_od_usb_imu.get("angle_degree"),
        "angular_velocity": temp_usb_imu_fi_shared_od_usb_imu.get("angular_velocity"),
        # "acceleration": temp_usb_imu_fi_shared_od_usb_imu.get("acceleration"),
    })

    return FunctionResult.SUCCESS


def get_quat(usb_imu):
    global usb_imu_fi_od

    return usb_imu_fi_od.get(usb_imu).get("quat")


def get_angle(usb_imu):
    global usb_imu_fi_od

    return usb_imu_fi_od.get(usb_imu).get("angle_degree")


def get_angular_velocity(usb_imu):
    global usb_imu_fi_od

    return usb_imu_fi_od.get(usb_imu).get("angular_velocity")


def get_acceleration(usb_imu):
    global usb_imu_fi_od

    return usb_imu_fi_od.get(usb_imu).get("acceleration")


# 测试代码
if __name__ == '__main__':

    ports = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
    Logger().print_trace('当前电脑所连接的 {} 串口设备共 {} 个: {}'.format('USB', len(ports), ports))

    usb_imu = '/dev/ttyUSB0'
    if init(usb_imu) == FunctionResult.SUCCESS:
        enable(usb_imu)

    # 配置延时器
    time_in_read_data_in_s = time.time()
    last_time_in_read_data_in_s = time.time()
    target_read_data_period_in_s = usb_imu_read_data_period  # update frequency

    while True:
        time_start_of_read_data_in_s = time.time()

        print(usb_imu_fi_shared_od.get(usb_imu).copy())

        time_end_of_read_data_in_s = time.time()
        time_of_read_data_in_s = time_end_of_read_data_in_s - time_start_of_read_data_in_s

        # wait for enough time for date update, and data to be ready
        time_to_sleep_in_s = target_read_data_period_in_s - time_of_read_data_in_s
        if time_to_sleep_in_s >= 0:
            pass
        else:
            time_to_sleep_in_s = 0

        time_to_sleep_mark_in_s = time.time()
        while True:
            time_offset_in_s = time.time() - time_to_sleep_mark_in_s
            if time_offset_in_s >= time_to_sleep_in_s:
                break

    usb = '/dev/ttyUSB0'
    usb_imu_fi_disable(usb)
