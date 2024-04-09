import socket
import time
import json
import threading

import numpy
from enum import Enum
from math import *
import struct

from .fi_logger import Logger

# if add to Robot-RCS project, uncomment this line!!!
from .fi_fsa_predefine import (FSAFunctionResult,
                                                 FSAControlWord,
                                                 FSAErrorCode,
                                                 FSAModeOfOperation,
                                                 FSAFlagState)


class FSA:
    def __init__(self, ip="192.168.137.100"):
        self.ip = ip

        self.update_send_time = 0
        self.update_receive_time = 0
        self.send_frame_buffer = []
        self.receive_frame_buffer = []

        self.flag_loss_connection = 0

        self.measured_position = 0
        self.measured_velocity = 0
        self.measured_torque = 0
        self.measured_current = 0

        self.command_mode_of_operation = 0
        self.command_position = 0
        self.command_velocity = 0
        self.command_torque = 0
        self.command_current = 0

        self.status_word = 0
        self.error_code = 0

        self.socket = 0
        self.comm_enable = False
        self.comm_block = False
        self.send_thread = None
        self.receive_thread = None

    def add_send_frame(self, port, data):
        dict_frame = {"port": port, "data": data}

        self.send_frame_buffer.append(dict_frame)


fsa_map = {}
fsa_timeout_time = 0.01  # unit: s
fsa_port_ctrl = 2333
fsa_port_comm = 2334
fsa_port_fast = 2335
fsa_network = "192.168.137.255"
fsa_flag_debug = False
fsa_flag_enable_send_thread = False
fsa_flag_enable_receive_thread = False
fsa_loss_connection_time = 1.0  # unit: s

fsa_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
fsa_socket.settimeout(fsa_timeout_time)
fsa_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

Logger().print_trace("fi_fsa start listening for broadcast...")

# ---------------------------------------------------------------------------------------------------------------------

fsa_flag_send_thread_handle_inited = False
fsa_flag_receive_thread_handle_inited = False


def handle_send():
    Logger().print_trace("fi_fsa handle_send() start...")
    global fsa_socket, fsa_map

    for ip in fsa_map.keys():
        fsa_map[ip].update_send_time = time.time()

    while True:
        for ip in fsa_map.keys():
            fsa: FSA = fsa_map[ip]

            if len(fsa.send_frame_buffer) > 0:
                # Jason 2024-01-27:
                # send to one ip one frame at one send loop
                # in case send too many data at once cause the actuator stack overflow
                send_frame = fsa.send_frame_buffer[0]
                send_frame_port = send_frame["port"]
                send_frame_data = send_frame["data"]

                json_str = json.dumps(send_frame_data)

                try:
                    fsa_socket.sendto(str.encode(json_str), (ip, send_frame_port))

                    fsa.send_frame_buffer.pop(0)

                except socket.timeout:
                    Logger().print_trace_warning("fsa.handle_send() timeout")

                except Exception as e:
                    Logger().print_trace_warning("fsa.handle_send() sendto except: \n", e)

    Logger().print_trace("fi_fsa handle_send() end...")


def handle_receive():
    Logger().print_trace("fi_fsa handle_receive() start...")
    global fsa_socket, fsa_map

    for ip in fsa_map.keys():
        fsa_map[ip].update_receive_time = time.time()

    while True:
        try:
            data, address = fsa_socket.recvfrom(1024)
            recv_ip, recv_port = address

            if recv_port == fsa_port_ctrl:
                json_obj = json.loads(data.decode("utf-8"))

                fsa: FSA = fsa_map[recv_ip]
                fsa.update_receive_time = time.time()

                if json_obj.get("position") is not None:
                    fsa.measured_position = json_obj.get("position")

                if json_obj.get("velocity") is not None:
                    fsa.measured_velocity = json_obj.get("velocity")

                if json_obj.get("torque") is not None:
                    fsa.measured_torque = json_obj.get("torque")

                if json_obj.get("current") is not None:
                    fsa.measured_current = json_obj.get("current")

                if json_obj.get("status_word") is not None:
                    fsa.status_word = json_obj.get("status_word")

                if json_obj.get("error_code") is not None:
                    fsa.error_code = json_obj.get("error_code")

            elif recv_port == fsa_port_comm:
                pass

            elif recv_port == fsa_port_fast:
                pass

        except socket.timeout:
            # Jason 2024-01-27:
            # do not handle receive timeout
            pass

        except Exception as e:
            Logger().print_trace_warning("fi_fsa handle_receive() except: \n", e)

    Logger().print_trace("fi_fsa handle_receive() end...")


# ---------------------------------------------------------------------------------------------------------------------

def init(server_ip):
    Logger().print_trace("fi_fsa ", server_ip, " init()")

    global fsa_map
    fsa_map[server_ip] = FSA(server_ip)

    return FSAFunctionResult.SUCCESS


def comm(server_ip, enable=True, block=True):
    fsa: FSA = fsa_map[server_ip]
    fsa.comm_enable = enable
    fsa.comm_block = block

    global fsa_flag_enable_send_thread, fsa_flag_enable_receive_thread
    if fsa_flag_enable_send_thread is True:
        start_send_thread()

    if fsa_flag_enable_receive_thread is True:
        start_receive_thread()

    return FSAFunctionResult.SUCCESS


def start_send_thread():
    global fsa_flag_send_thread_handle_inited
    if fsa_flag_send_thread_handle_inited is False:
        threading.Thread(target=handle_send, args=()).start()
        fsa_flag_send_thread_handle_inited = True

    return FSAFunctionResult.SUCCESS


def start_receive_thread():
    global fsa_flag_receive_thread_handle_inited
    if fsa_flag_receive_thread_handle_inited is False:
        threading.Thread(target=handle_receive, args=()).start()
        fsa_flag_receive_thread_handle_inited = True

    return FSAFunctionResult.SUCCESS


# ---------------------------------------------------------------------------------------------------------------------
# Control Parameters of FSA

# fsa Get root attributes
# Parameters: including device IP
# Get all basic attributes of fsa, including serial number, bus voltage, motor temperature, inverter temperature, version number
def get_root(server_ip):
    data = {
        "method": "GET",
        "reqTarget": "/",
        "property": ""
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.get_root() except")


# fsa enable
# Parameters: including device IP and motor number
# Each fsa can control two motors, M0 and M1
def set_enable(server_ip):
    data = {
        "method": "SET",
        "reqTarget": "/control_word",
        "property": "",
        "control_word": FSAControlWord.SERVO_ON,
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))

    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip + " : Recv Data Error !")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.enable() except")
        return None


# fsa Disable
# Parameters: including device IP and motor number
# Each fsa can control two motors, M0 and M1
def set_disable(server_ip):
    data = {
        "method": "SET",
        "reqTarget": "/control_word",
        "property": "",
        "control_word": FSAControlWord.SERVO_OFF,
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))

    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip + " : Recv Data Error !")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.disable() except")
        return None


# CALIBRATE_ENCODER = 0xA3

# fsa Calibrate Encoder
# Parameters: including device IP and motor number
# Auto rotate clockwise once ,then change direction turn around
def set_calibrate_encoder(server_ip):
    data = {
        "method": "SET",
        "reqTarget": "/control_word",
        "property": "",
        "control_word": FSAControlWord.CALIBRATE_ENCODER,
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))

    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip + " : Recv Data Error !")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fi_fsa.set_calibrate_motor() except")
        return None


# fsa Clear Fault
# Parameters: including device IP and motor number
# Clear Fault
def clear_fault(server_ip):
    data = {
        "method": "SET",
        "reqTarget": "/control_word",
        "property": "",
        "control_word": FSAControlWord.CLEAR_FAULT,
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))

    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip + " : Recv Data Error !")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fi_fsa.set_clear_fault() except")
        return None


# fsa Get current status
# Parameters: including device IP
# Get fsa Get current status
def get_state(server_ip):
    data = {
        "method": "GET",
        "reqTarget": "/state",
        "property": ""
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))

    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return json_obj.get("state")

        else:
            Logger().print_trace_error(server_ip + " : Recv Data Error !")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.get_state() except")
        return None


# fsa set control mode
# Parameters: including server ip，motor number
# no return code
def set_mode_of_operation(server_ip, mode_of_operation):
    data = {
        "method": "SET",
        "reqTarget": "/mode_of_operation",
        "mode_of_operation": mode_of_operation,
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))

    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip + " : Recv Data Error !")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.control_mode() except")
        return None


def get_home_offset(server_ip):
    data = {
        "method": "GET",
        "reqTarget": "/home_offset",
        "property": ""
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.get_root_config() except")
        return None


# fsa reset linear count
# Parameters: including server ip，motor number
# no return code
def set_home_offset(server_ip, home_offset):
    data = {
        "method": "SET",
        "reqTarget": "/home_offset",
        "home_offset": home_offset,
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.set_linear_count() except")
        return None


def set_home_position(server_ip):
    data = {
        "method": "SET",
        "reqTarget": "/home_position",
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.set_linear_count() except")
        return None


# fsa Get Root Config property
# Parameters: including device IP
# Get fsa bus voltage over-voltage and under-voltage protection threshold
def get_pid_param(server_ip):
    data = {
        "method": "GET",
        "reqTarget": "/pid_param",
        "property": ""
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.get_root_config() except")
        return None


# fsa set Root Config properties
# Parameter: The protection threshold of bus voltage overvoltage and undervoltage
# Return success or failure
def set_pid_param(server_ip,
                  control_position_kp,
                  control_velocity_kp,
                  control_velocity_ki):
    data = {
        "method": "SET",
        "reqTarget": "/pid_param",
        "property": "",
        "control_position_kp": control_position_kp,
        "control_velocity_kp": control_velocity_kp,
        "control_velocity_ki": control_velocity_ki,
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.set_root_config() except")
        return None


def clear_pid_param(server_ip):
    data = {"method": "SET",
            "reqTarget": "/pid_param",
            "property": "clear",
            }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.set_root_config() except")
        return None


# fsa Get Root Config property
# Parameters: including device IP
# Get fsa bus voltage over-voltage and under-voltage protection threshold
def get_pid_param_imm(server_ip):
    data = {
        "method": "GET",
        "reqTarget": "/pid_param_imm",
        "property": ""
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fi_fsa.get_root_config() except")
        return None


# fsa set Root Config properties
# Parameter: The protection threshold of bus voltage overvoltage and undervoltage
# Return success or failure
def set_pid_param_imm(server_ip,
                      control_position_kp,
                      control_velocity_kp,
                      control_velocity_ki):
    data = {
        "method": "SET",
        "reqTarget": "/pid_param_imm",
        "property": "",
        "control_position_kp_imm": control_position_kp,
        "control_velocity_kp_imm": control_velocity_kp,
        "control_velocity_ki_imm": control_velocity_ki,
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))

    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " set_pid_param_imm() receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " set_pid_param_imm() Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " set_pid_param_imm() except")
        return None


# fsa Get Root Config property
# Parameters: including device IP
# Get fsa bus voltage over-voltage and under-voltage protection threshold
def get_control_param(server_ip):
    data = {
        "method": "GET",
        "reqTarget": "/control_param",
        "property": ""
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fi_fsa.get_control_param() except")
        return None


# fsa set Control Config properties
# Parameter: Set Motor Max Speed ,acceleration and current
# Return success or failure
def set_control_param(server_ip,
                      motor_max_speed,
                      motor_max_acceleration,
                      motor_max_current):
    data = {
        "method": "SET",
        "reqTarget": "/control_param",
        "property": "",
        "motor_max_speed": motor_max_speed,
        "motor_max_acceleration": motor_max_acceleration,
        "motor_max_current": motor_max_current,
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fi_fsa.set_control_param() except")
        return None


# fsa get Control Config properties
# Parameters: including device IP
# Get fsa bus voltage over-voltage and under-voltage protection threshold
def get_control_param_imm(server_ip):
    data = {
        "method": "GET",
        "reqTarget": "/control_param_imm",
        "property": ""
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fi_fsa.get_root_config() except")
        return None


# fsa set Control Config properties
# Parameter: Set Motor Max Speed ,acceleration and current
# Return success or failure
def set_control_param_imm(server_ip,
                          motor_max_speed,
                          motor_max_acceleration,
                          motor_max_current):
    data = {
        "method": "SET",
        "reqTarget": "/control_param_imm",
        "property": "",
        "motor_max_speed_imm": motor_max_speed,
        "motor_max_acceleration_imm": motor_max_acceleration,
        "motor_max_current_imm": motor_max_current,
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fi_fsa.set_control_param() except")
        return None


def get_flag_of_operation(server_ip):
    data = {
        "method": "GET",
        "reqTarget": "/flag_of_operation",
        "property": ""
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.get_root_config() except")
        return None


def set_flag_of_operation(server_ip,
                          flag_do_use_store_actuator_param,
                          flag_do_use_store_motor_param,
                          flag_do_use_store_encoder_param,
                          flag_do_use_store_pid_param):
    data = {
        "method": "SET",
        "reqTarget": "/flag_of_operation",
        "property": "",
        "flag_do_use_store_actuator_param": flag_do_use_store_actuator_param,
        "flag_do_use_store_motor_param": flag_do_use_store_motor_param,
        "flag_do_use_store_encoder_param": flag_do_use_store_encoder_param,
        "flag_do_use_store_pid_param": flag_do_use_store_pid_param,
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.set_root_config() except")
        return None


def clear_flag_of_operation(server_ip):
    data = {"method": "SET",
            "reqTarget": "/flag_of_operation",
            "property": "clear",
            }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.set_root_config() except")
        return None


# fsa Get Root Config property
# Parameters: including device IP
# Get fsa bus voltage over-voltage and under-voltage protection threshold
def get_config(server_ip):
    data = {
        "method": "GET",
        "reqTarget": "/config",
        "property": ""
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fi_fsa.get_root_config() except")
        return None


# fsa set Root Config properties
# Parameter: The protection threshold of bus voltage overvoltage and undervoltage
# Return success or failure
def set_config(server_ip, dict):
    data = {"method": "SET",
            "reqTarget": "/config",
            "property": "",

            "actuator_type": dict["actuator_type"],
            "actuator_direction": dict["actuator_direction"],
            "actuator_reduction_ratio": dict["actuator_reduction_ratio"],

            "motor_type": dict["motor_type"],
            "motor_hardware_type": dict["motor_hardware_type"],
            "motor_vbus": dict["motor_vbus"],
            "motor_direction": dict["motor_direction"],
            "motor_pole_pairs": dict["motor_pole_pairs"],
            "motor_max_speed": dict["motor_max_speed"],
            "motor_max_acceleration": dict["motor_max_acceleration"],
            "motor_max_current": dict["motor_max_current"],

            "encoder_direction": dict["encoder_direction"],
            }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fi_fsa.set_config() except")
        return None


# fsa save configuration
# Parameters: including device IP
def save_config(server_ip):
    data = {
        "method": "SET",
        "reqTarget": "/config",
        "property": "save"
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fi_fsa.save_config() except")
        return None


# fsa clear configuration
# Parameters: including device IP
def erase_config(server_ip):
    data = {
        "method": "SET",
        "reqTarget": "/config",
        "property": "erase"
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fi_fsa.erase_config() except")
        return None


# fsa restart the motor drive
# Parameters: including device IP
def reboot(server_ip):
    data = {
        "method": "SET",
        "reqTarget": "/reboot",
        "property": ""
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fi_fsa.reboot_motor_drive() except")
        return None


# fsa Get error code
# Parameters: including server IP
def get_error_code(server_ip):
    data = {
        "method": "GET",
        "reqTarget": "/error_code",
        "property": ""
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            temp = json_obj.get("error_code")
            count = 0
            for i in FSAErrorCode:
                temp_error_code = (temp << 1 >> count) & 0x01
                if temp_error_code == 1:
                    Logger().print_trace("Now Error Type = ", i.name)
                count = count + 1
            return json_obj.get("error_code")
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fi_fsa.get_error() except")
        return None


# fsa Get actuator position, velocity, current
# Parameters: including server ip，motor number
# Return position, speed, current in tuple
def get_pvc(server_ip):
    data = {
        "method": "GET",
        "reqTarget": "/measured",
        "position": True,
        "velocity": True,
        "current": True,
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return json_obj.get("position"), json_obj.get("velocity"), json_obj.get("current")
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return FSAFunctionResult.FAIL

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return FSAFunctionResult.TIMEOUT

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fi_fsa.get_pvc() except")
        return FSAFunctionResult.FAIL


def get_pvcc(server_ip):
    data = {
        "method": "GET",
        "reqTarget": "/measured",
        "position": True,
        "velocity": True,
        "current": True,
        "current_id": True,
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return json_obj.get("position"), json_obj.get("velocity"), json_obj.get("current"), json_obj.get(
                "current_id")
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return FSAFunctionResult.FAIL

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return FSAFunctionResult.TIMEOUT

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fi_fsa.get_pvc() except")
        return FSAFunctionResult.FAIL


def get_pvcccc(server_ip):
    data = {
        "method": "GET",
        "reqTarget": "/measured",
        "position": True,
        "velocity": True,
        "current": True,
        "current_id": True,
        "phase_current_ib": True,
        "phase_current_ic": True,
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return json_obj.get("position"), json_obj.get("velocity"), json_obj.get("current"), \
                json_obj.get("current_id"), json_obj.get("phase_current_ib"), json_obj.get("phase_current_ic")
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return FSAFunctionResult.FAIL

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return FSAFunctionResult.TIMEOUT

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fi_fsa.get_pvc() except")
        return FSAFunctionResult.FAIL


# fsa position control
# parameter: server IP["xxx.xxx.xxx.xxx"], position[deg], velocity feedforward[deg/s], current feedforward[A]
# return position, velocity, current
def set_position_control(server_ip, position, velocity_ff=0.0, current_ff=0.0):
    data = {
        "method": "SET",
        "reqTarget": "/position_control",
        "reply_enable": True,
        "position": position,
        "velocity_ff": velocity_ff,
        "current_ff": current_ff,
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))

    try:
        data, address = fsa_socket.recvfrom(1024)

        json_obj = json.loads(data.decode("utf-8"))

        if fsa_flag_debug is True:
            Logger().print_trace(
                server_ip + " : " + "Position = %.2f, Velocity = %.0f, Current = %.4f \n"
                % (json_obj.get("position"), json_obj.get("velocity"), json_obj.get("current")))

        if json_obj.get("status") == "OK":
            return json_obj.get("position"), json_obj.get("velocity"), json_obj.get("current")
        else:
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.set_position_control() except")
        return None


# fsa velocity control
# parameter: server IP["xxx.xxx.xxx.xxx"], velocity[deg/s], current feedforward[A]
# return position, velocity, current
def set_velocity_control(server_ip, velocity, current_ff=0.0):
    data = {
        "method": "SET",
        "reqTarget": "/velocity_control",
        "reply_enable": True,
        "velocity": velocity,
        "current_ff": current_ff,
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))

    try:
        data, address = fsa_socket.recvfrom(1024)

        json_obj = json.loads(data.decode("utf-8"))

        if fsa_flag_debug is True:
            Logger().print_trace(
                server_ip + " : " + "Position = %.2f, Velocity = %.0f, Current = %.4f \n"
                % (json_obj.get("position"), json_obj.get("velocity"), json_obj.get("current")))

        if json_obj.get("status") == "OK":
            return json_obj.get("position"), json_obj.get("velocity"), json_obj.get("current")
        else:
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.set_velocity() except")
        return None


def set_torque_control(server_ip, torque):
    # data = {
    #     "method": "SET",
    #     "reqTarget": "/torque_control",
    #     "reply_enable": True,
    #     "torque": torque,
    # }
    data = {
        "method": "SET",
        "reqTarget": "/current_control",
        "reply_enable": True,
        "current": torque,
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))

    try:
        data, address = fsa_socket.recvfrom(1024)

        json_obj = json.loads(data.decode("utf-8"))

        if fsa_flag_debug is True:
            Logger().print_trace(
                server_ip + " : " + "Position = %.2f, Velocity = %.0f, Current = %.4f \n"
                % (json_obj.get("position"), json_obj.get("velocity"), json_obj.get("current")))

        if json_obj.get("status") == "OK":
            return json_obj.get("position"), json_obj.get("velocity"), json_obj.get("current")
        else:
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.set_current() except")
        return None


# fsa current control
# parameter: server IP["xxx.xxx.xxx.xxx"], current[A]
# return position, velocity, current
def set_current_control(server_ip, current):
    data = {
        "method": "SET",
        "reqTarget": "/current_control",
        "reply_enable": True,
        "current": current,
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))

    try:
        data, address = fsa_socket.recvfrom(1024)

        json_obj = json.loads(data.decode("utf-8"))

        if fsa_flag_debug is True:
            Logger().print_trace(
                server_ip + " : " + "Position = %.2f, Velocity = %.0f, Current = %.4f \n"
                % (json_obj.get("position"), json_obj.get("velocity"), json_obj.get("current")))

        if json_obj.get("status") == "OK":
            return json_obj.get("position"), json_obj.get("velocity"), json_obj.get("current")
        else:
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.set_current() except")
        return None


# ---------------------------------------------------------------------------------------------------------------------
# Control Parameters of FSA Group

def enable_group(server_ips):
    time.sleep(1)

    # send request
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        data = {
            "method": "SET",
            "reqTarget": "/control_word",
            "property": "",
            "control_word": FSAControlWord.SERVO_ON,
        }

        json_str = json.dumps(data)

        if fsa_flag_debug is True:
            Logger().print_trace("Send JSON Obj:", json_str)

        if fsa_flag_enable_send_thread is True:
            fsa: FSA = fsa_map[server_ip]
            fsa.add_send_frame(fsa_port_ctrl, data)
        else:
            try:
                fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
            except Exception as e:
                Logger().print_trace_warning("fsa.set_position_control_group() sendto except")

    # receive response
    if fsa_flag_enable_receive_thread is True:
        func_result = [FSAFunctionResult.SUCCESS] * len(server_ips)
    else:
        response = {}
        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            response.update({server_ip: {}})

        if fsa_flag_debug is True:
            Logger().print_trace(response)

        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            try:
                data, address = fsa_socket.recvfrom(1024)
                recv_ip, recv_port = address

                if response.get(recv_ip) is not None:
                    response.get(recv_ip).update({"data": data})
                else:
                    Logger().print_trace_warning("fsa.enable_group() receive wrong ip address ", (recv_ip, recv_port))

                    # 接收到错误的 ip，就再接收一次；如果仍然接收错误，则放弃处理，执行下一次的接收
                    data, address = fsa_socket.recvfrom(1024)
                    recv_ip, recv_port = address

                    if response.get(recv_ip) is not None:
                        response.get(recv_ip).update({"data": data})
                    else:
                        Logger().print_trace_warning("fsa.enable_group() receive wrong ip address ", (recv_ip, recv_port))
                        continue

                if fsa_flag_debug is True:
                    Logger().print_trace("Received from {}:{}".format(recv_ip, data.decode("utf-8")))

            except socket.timeout:  # fail after 1 second of no activity
                Logger().print_trace_error("fsa.enable_group() Timeout")
                continue

            except e:
                Logger().print_trace_warning("fsa.enable_group() except")
                continue

        if fsa_flag_debug is True:
            Logger().print_trace("response = ", response)

        for i in range(len((server_ips))):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            data = response.get(server_ip).get("data")

            try:
                json_obj = json.loads(data.decode("utf-8"))

                if json_obj.get("status") == "OK":
                    response.get(server_ip).update({"return": FSAFunctionResult.SUCCESS})

                else:
                    response.get(server_ip).update({"return": FSAFunctionResult.FAIL})
                    Logger().print_trace_error(server_ip, " receive status is not OK!")
                    continue

            except Exception as e:
                response.get(server_ip).update({"return": FSAFunctionResult.FAIL})
                Logger().print_trace_warning("fsa.enable_group() except")
                continue

        func_result = []
        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            func_result.append(response.get(server_ip).get("return"))

        if fsa_flag_debug is True:
            Logger().print_trace("func_result = ", func_result)

    return func_result


def disable_group(server_ips):
    # send request
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        data = {
            "method": "SET",
            "reqTarget": "/control_word",
            "property": "",
            "control_word": FSAControlWord.SERVO_OFF,
        }

        json_str = json.dumps(data)

        if fsa_flag_debug is True:
            Logger().print_trace("Send JSON Obj:", json_str)

        if fsa_flag_enable_send_thread is True:
            fsa: FSA = fsa_map[server_ip]
            fsa.add_send_frame(fsa_port_ctrl, data)
        else:
            try:
                fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
            except Exception as e:
                Logger().print_trace_warning("fsa.set_position_control_group() sendto except")

    # receive response
    if fsa_flag_enable_receive_thread is True:
        func_result = [FSAFunctionResult.SUCCESS] * len(server_ips)
    else:
        response = {}
        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            response.update({server_ip: {}})

        if fsa_flag_debug is True:
            Logger().print_trace(response)

        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            try:
                data, address = fsa_socket.recvfrom(1024)
                recv_ip, recv_port = address

                if response.get(recv_ip) is not None:
                    response.get(recv_ip).update({"data": data})
                else:
                    Logger().print_trace_warning("fsa.disable_group() receive wrong ip address ", (recv_ip, recv_port))

                    # 接收到错误的 ip，就再接收一次；如果仍然接收错误，则放弃处理，执行下一次的接收
                    data, address = fsa_socket.recvfrom(1024)
                    recv_ip, recv_port = address

                    if response.get(recv_ip) is not None:
                        response.get(recv_ip).update({"data": data})
                    else:
                        Logger().print_trace_warning("fsa.disable_group() receive wrong ip address ",
                                                   (recv_ip, recv_port))
                        continue

                if fsa_flag_debug is True:
                    try:
                        Logger().print_trace("Received from {}:{}".format(recv_ip, data.decode("utf-8")))
                    except Exception as e:
                        Logger().print_trace("Received from {} data none".format(recv_ip))
                        pass

            except socket.timeout:  # fail after 1 second of no activity
                Logger().print_trace_error("fsa.disable_group() Timeout")
                continue

            except e:
                Logger().print_trace_warning("fsa.disable_group() except", e)
                continue

        if fsa_flag_debug is True:
            Logger().print_trace("response = ", response)

        for i in range(len((server_ips))):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            data = response.get(server_ip).get("data")

            try:
                if data is None:
                    Logger().print_trace_error(server_ip, " receive value is None!")
                    response.get(server_ip).update({"return": FSAFunctionResult.FAIL})
                    continue

                json_obj = json.loads(data.decode("utf-8"))

                if json_obj.get("status") == "OK":
                    response.get(server_ip).update({"return": FSAFunctionResult.SUCCESS})

                else:
                    response.get(server_ip).update({"return": FSAFunctionResult.FAIL})
                    Logger().print_trace_error(server_ip, " receive status is not OK!")
                    continue

            # except e:
            except Exception as e:
                response.get(server_ip).update({"return": FSAFunctionResult.FAIL})
                # Logger().print_trace_warning("fsa.disable_group() except", e)
                Logger().print_trace_warning("fsa.disable_group() except")
                continue

        func_result = []
        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            func_result.append(response.get(server_ip).get("return"))

        if fsa_flag_debug is True:
            Logger().print_trace("func_result = ", func_result)

    return func_result


def get_state_group(server_ips):
    # send request
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        data = {
            "method": "GET",
            "reqTarget": "/state",
        }

        json_str = json.dumps(data)

        if fsa_flag_debug is True:
            Logger().print_trace("Send JSON Obj:", json_str)

        fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))

    # receive response
    response = {}
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        response.update({server_ip: {}})

    if fsa_flag_debug is True:
        Logger().print_trace(response)

    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        try:
            data, address = fsa_socket.recvfrom(1024)
            recv_ip, recv_port = address
            response.get(recv_ip).update({"data": data})

            if fsa_flag_debug is True:
                Logger().print_trace("Received from {}:{}".format(recv_ip, data.decode("utf-8")))

        except socket.timeout:  # fail after 1 second of no activity
            Logger().print_trace_error("fsa.get_state_group() Timeout")
            continue

        except Exception as e:
            Logger().print_trace_warning("fsa.get_state_group() except")
            continue

    if fsa_flag_debug is True:
        Logger().print_trace(response)

    for i in range(len((server_ips))):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        data = response.get(server_ip).get("data")

        try:
            json_obj = json.loads(data.decode("utf-8"))

            if json_obj.get("status") == "OK":
                response.get(server_ip).update({"return": FSAFunctionResult.SUCCESS})

            else:
                response.get(server_ip).update({"return": FSAFunctionResult.FAIL})
                Logger().print_trace_error(server_ip, " receive status is not OK!")
                continue

        except Exception as e:
            Logger().print_trace_warning("fsa.get_state_group() except")
            continue

    func_result = []
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        func_result.append(response.get(server_ip).get("return"))

    if fsa_flag_debug is True:
        Logger().print_trace(func_result)

    return func_result


def get_error_group(server_ips):
    # send request
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        data = {
            "method": "GET",
            "reqTarget": "/error_code",
            "property": ""
        }

        json_str = json.dumps(data)

        if fsa_flag_debug is True:
            Logger().print_trace("Send JSON Obj:", json_str)

        if fsa_flag_enable_send_thread is True:
            fsa: FSA = fsa_map[server_ip]
            fsa.add_send_frame(fsa_port_ctrl, data)
        else:
            try:
                fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
            except Exception as e:
                Logger().print_trace_warning("fsa.set_position_control_group() sendto except")

    # receive response
    error_codes = [0] * len(server_ips)

    if fsa_flag_enable_receive_thread is True:
        for i in range(len(server_ips)):
            fsa: FSA = fsa_map.get(server_ips[i])
            error_codes[i] = fsa.error_code
    else:
        response = {}
        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            response.update({server_ip: {}})

        if fsa_flag_debug is True:
            Logger().print_trace(response)

        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            try:
                data, address = fsa_socket.recvfrom(1024)
                recv_ip, recv_port = address
                response.get(recv_ip).update({"data": data})

                if fsa_flag_debug is True:
                    Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

            except socket.timeout:  # fail after 1 second of no activity
                Logger().print_trace_error("fsa.get_error_group() Timeout")
                continue

            except Exception as e:
                Logger().print_trace_warning("fsa.get_error_group() except")
                continue

        if fsa_flag_debug is True:
            Logger().print_trace(response)

        for i in range(len((server_ips))):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            data = response.get(server_ip).get("data")

            try:
                json_obj = json.loads(data.decode("utf-8"))

                if json_obj.get("status") == "OK":
                    response.get(server_ip).update({"return": FSAFunctionResult.SUCCESS})
                    response.get(server_ip).update({"error_code": json_obj.get("error_code")})
                else:
                    response.get(server_ip).update({"return": FSAFunctionResult.FAIL})
                    Logger().print_trace_error(server_ip, " receive status is not OK!")
                    continue

            except Exception as e:
                response.get(server_ip).update({"return": FSAFunctionResult.FAIL})
                Logger().print_trace_warning(server_ip + " fsa.get_error_group() except")
                continue

        error_codes = []
        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            error_codes.append(response.get(server_ip).get("error_code"))

        if fsa_flag_debug is True:
            Logger().print_trace(error_codes)
        Logger().print_trace("error_code=", error_codes)

    return error_codes


def clear_error_group(server_ips):
    # send request
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        data = {
            "method": "SET",
            "reqTarget": "/control_word",
            "property": "",
            "control_word": FSAControlWord.CLEAR_FAULT,
        }

        json_str = json.dumps(data)

        if fsa_flag_debug is True:
            Logger().print_trace("Send JSON Obj:", json_str)

        if fsa_flag_enable_send_thread is True:
            fsa: FSA = fsa_map[server_ip]
            fsa.add_send_frame(fsa_port_ctrl, data)
        else:
            try:
                fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
            except Exception as e:
                Logger().print_trace_warning("fsa.set_position_control_group() sendto except")

    # receive response
    if fsa_flag_enable_receive_thread is True:
        func_result = [FSAFunctionResult.SUCCESS] * len(server_ips)
    else:
        response = {}
        for i in range(len(server_ips)):
            server_ip = server_ips[i]
            response.update({server_ip: {}})

        if fsa_flag_debug is True:
            Logger().print_trace(response)

        for i in range(len(server_ips)):
            try:
                data, address = fsa_socket.recvfrom(1024)
                recv_ip, recv_port = address
                response.get(recv_ip).update({"data": data})

                if fsa_flag_debug is True:
                    Logger().print_trace("Received from {}:{}".format(recv_ip, data.decode("utf-8")))

            except socket.timeout:  # fail after 1 second of no activity
                Logger().print_trace_error("fsa.clear_error_group() Timeout")
                continue

            except Exception as e:
                Logger().print_trace_warning("fsa.clear_error_group() except")
                continue

        if fsa_flag_debug is True:
            Logger().print_trace(response)

        for i in range(len((server_ips))):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            data = response.get(server_ip).get("data")

            try:
                json_obj = json.loads(data.decode("utf-8"))

                if json_obj.get("status") == "OK":
                    response.get(server_ip).update({"return": FSAFunctionResult.SUCCESS})
                else:
                    response.get(server_ip).update({"return": FSAFunctionResult.FAIL})
                    Logger().print_trace_error(server_ip, " receive status is not OK!")
                    continue

            except Exception as e:
                response.get(server_ip).update({"return": FSAFunctionResult.FAIL})
                Logger().print_trace_warning(server_ip + " fsa.clear_error_group() except")
                continue

        func_result = []
        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            func_result.append(response.get(server_ip).get("return"))

        if fsa_flag_debug is True:
            Logger().print_trace(func_result)

    return func_result


def get_pvc_group(server_ips):
    # send request
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        data = {
            "method": "GET",
            "reqTarget": "/measured",
            "position": True,
            "velocity": True,
            "current": True,
        }

        json_str = json.dumps(data)

        if fsa_flag_debug is True:
            Logger().print_trace("Send JSON Obj:", json_str)

        if fsa_flag_enable_send_thread is True:
            fsa: FSA = fsa_map[server_ip]
            fsa.add_send_frame(fsa_port_ctrl, data)
        else:
            try:
                fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
            except Exception as e:
                Logger().print_trace_warning("fsa.set_position_control_group() sendto except")

    # receive response
    positions = [0] * len(server_ips)
    velocitys = [0] * len(server_ips)
    currents = [0] * len(server_ips)
    timeouts = [0] * len(server_ips)

    if fsa_flag_enable_receive_thread is True:
        for i in range(len(server_ips)):
            fsa: FSA = fsa_map.get(server_ips[i])
            positions[i] = fsa.measured_position
            velocitys[i] = fsa.measured_velocity
            currents[i] = fsa.measured_current
    else:
        response = {}
        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            response.update({server_ip: {}})

        if fsa_flag_debug is True:
            Logger().print_trace(response)

        for i in range(len(server_ips)):
            try:
                data, address = fsa_socket.recvfrom(1024)
                recv_ip, recv_port = address
                response.get(recv_ip).update({"data": data})

                if fsa_flag_debug is True:
                    Logger().print_trace("Received from {}:{}".format(recv_ip, data.decode("utf-8")))

            except socket.timeout:  # fail after 1 second of no activity
                Logger().print_trace_error("fsa.get_pvc_group() Timeout")
                continue

            except Exception as e:
                Logger().print_trace_warning("fsa.get_pvc_group() except")
                continue

        if fsa_flag_debug is True:
            Logger().print_trace(response)

        for i in range(len((server_ips))):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            data = response.get(server_ip).get("data")

            try:
                json_obj = json.loads(data.decode("utf-8"))

                if json_obj.get("status") == "OK":
                    response.get(server_ip).update({"return": FSAFunctionResult.SUCCESS})
                    response.get(server_ip).update({"position": json_obj.get("position")})
                    response.get(server_ip).update({"velocity": json_obj.get("velocity")})
                    response.get(server_ip).update({"current": json_obj.get("current")})
                else:
                    response.get(server_ip).update({"return": FSAFunctionResult.FAIL})
                    Logger().print_trace_error(server_ip, " receive status is not OK!")
                    continue

            except Exception as e:
                response.get(server_ip).update({"return": FSAFunctionResult.FAIL})
                Logger().print_trace_warning("fsa.get_pvc_group() except")
                continue

        for index in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            if response.get(server_ips[index]).get("data") is None:
                timeouts[index] = 1

        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            positions[i] = response.get(server_ip).get("position")
            velocitys[i] = response.get(server_ip).get("velocity")
            currents[i] = response.get(server_ip).get("current")

    return positions, velocitys, currents, timeouts


def set_mode_of_operation_group(server_ips, mode_of_operations):
    # send request
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        mode_of_operation = mode_of_operations[i]

        data = {
            "method": "SET",
            "reqTarget": "/mode_of_operation",
            "mode_of_operation": mode_of_operation,
        }

        json_str = json.dumps(data)

        if fsa_flag_debug is True:
            Logger().print_trace("Send JSON Obj:", json_str)

        if fsa_flag_enable_send_thread is True:
            fsa: FSA = fsa_map[server_ip]
            fsa.add_send_frame(fsa_port_ctrl, data)
        else:
            try:
                fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
            except Exception as e:
                Logger().print_trace_warning("fsa.set_position_control_group() sendto except")

    # receive response
    if fsa_flag_enable_receive_thread is True:
        func_result = [FSAFunctionResult.SUCCESS] * len(server_ips)
    else:
        response = {}
        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            response.update({server_ip: {}})

        if fsa_flag_debug is True:
            Logger().print_trace(response)

        for i in range(len(server_ips)):
            try:
                data, address = fsa_socket.recvfrom(1024)
                recv_ip, recv_port = address
                response.get(recv_ip).update({"data": data})

                if fsa_flag_debug is True:
                    Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

            except socket.timeout:  # fail after 1 second of no activity
                Logger().print_trace_error("fsa.set_linear_count_group() Timeout")
                continue

            except Exception as e:
                Logger().print_trace_warning("fsa.set_linear_count_group() except")
                continue

        if fsa_flag_debug is True:
            Logger().print_trace(response)

        for i in range(len((server_ips))):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            data = response.get(server_ip).get("data")

            try:
                json_obj = json.loads(data.decode("utf-8"))

                if json_obj.get("status") == "OK":
                    response.get(server_ip).update({"return": FSAFunctionResult.SUCCESS})
                else:
                    response.get(server_ip).update({"return": FSAFunctionResult.FAIL})
                    Logger().print_trace_error(server_ip, " receive status is not OK!")
                    continue

            except Exception as e:
                response.get(server_ip).update({"return": FSAFunctionResult.FAIL})
                Logger().print_trace_error(server_ip, " receive data decode error!")
                continue

        func_result = []
        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            func_result.append(response.get(server_ip).get("return"))

    return func_result


# fsa reset linear count
# Parameters: including server ip，motor number
# no return code
def set_home_offset_group(server_ips, home_offsets=list):
    # send request
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        home_offset = home_offsets[i]

        data = {
            "method": "SET",
            "reqTarget": "/home_offset",
            "home_offset": home_offset
        }

        json_str = json.dumps(data)

        if fsa_flag_debug is True:
            Logger().print_trace("Send JSON Obj:", json_str)

        if fsa_flag_enable_send_thread is True:
            fsa: FSA = fsa_map[server_ip]
            fsa.add_send_frame(fsa_port_ctrl, data)
        else:
            try:
                fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
            except Exception as e:
                Logger().print_trace_warning("fsa.set_position_control_group() sendto except")

    # receive response
    if fsa_flag_enable_receive_thread is True:
        func_result = [FSAFunctionResult.SUCCESS] * len(server_ips)
    else:
        response = {}
        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            response.update({server_ip: {}})

        if fsa_flag_debug is True:
            Logger().print_trace(response)

        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            try:
                data, address = fsa_socket.recvfrom(1024)
                recv_ip, recv_port = address
                response.get(recv_ip).update({"data": data})

                if fsa_flag_debug is True:
                    Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

            except socket.timeout:  # fail after 1 second of no activity
                Logger().print_trace_error("fsa.set_linear_count_group() Timeout")
                continue

            except Exception as e:
                Logger().print_trace_warning("fsa.set_linear_count_group() except")
                continue

        if fsa_flag_debug is True:
            Logger().print_trace(response)

        for i in range(len((server_ips))):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            data = response.get(server_ip).get("data")

            try:
                json_obj = json.loads(data.decode("utf-8"))

                if json_obj.get("status") == "OK":
                    response.get(server_ip).update({"return": FSAFunctionResult.SUCCESS})
                else:
                    response.get(server_ip).update({"return": FSAFunctionResult.FAIL})
                    Logger().print_trace_error(server_ip, " receive status is not OK!")
                    continue

            except Exception as e:
                response.get(server_ip).update({"return": FSAFunctionResult.FAIL})
                Logger().print_trace_error(server_ip, " receive data decode error!")
                continue

        func_result = []
        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            func_result.append(response.get(server_ip).get("return"))

    return func_result


# fsa position control
# parameter: server IP["xxx.xxx.xxx.xxx"], position[deg], velocity feedforward[deg/s], current feedforward[A]
# return position, velocity, current
def set_position_control_group(server_ips, positions, velocity_ffs, current_ffs):
    # send request
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        position = positions[i]
        velocity_ff = velocity_ffs[i]
        current_ff = current_ffs[i]

        data = {
            "method": "SET",
            "reqTarget": "/position_control",
            "reply_enable": True,
            "position": position,
            "velocity_ff": velocity_ff,
            "current_ff": current_ff,
        }

        json_str = json.dumps(data)

        if fsa_flag_debug is True:
            Logger().print_trace("Send JSON Obj:", json_str)

        if fsa_flag_enable_send_thread is True:
            fsa: FSA = fsa_map[server_ip]
            fsa.add_send_frame(fsa_port_ctrl, data)
        else:
            try:
                fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
            except Exception as e:
                Logger().print_trace_warning("fsa.set_position_control_group() sendto except")

    # reiceve response
    positions = [0] * len(server_ips)
    velocitys = [0] * len(server_ips)
    currents = [0] * len(server_ips)

    if fsa_flag_enable_receive_thread is True:
        for i in range(len(server_ips)):
            fsa: FSA = fsa_map.get(server_ips[i])
            positions[i] = fsa.measured_position
            velocitys[i] = fsa.measured_velocity
            currents[i] = fsa.measured_current
    else:
        # get response
        response = {}
        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            response.update({server_ip: {}})

        receive_ips = []
        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            try:
                data, address = fsa_socket.recvfrom(1024)
                recv_ip, recv_port = address

                json_obj = json.loads(data.decode("utf-8"))
                if json_obj.get("status") == "OK":
                    response.get(recv_ip).update({"data": json_obj})
                    receive_ips.append(recv_ip)
                else:
                    continue
                if recv_ip not in server_ips or fsa_map[server_ip].comm_enable is False:
                    continue

                if fsa_flag_debug is True:
                    Logger().print_trace(
                        str(i) + " : " + "Position = %.2f, Velocity = %.0f, Current = %.4f \n"
                        % (json_obj.get("position"), json_obj.get("velocity"), json_obj.get("current")))

            except socket.timeout:  # fail after 1 second of no activity
                Logger().print_trace_error(
                    str(i) + " : set_position_control_group() Didn't receive anymore data! [Timeout]")
                Logger().print_trace_warning(str(receive_ips) + " is all received ip address")
                continue
                # return None
            except Exception as e:
                Logger().print_trace_warning(str(i) + " fsa.set_position_control_group() except")
                Logger().print_trace_warning(str(receive_ips) + " is all received ip address")
                continue
                # return None

        # data parse
        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            data = response.get(server_ip).get("data")

            if data:
                feedback = [data.get("position"), data.get("velocity"), data.get("current")]
            else:
                feedback = None

            response.get(server_ip).update({"feedback": feedback})

        # feedback
        positions = []
        velocitys = []
        currents = []
        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            feedback = response.get(server_ip).get("feedback")

            if feedback is not None:
                positions.append(feedback[0])
                velocitys.append(feedback[1])
                currents.append(feedback[2])
            else:
                positions.append(None)
                velocitys.append(None)
                currents.append(None)

    return positions, velocitys, currents


def set_velocity_control_group(server_ips, velocities, current_ffs):
    # send request
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        velocity = velocities[i]
        current_ff = current_ffs[i]

        data = {
            "method": "SET",
            "reqTarget": "/velocity_control",
            "reply_enable": True,
            "velocity": velocity,
            "current_ff": current_ff,
        }

        json_str = json.dumps(data)

        if fsa_flag_debug is True:
            Logger().print_trace("Send JSON Obj:", json_str)

        if fsa_flag_enable_send_thread is True:
            fsa: FSA = fsa_map[server_ip]
            fsa.add_send_frame(fsa_port_ctrl, data)
        else:
            try:
                fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
            except Exception as e:
                Logger().print_trace_warning("fsa.set_position_control_group() sendto except")

    # receive response
    positions = [0] * len(server_ips)
    velocitys = [0] * len(server_ips)
    currents = [0] * len(server_ips)

    if fsa_flag_enable_receive_thread is True:
        for i in range(len(server_ips)):
            fsa: FSA = fsa_map.get(server_ips[i])
            positions[i] = fsa.measured_position
            velocitys[i] = fsa.measured_velocity
            currents[i] = fsa.measured_current
    else:
        # get response
        response = {}
        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            response.update({server_ip: {}})

        receive_ips = []
        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            try:
                data, address = fsa_socket.recvfrom(1024)
                recv_ip, recv_port = address

                json_obj = json.loads(data.decode("utf-8"))
                if json_obj.get("status") == "OK":
                    response.get(recv_ip).update({"data": json_obj})
                    receive_ips.append(recv_ip)
                else:
                    continue
                if recv_ip not in server_ips or fsa_map[server_ip].comm_enable is False:
                    continue

                if fsa_flag_debug is True:
                    Logger().print_trace(
                        str(i) + " : " + "Position = %.2f, Velocity = %.0f, Current = %.4f \n"
                        % (json_obj.get("position"), json_obj.get("velocity"), json_obj.get("current")))

            except socket.timeout:  # fail after 1 second of no activity
                Logger().print_trace_error(
                    str(i) + " : set_velocity_control_group() Didn't receive anymore data! [Timeout]")
                Logger().print_trace_warning(str(receive_ips) + " is all received ip address")
                continue
                # return None
            except Exception as e:
                Logger().print_trace_warning(str(i) + " fsa.set_velocity_control_group() except")
                Logger().print_trace_warning(str(receive_ips) + " is all received ip address")
                continue
                # return None

        # data parse
        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            data = response.get(server_ip).get("data")

            if data:
                feedback = [data.get("position"), data.get("velocity"), data.get("current")]
            else:
                feedback = None

            response.get(server_ip).update({"feedback": feedback})

        # feedback
        positions = []
        velocitys = []
        currents = []
        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            feedback = response.get(server_ip).get("feedback")

            if feedback is not None:
                positions.append(feedback[0])
                velocitys.append(feedback[1])
                currents.append(feedback[2])
            else:
                positions.append(None)
                velocitys.append(None)
                currents.append(None)

    return positions, velocitys, currents


def set_current_control_group(server_ips, currents):
    # send request
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        current = currents[i]

        data = {
            "method": "SET",
            "reqTarget": "/current_control",
            "reply_enable": True,
            "current": current,
        }

        json_str = json.dumps(data)

        if fsa_flag_debug is True:
            Logger().print_trace("Send JSON Obj:", json_str)

        if fsa_flag_enable_send_thread is True:
            fsa: FSA = fsa_map[server_ip]
            fsa.add_send_frame(fsa_port_ctrl, data)
        else:
            try:
                fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
            except Exception as e:
                Logger().print_trace_warning("fsa.set_position_control_group() sendto except")

    # receive response
    positions = [0] * len(server_ips)
    velocitys = [0] * len(server_ips)
    currents = [0] * len(server_ips)

    # receive response
    if fsa_flag_enable_receive_thread is True:
        for i in range(len(server_ips)):
            fsa: FSA = fsa_map.get(server_ips[i])
            positions[i] = fsa.measured_position
            velocitys[i] = fsa.measured_velocity
            currents[i] = fsa.measured_current
    else:
        response = {}
        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            response.update({server_ip: {}})

        receive_ips = []
        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            try:
                data, address = fsa_socket.recvfrom(1024)
                recv_ip, recv_port = address

                json_obj = json.loads(data.decode("utf-8"))
                if json_obj.get("status") == "OK":
                    response.get(recv_ip).update({"data": json_obj})
                    receive_ips.append(recv_ip)
                else:
                    continue

                if recv_ip not in server_ips or fsa_map[server_ip].comm_enable is False:
                    continue

                if fsa_flag_debug is True:
                    Logger().print_trace(
                        str(i) + " : " + "Position = %.2f, Velocity = %.0f, Current = %.4f \n"
                        % (json_obj.get("position"), json_obj.get("velocity"), json_obj.get("current")))

            except socket.timeout:  # fail after 1 second of no activity
                Logger().print_trace_error(
                    str(i) + " : set_current_control_group() Didn't receive anymore data! [Timeout]")
                Logger().print_trace_warning(str(receive_ips) + " is all received ip address")
                continue
                # return None
            except Exception as e:
                Logger().print_trace_warning(str(i) + " fsa.set_current_control_group() except")
                Logger().print_trace_warning(str(receive_ips) + " is all received ip address")
                continue
                # return None

        # data parse
        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            data = response.get(server_ip).get("data")

            if data:
                feedback = [data.get("position"), data.get("velocity"), data.get("current")]
            else:
                feedback = None

            response.get(server_ip).update({"feedback": feedback})

        # feedback
        positions = []
        velocitys = []
        currents = []
        for i in range(len(server_ips)):
            server_ip = server_ips[i]

            if fsa_map[server_ip].comm_enable is False:
                continue

            feedback = response.get(server_ip).get("feedback")

            if feedback is not None:
                positions.append(feedback[0])
                velocitys.append(feedback[1])
                currents.append(feedback[2])
            else:
                positions.append(None)
                velocitys.append(None)
                currents.append(None)

    return positions, velocitys, currents


# ---------------------------------------------------------------------------------------------------------------------
# Communication Parameters of FSA

# fsa Get root attributes
# Parameters: including device IP
# Get all basic attributes of fsa, including serial number, bus voltage, motor temperature, inverter temperature, version number
def get_comm_root(server_ip):
    data = {
        "method": "GET",
        "reqTarget": "/",
        "property": "",
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_comm))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        return json_obj
    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.get_root() except")


# fsa Get Root Config property
# Parameters: including device IP
# Get fsa bus voltage over-voltage and under-voltage protection threshold
def get_comm_config(server_ip):
    data = {
        "method": "GET",
        "reqTarget": "/config",
        "property": "",
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_comm))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.get_root_config() except")
        return None


# fsa set Root Config properties
# Parameter: The protection threshold of bus voltage overvoltage and undervoltage
# Return success or failure
def set_comm_config(server_ip, dict):
    data = {
        "method": "SET",
        "reqTarget": "/config",
        "property": "",
        "name": dict.get("name"),
        "DHCP_enable": dict.get("DHCP_enable"),
        "SSID": dict.get("SSID"),
        "password": dict.get("password"),
        "static_IP": dict.get("static_IP"),
        "gateway": dict.get("gateway"),
        "subnet_mask": dict.get("subnet_mask"),
        "dns_1": dict.get("dns_1"),
        "dns_2": dict.get("dns_2"),
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_comm))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.set_root_config() except")
        return None


# fsa save configuration
# Parameters: including device IP
def save_comm_config(server_ip):
    data = {
        "method": "SET",
        "reqTarget": "/config",
        "property": "save"
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_comm))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.save_config() except")
        return None


# fsa clear configuration
# Parameters: including device IP
def erase_comm_config(server_ip):
    data = {
        "method": "SET",
        "reqTarget": "/config",
        "property": "erase"
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_comm))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.erase_config() except")
        return None


# fsa restart
# Parameters: including device IP
def reboot_comm(server_ip):
    data = {
        "method": "SET",
        "reqTarget": "/reboot",
        "property": ""
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_comm))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSAFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.reboot() except")
        return None


# ---------------------------------------------------------------------------------------------------------------------
# Communication Parameters of FSA Group

# fsa restart
# Parameters: including device IP
def reboot_comm_group(server_ips):
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        data = {
            "method": "SET",
            "reqTarget": "/reboot",
            "property": ""
        }

        json_str = json.dumps(data)

        if fsa_flag_debug is True:
            Logger().print_trace("Send JSON Obj:", json_str)

        fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_comm))

    response = {}
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        response.update({server_ip: {}})

    if fsa_flag_debug is True:
        Logger().print_trace(response)

    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        try:
            data, address = fsa_socket.recvfrom(1024)
            recv_ip, recv_port = address
            response.get(recv_ip).update({"data": data})

            if fsa_flag_debug is True:
                Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        except socket.timeout:  # fail after 1 second of no activity
            Logger().print_trace_error("fsa.reboot_group() Timeout")
            continue

        except Exception as e:
            Logger().print_trace_warning("fsa.reboot_group() except")
            continue

    if fsa_flag_debug is True:
        Logger().print_trace(response)

    for i in range(len((server_ips))):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        data = response.get(server_ip).get("data")

        try:
            json_obj = json.loads(data.decode("utf-8"))

            if json_obj.get("status") == "OK":
                response.get(server_ip).update({"return": FSAFunctionResult.SUCCESS})
            else:
                response.get(server_ip).update({"return": FSAFunctionResult.FAIL})
                Logger().print_trace_error(server_ip, " receive status is not OK!")
                continue

        except Exception as e:
            response.get(server_ip).update({"return": FSAFunctionResult.FAIL})
            Logger().print_trace_warning(server_ip + " fsa.reboot_group() except")
            continue

    func_result = []
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        func_result.append(response.get(server_ip).get("return"))

    if fsa_flag_debug is True:
        Logger().print_trace(func_result)

    return func_result


# ---------------------------------------------------------------------------------------------------------------------
# Communication Parameters of FSE

# get abs encoder position
# Parameters: including server ip
# Return position in tuple
def get_abs_encoder_angle(server_ip):
    data = {
        "method": "GET",
        "reqTarget": "/measured",
        "property": ""
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_ctrl))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return json_obj.get("angle")
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return FSAFunctionResult.FAIL

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return FSAFunctionResult.TIMEOUT

    except Exception as e:
        Logger().print_trace_warning(server_ip + " fsa.get_abs_encoder_angle() except")
        return FSAFunctionResult.FAIL


# ---------------------------------------------------------------------------------------------------------------------
# FAST 通信模式 （使用字节流进行数据传输）

def fast_set_enable(server_ip):
    tx_messages = struct.pack('>B', 0x01)

    if fsa_flag_debug is True:
        Logger().print_trace(server_ip + " : Send Data:", tx_messages)

    try:
        fsa_socket.sendto(tx_messages, (server_ip, fsa_port_fast))
    except Exception as e:
        Logger().print_trace("fi_fsa.fast_set_enable() except: \n", e)

    return FSAFunctionResult.SUCCESS


def fast_set_disable(server_ip):
    tx_messages = struct.pack('>B', 0x02)

    if fsa_flag_debug is True:
        Logger().print_trace(server_ip + " : Send Data:", tx_messages)

    try:
        fsa_socket.sendto(tx_messages, (server_ip, fsa_port_fast))
    except Exception as e:
        Logger().print_trace("fi_fsa.fast_set_disable() except: \n", e)

    return FSAFunctionResult.SUCCESS


def fast_set_clear_fault(server_ip):
    tx_messages = struct.pack('>B', 0x03)

    if fsa_flag_debug is True:
        Logger().print_trace(server_ip + " : Send Data:", tx_messages)

    try:
        fsa_socket.sendto(tx_messages, (server_ip, fsa_port_fast))
    except Exception as e:
        Logger().print_trace("fi_fsa.fast_set_clear_fault() except: \n", e)

    return FSAFunctionResult.SUCCESS


def fast_set_position_mode(server_ip):
    tx_messages = struct.pack('>B', 0x04)

    if fsa_flag_debug is True:
        Logger().print_trace(server_ip + " : Send Data:", tx_messages)

    try:
        fsa_socket.sendto(tx_messages, (server_ip, fsa_port_fast))
    except Exception as e:
        Logger().print_trace("fi_fsa.fast_set_position_mode() except: \n", e)

    return FSAFunctionResult.SUCCESS


def fast_set_velocity_mode(server_ip):
    tx_messages = struct.pack('>B', 0x05)

    if fsa_flag_debug is True:
        Logger().print_trace(server_ip + " : Send Data:", tx_messages)

    try:
        fsa_socket.sendto(tx_messages, (server_ip, fsa_port_fast))
    except Exception as e:
        Logger().print_trace("fi_fsa.fast_set_velocity_mode() except: \n", e)

    return FSAFunctionResult.SUCCESS


def fast_set_torque_mode(server_ip):
    tx_messages = struct.pack('>B', 0x06)

    if fsa_flag_debug is True:
        Logger().print_trace(server_ip + " : Send Data:", tx_messages)

    try:
        fsa_socket.sendto(tx_messages, (server_ip, fsa_port_fast))
    except Exception as e:
        Logger().print_trace("fi_fsa.fast_set_torque_mode() except: \n", e)

    return FSAFunctionResult.SUCCESS


def fast_set_current_mode(server_ip):
    tx_messages = struct.pack('>B', 0x07)

    if fsa_flag_debug is True:
        Logger().print_trace(server_ip + " : Send Data:", tx_messages)

    try:
        fsa_socket.sendto(tx_messages, (server_ip, fsa_port_fast))
    except Exception as e:
        Logger().print_trace("fi_fsa.fast_set_current_mode() except: \n", e)

    return FSAFunctionResult.SUCCESS


def fast_set_mode_of_operation(server_ip, mode_of_operation):
    if mode_of_operation == FSAModeOfOperation.POSITION_CONTROL:
        return fast_set_position_mode(server_ip)
    elif mode_of_operation == FSAModeOfOperation.VELOCITY_CONTROL:
        return fast_set_velocity_mode(server_ip)
    elif mode_of_operation == FSAModeOfOperation.TORQUE_CONTROL:
        return fast_set_torque_mode(server_ip)
    elif mode_of_operation == FSAModeOfOperation.CURRENT_CONTROL:
        return fast_set_current_mode(server_ip)
    else:
        return None


def fast_set_position_control(server_ip, position, velocity_ff=0, current_ff=0):
    tx_messages = struct.pack('>Bfff', 0x0A, position, velocity_ff, current_ff)

    if fsa_flag_debug is True:
        Logger().print_trace(server_ip + " : Send Data:", tx_messages)

    try:
        fsa_socket.sendto(tx_messages, (server_ip, fsa_port_fast))
    except Exception as e:
        Logger().print_trace("fi_fsa.fast_set_position_control() except: \n", e)

    return FSAFunctionResult.SUCCESS


def fast_set_velocity_control(server_ip, velocity, current_ff=0):
    tx_messages = struct.pack('>Bff', 0x0B, velocity, current_ff)

    if fsa_flag_debug is True:
        Logger().print_trace(server_ip + " : Send Data:", tx_messages)

    try:
        fsa_socket.sendto(tx_messages, (server_ip, fsa_port_fast))
    except Exception as e:
        Logger().print_trace("fi_fsa.fast_set_velocity_control() except: \n", e)

    return FSAFunctionResult.SUCCESS


def fast_set_torque_control(server_ip, torque):
    tx_messages = struct.pack('>Bf', 0x0C, torque)

    if fsa_flag_debug is True:
        Logger().print_trace(server_ip + " : Send Data:", tx_messages)

    try:
        fsa_socket.sendto(tx_messages, (server_ip, fsa_port_fast))
    except Exception as e:
        Logger().print_trace("fi_fsa.fast_set_torque_control() except: \n", e)

    return FSAFunctionResult.SUCCESS


def fast_set_current_control(server_ip, current):
    tx_messages = struct.pack('>Bf', 0x0D, current)

    if fsa_flag_debug is True:
        Logger().print_trace(server_ip + " : Send Data:", tx_messages)

    try:
        fsa_socket.sendto(tx_messages, (server_ip, fsa_port_fast))
    except Exception as e:
        Logger().print_trace("fi_fsa.fast_set_current_control() except: \n", e)

    return FSAFunctionResult.SUCCESS


# AIOS get pvc through pt port
# 参数：包括设备IP 电机号
# 无返回
def fast_get_pvc(server_ip):
    tx_messages = struct.pack('>B', 0x1A)

    if fsa_flag_debug is True:
        Logger().print_trace(server_ip + " : Send Data:", tx_messages)

    fsa_socket.sendto(tx_messages, (server_ip, fsa_port_fast))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace(server_ip + ': Server received from {}:{}'.format(address, data))

        feedback, position, velocity, current = struct.unpack('>Bfff', data[0:1 + 4 + 4 + 4])
        return position, velocity, current

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except:
        Logger().print_trace_warning(server_ip + " fi_fsa.fast_get_pvc() except")
        return None


def fast_get_error(server_ip):
    tx_messages = struct.pack('>B', 0x1B)

    if fsa_flag_debug is True:
        Logger().print_trace(server_ip + " : Send Data:", tx_messages)

    fsa_socket.sendto(tx_messages, (server_ip, fsa_port_fast))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace(server_ip + ': Server received from {}:{}'.format(address, data))

        feedback, error = struct.unpack('>Bi', data[0:1 + 4])
        return error

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except:
        Logger().print_trace_warning(server_ip + " fi_fsa.get_error_fast() except")
        return None


# ---------------------------------------------------------------------------------------------------------------------
# FSA FAST Group

def fast_set_enable_group(server_ips):
    # send request
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        tx_messages = struct.pack('>B', 0x01)

        try:
            fsa_socket.sendto(tx_messages, (server_ip, fsa_port_fast))
        except Exception as e:
            Logger().print_trace("fi_fsa.fast_set_enable_group() except: \n", e)

    return FSAFunctionResult.SUCCESS


def fast_set_disable_group(server_ips):
    # send request
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        tx_messages = struct.pack('>B', 0x02)

        try:
            fsa_socket.sendto(tx_messages, (server_ip, fsa_port_fast))
        except Exception as e:
            Logger().print_trace("fi_fsa.fast_set_disable_group() except: \n", e)

    return FSAFunctionResult.SUCCESS


def fast_set_clear_fault_group(server_ips):
    # send request
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        tx_messages = struct.pack('>B', 0x03)

        try:
            fsa_socket.sendto(tx_messages, (server_ip, fsa_port_fast))
        except Exception as e:
            Logger().print_trace("fi_fsa.fast_set_clear_fault_group() except: \n", e)

    return FSAFunctionResult.SUCCESS


def fast_set_mode_of_operation_group(server_ips, mode_of_operations):
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        mode_of_operation = mode_of_operations[i]

        fast_set_mode_of_operation(server_ip=server_ip,
                                   mode_of_operation=mode_of_operation)

    return FSAFunctionResult.SUCCESS


def fast_set_position_control_group(server_ips, positions, velocity_ffs, current_ffs):
    # send request
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        position = positions[i]
        velocity_ff = velocity_ffs[i]
        current_ff = current_ffs[i]

        tx_messages = struct.pack('>Bfff', 0x0A, position, velocity_ff, current_ff)

        try:
            fsa_socket.sendto(tx_messages, (server_ip, fsa_port_fast))
        except Exception as e:
            Logger().print_trace("fi_fsa.fast_set_position_control_group() except: \n", e)

    return FSAFunctionResult.SUCCESS


def fast_set_velocity_control_group(server_ips, velocitys, current_ffs):
    # send request
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        velocity = velocitys[i]
        torque = current_ffs[i]

        tx_messages = struct.pack('>Bff', 0x0B, velocity, torque)

        try:
            fsa_socket.sendto(tx_messages, (server_ip, fsa_port_fast))
        except Exception as e:
            Logger().print_trace("fi_fsa.fast_set_velocity_control_group() except: \n", e)

    return FSAFunctionResult.SUCCESS


def fast_set_torque_control_group(server_ips, torques):
    # send request
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        torque = torques[i]

        tx_messages = struct.pack('<Bf', 0x0C, torque)

        try:
            fsa_socket.sendto(tx_messages, (server_ip, fsa_port_fast))
        except Exception as e:
            Logger().print_trace("fi_fsa.fast_set_torque_control_group() except: \n", e)

    return FSAFunctionResult.SUCCESS


# Jason 2024-01-27:
# Time Cost: 0.5 ~ 0.8ms
def fast_get_pvc_group(server_ips):
    # send request
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        tx_messages = struct.pack('>B', 0x1A)

        try:
            fsa_socket.sendto(tx_messages, (server_ip, fsa_port_fast))
        except Exception as e:
            Logger().print_trace("fi_fsa.fast_get_pvc_group() except: \n", e)

    # get response
    response = {}
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        response.update({server_ip: {}})

    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        try:
            data, address = fsa_socket.recvfrom(1024)

            recv_ip, recv_port = address

            if recv_ip not in server_ips or fsa_map[server_ip].comm_enable is False:
                continue

            # print(recv_ip)

            response.get(recv_ip).update({"data": data})

        except socket.timeout:  # fail after 1 second of no activity
            Logger().print_trace_error("fi_fsa.fast_get_pvc_group() Timeout")
            continue

    # data parse
    feedbacks = []
    positions = []
    velocitys = []
    currents = []

    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            feedbacks.append(0)
            positions.append(0)
            velocitys.append(0)
            currents.append(0)
            continue

        data = response.get(server_ip).get("data")

        if data is None:
            feedback, position, velocity, current = None, None, None, None
        else:
            feedback, position, velocity, current = struct.unpack('>Bfff', data[0:1 + 4 + 4 + 4])

        feedbacks.append(feedback)
        positions.append(position)
        velocitys.append(velocity)
        currents.append(current)

    return positions, velocitys, currents


# Jason 2024-01-27:
# Time Cost: 0.5 ~ 0.8ms
def fast_get_error_group(server_ips):
    # send request
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        tx_messages = struct.pack('>B', 0x1B)

        try:
            fsa_socket.sendto(tx_messages, (server_ip, fsa_port_fast))
        except Exception as e:
            Logger().print_trace("fi_fsa.fast_get_error_group() except: \n", e)

    # get response
    response = {}
    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        response.update({server_ip: {}})

    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            continue

        try:
            data, address = fsa_socket.recvfrom(1024)

            recv_ip, recv_port = address

            if recv_ip not in server_ips or fsa_map[server_ip].comm_enable is False:
                continue

            response.get(recv_ip).update({"data": data})

        except socket.timeout:  # fail after 1 second of no activity
            Logger().print_trace_error("fi_fsa.fast_get_error_group() Timeout")
            continue

    # data parse
    feedbacks = []
    errors = []

    for i in range(len(server_ips)):
        server_ip = server_ips[i]

        if fsa_map[server_ip].comm_enable is False:
            feedbacks.append(0)
            errors.append(0)
            continue

        data = response.get(server_ip).get("data")

        if data is None:
            feedback, error = None, None
        else:
            feedback, error = struct.unpack('>Bi', data[0:1 + 4])

        feedbacks.append(feedback)
        errors.append(error)

    return errors


# ---------------------------------------------------------------------------------------------------------------------
# Broadcast Query FSA in LAN

# 广播查询局域网下的全部
# 参数：无
# 返回 成功 失败 超时
def broadcast_func():
    Logger().print_trace("fi_fsa start listening for broadcast...")

    found_server = False
    address_list = []

    fsa_socket.sendto("Is any fourier smart server here?".encode("utf-8"), (fsa_network, fsa_port_comm))
    print("\n")

    while True:
        try:
            data, address = fsa_socket.recvfrom(1024)
            address_list.append(address[0])
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))
            # json_obj = json.loads(data.decode("utf-8"))
            found_server = True

        except socket.timeout:  # fail after 1 second of no activity
            if found_server:
                print("\n")
                print("found servers")
                print(address_list)
                print("lookup Finished! \n")
                time.sleep(2)
                return address_list
            else:
                Logger().print_trace_error("Do not have any server! [Timeout] \n")
                return False


# 广播查询局域网下的全部 filter_type = "Actuator" or "AbsEncoder" or "CtrlBox"
def broadcast_func_with_filter(filter_type=None):
    Logger().print_trace("fi_fsa start listening for broadcast...")

    found_server = False
    address_list = []

    fsa_socket.sendto("Is any fourier smart server here?".encode("utf-8"), (fsa_network, fsa_port_comm))
    print("\n")

    while True:
        try:
            data, address = fsa_socket.recvfrom(1024)
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

            # 如果没有过滤条件，直接返回所有的IP
            # 如果有过滤条件，只返回符合条件的IP
            if filter_type is None:
                address_list.append(address[0])
                found_server = True
                continue
            else:
                pass

            json_obj = json.loads(data.decode("utf-8"))
            if "type" in json_obj:
                if json_obj["type"] == filter_type:
                    address_list.append(address[0])
                    found_server = True

        except socket.timeout:  # fail after 1 second of no activity
            if found_server:
                print("\n")
                print("found servers")
                print(address_list)
                print("lookup Finished! \n")
                time.sleep(2)
                return address_list
            else:
                Logger().print_trace_error("Do not have any server! [Timeout] \n")
                return False


def ota(server_ip):
    data = {
        "method": "SET",
        "reqTarget": "/ota",
        "property": ""
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_comm))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode('utf-8'))

    except socket.timeout:  # fail after 1 second of no activity
        print("Didn't receive anymore data! [Timeout]")


def ota_test(server_ip):
    data = {
        "method": "SET",
        "reqTarget": "/ota_test",
        "property": ""
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_comm))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode('utf-8'))

    except socket.timeout:  # fail after 1 second of no activity
        print("Didn't receive anymore data! [Timeout]")


def ota_devel(server_ip):
    data = {
        "method": "SET",
        "reqTarget": "/ota_devel",
        "property": ""
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_comm))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode('utf-8'))

    except socket.timeout:  # fail after 1 second of no activity
        print("Didn't receive anymore data! [Timeout]")


def ota_cloud(server_ip):
    data = {
        "method": "SET",
        "reqTarget": "/ota_cloud",
        "property": ""
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_comm))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode('utf-8'))

    except socket.timeout:  # fail after 1 second of no activity
        print("Didn't receive anymore data! [Timeout]")


def ota_driver(server_ip):
    data = {
        "method": "SET",
        "reqTarget": "/ota_driver",
        "property": ""
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_comm))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode('utf-8'))

    except socket.timeout:  # fail after 1 second of no activity
        print("Didn't receive anymore data! [Timeout]")


def ota_driver_test(server_ip):
    data = {
        "method": "SET",
        "reqTarget": "/ota_driver_test",
        "property": ""
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_comm))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode('utf-8'))

    except socket.timeout:  # fail after 1 second of no activity
        print("Didn't receive anymore data! [Timeout]")


def ota_driver_devel(server_ip):
    data = {
        "method": "SET",
        "reqTarget": "/ota_driver_devel",
        "property": ""
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_comm))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode('utf-8'))

    except socket.timeout:  # fail after 1 second of no activity
        print("Didn't receive anymore data! [Timeout]")


def ota_driver_cloud(server_ip):
    data = {
        "method": "SET",
        "reqTarget": "/ota_driver_cloud",
        "property": ""
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_comm))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode('utf-8'))

    except socket.timeout:  # fail after 1 second of no activity
        print("Didn't receive anymore data! [Timeout]")


def encrypt(server_ip, username, password):
    data = {
        "method": "SET",
        "reqTarget": "/encrypt",
        "property": "",
        "username": username,
        "password": password
    }

    json_str = json.dumps(data)

    if fsa_flag_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    fsa_socket.sendto(str.encode(json_str), (server_ip, fsa_port_comm))
    try:
        data, address = fsa_socket.recvfrom(1024)

        if fsa_flag_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode('utf-8'))

    except socket.timeout:  # fail after 1 second of no activity
        print("Didn't receive anymore data! [Timeout]")
