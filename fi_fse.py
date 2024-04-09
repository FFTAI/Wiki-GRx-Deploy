import socket
import time
import json

from .fi_logger import Logger


class FSEFunctionResult:
    SUCCESS = 0
    FAIL = -1
    RUNNING = 1
    PREPARE = 2
    EXECUTE = 3
    NOT_EXECUTE = 4
    TIMEOUT = 5


class FSEFlagState:
    CLEAR = 0
    SET = 1


fse_timeout = 0.1
fse_port_ctrl = 2333
fse_port_comm = 2334
fse_port_fast = 2335
fse_network = "192.168.137.255"
fse_debug = False

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(fse_timeout)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

Logger().print_trace("FSE start listening for broadcast...")


# ---------------------------------------------------------------------------------------------------------------------

def init(server_ip):
    return FSEFunctionResult.SUCCESS


def init_group(server_ips):
    return FSEFunctionResult.SUCCESS


# ---------------------------------------------------------------------------------------------------------------------


def get_root(server_ip):
    data = {
        "method": "GET",
        "reqTarget": "/",
        "property": ""
    }

    json_str = json.dumps(data)

    if fse_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    s.sendto(str.encode(json_str), (server_ip, fse_port_ctrl))
    try:
        data, address = s.recvfrom(1024)

        if fse_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
    except:
        Logger().print_trace_warning(server_ip + " fi_fse.get_root() except")


def set_root(server_ip):
    data = {"method": "SET",
            "reqTarget": "/",
            "property": "",
            }

    json_str = json.dumps(data)

    if fse_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    s.sendto(str.encode(json_str), (server_ip, fse_port_ctrl))
    try:
        data, address = s.recvfrom(1024)

        if fse_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSEFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except:
        Logger().print_trace_warning(server_ip + " fi_fse.set_root_config() except")
        return None


def get_config(server_ip):
    data = {
        "method": "GET",
        "reqTarget": "/config",
        "property": ""
    }

    json_str = json.dumps(data)

    if fse_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    s.sendto(str.encode(json_str), (server_ip, fse_port_ctrl))
    try:
        data, address = s.recvfrom(1024)

        if fse_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSEFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except:
        Logger().print_trace_warning(server_ip + " fi_fse.get_root_config() except")
        return None


def set_config(server_ip, dict):
    data = {"method": "SET",
            "reqTarget": "/config",
            "property": "",

            "home_offset": dict["home_offset"],
            }

    json_str = json.dumps(data)

    if fse_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    s.sendto(str.encode(json_str), (server_ip, fse_port_ctrl))
    try:
        data, address = s.recvfrom(1024)

        if fse_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSEFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except:
        Logger().print_trace_warning(server_ip + " fi_fse.set_root_config() except")
        return None


def save_config(server_ip):
    data = {
        "method": "SET",
        "reqTarget": "/config",
        "property": "save"
    }

    json_str = json.dumps(data)

    if fse_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    s.sendto(str.encode(json_str), (server_ip, fse_port_ctrl))
    try:
        data, address = s.recvfrom(1024)

        if fse_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSEFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except:
        Logger().print_trace_warning(server_ip + " fi_fse.save_config() except")
        return None


def get_measured(server_ip):
    data = {
        "method": "GET",
        "reqTarget": "/measured",
    }

    json_str = json.dumps(data)

    if fse_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    s.sendto(str.encode(json_str), (server_ip, fse_port_ctrl))
    try:
        data, address = s.recvfrom(1024)

        if fse_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return json_obj.get("angle"), json_obj.get("radian")
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return FSEFunctionResult.FAIL

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return FSEFunctionResult.TIMEOUT

    except:
        Logger().print_trace_warning(server_ip + " fi_fse.get_pvc() except")
        return FSEFunctionResult.FAIL


def reboot(server_ip):
    data = {
        "method": "SET",
        "reqTarget": "/reboot",
        "property": ""
    }

    json_str = json.dumps(data)

    if fse_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    s.sendto(str.encode(json_str), (server_ip, fse_port_ctrl))
    try:
        data, address = s.recvfrom(1024)

        if fse_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSEFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except:
        Logger().print_trace_warning(server_ip + " fi_fse.reboot_motor_drive() except")
        return None


def get_home_offset(server_ip):
    data = {
        "method": "GET",
        "reqTarget": "/home_offset",
        "property": ""
    }

    json_str = json.dumps(data)

    if fse_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    s.sendto(str.encode(json_str), (server_ip, fse_port_ctrl))
    try:
        data, address = s.recvfrom(1024)

        if fse_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSEFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except:
        Logger().print_trace_warning(server_ip + " fi_fse.get_root_config() except")
        return None


# fse reset linear count
# Parameters: including server ip，motor number
# no return code
def set_home_offset(server_ip, home_offset):
    data = {
        "method": "SET",
        "reqTarget": "/home_offset",
        "home_offset": home_offset,
    }

    json_str = json.dumps(data)

    if fse_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    s.sendto(str.encode(json_str), (server_ip, fse_port_ctrl))
    try:
        data, address = s.recvfrom(1024)

        if fse_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSEFunctionResult.SUCCESS
        else:
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except:
        Logger().print_trace_warning(server_ip + " fi_fse.set_linear_count() except")
        return None


def set_home_position(server_ip):
    data = {
        "method": "SET",
        "reqTarget": "/home_position",
    }

    json_str = json.dumps(data)

    if fse_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    s.sendto(str.encode(json_str), (server_ip, fse_port_ctrl))
    try:
        data, address = s.recvfrom(1024)

        if fse_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSEFunctionResult.SUCCESS
        else:
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except:
        Logger().print_trace_warning(server_ip + " fi_fse.set_linear_count() except")
        return None


def get_comm_root(server_ip):
    data = {
        "method": "GET",
        "reqTarget": "/",
        "property": "",
    }

    json_str = json.dumps(data)

    if fse_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    s.sendto(str.encode(json_str), (server_ip, fse_port_comm))
    try:
        data, address = s.recvfrom(1024)

        if fse_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        return json_obj
    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
    except:
        Logger().print_trace_warning(server_ip + " fi_fse.get_root() except")


def set_comm_root(server_ip, dict):
    data = {
        "method": "SET",
        "reqTarget": "/",
        "property": "",
    }

    json_str = json.dumps(data)

    if fse_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    s.sendto(str.encode(json_str), (server_ip, fse_port_comm))
    try:
        data, address = s.recvfrom(1024)

        if fse_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSEFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except:
        Logger().print_trace_warning(server_ip + " fi_fse.set_comm_config() except")
        return None


def get_comm_config(server_ip):
    data = {
        "method": "GET",
        "reqTarget": "/config",
        "property": "",
    }

    json_str = json.dumps(data)

    if fse_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    s.sendto(str.encode(json_str), (server_ip, fse_port_comm))
    try:
        data, address = s.recvfrom(1024)

        if fse_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSEFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except:
        Logger().print_trace_warning(server_ip + " fi_fse.get_root_config() except")
        return None


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

    if fse_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    s.sendto(str.encode(json_str), (server_ip, fse_port_comm))
    try:
        data, address = s.recvfrom(1024)

        if fse_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSEFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except:
        Logger().print_trace_warning(server_ip + " fi_fse.set_comm_config() except")
        return None


def save_comm_config(server_ip):
    data = {
        "method": "SET",
        "reqTarget": "/config",
        "property": "save"
    }

    json_str = json.dumps(data)

    if fse_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    s.sendto(str.encode(json_str), (server_ip, fse_port_comm))
    try:
        data, address = s.recvfrom(1024)

        if fse_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSEFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except:
        Logger().print_trace_warning(server_ip + " fi_fse.save_config() except")
        return None


def reboot_comm(server_ip):
    data = {
        "method": "SET",
        "reqTarget": "/reboot",
        "property": ""
    }

    json_str = json.dumps(data)

    if fse_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    s.sendto(str.encode(json_str), (server_ip, fse_port_comm))
    try:
        data, address = s.recvfrom(1024)

        if fse_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode("utf-8"))

        if json_obj.get("status") == "OK":
            return FSEFunctionResult.SUCCESS
        else:
            Logger().print_trace_error(server_ip, " receive status is not OK!")
            return None

    except socket.timeout:  # fail after 1 second of no activity
        Logger().print_trace_error(server_ip + " : Didn't receive anymore data! [Timeout]")
        return None

    except:
        Logger().print_trace_warning(server_ip + " fi_fse.reboot() except")
        return None


# ---------------------------------------------------------------------------------------------------------------------
# Broadcast Query FSE in LAN

def broadcast_func():
    Logger().print_trace("FSE start listening for broadcast...")

    found_server = False
    address_list = []

    s.sendto("Is any fourier smart server here?".encode("utf-8"), (fse_network, fse_port_comm))
    print("\n")

    while True:
        try:
            data, address = s.recvfrom(1024)
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
    Logger().print_trace("FSE start listening for broadcast...")

    found_server = False
    address_list = []

    s.sendto("Is any fourier smart server here?".encode("utf-8"), (fse_network, fse_port_comm))
    print("\n")

    while True:
        try:
            data, address = s.recvfrom(1024)
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

    if fse_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    s.sendto(str.encode(json_str), (server_ip, fse_port_comm))
    try:
        data, address = s.recvfrom(1024)

        if fse_debug is True:
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

    if fse_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    s.sendto(str.encode(json_str), (server_ip, fse_port_comm))
    try:
        data, address = s.recvfrom(1024)

        if fse_debug is True:
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

    if fse_debug is True:
        Logger().print_trace("Send JSON Obj:", json_str)

    s.sendto(str.encode(json_str), (server_ip, fse_port_comm))
    try:
        data, address = s.recvfrom(1024)

        if fse_debug is True:
            Logger().print_trace("Received from {}:{}".format(address, data.decode("utf-8")))

        json_obj = json.loads(data.decode('utf-8'))

    except socket.timeout:  # fail after 1 second of no activity
        print("Didn't receive anymore data! [Timeout]")
