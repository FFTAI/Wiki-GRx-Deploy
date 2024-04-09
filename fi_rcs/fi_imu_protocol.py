#!/usr/bin/env python3
# -*- coding: utf-8 -*-

' fi protocol module '

import threading
import struct
import datetime
import time


class FIFrame_Exception(Exception):
    def __init__(self, err='Frame Error'):
        Exception.__init__(self, err)


class FIFrame_NoValid_Exception(FIFrame_Exception):
    def __init__(self, err='No valid frame received'):
        Exception.__init__(self, err)


class FIFrame_NotCompleted_Exception(FIFrame_Exception):
    def __init__(self, err='No full frame received'):
        Exception.__init__(self, err)


class FIFrame_ErrorFrame_Exception(FIFrame_Exception):
    def __init__(self, err='Error frame'):
        Exception.__init__(self, err)


def _parse_data_packet_0x90(data_section: list, node_num=None):
    module_id = {
        "id": data_section[0],
    }
    return module_id


def _parse_data_packet_0xD1(data_section: list, node_num=None):
    quaternion_list = []

    # for pos in range(node_num):
    pos = 0
    t_pos = pos * 16
    W = float(struct.unpack("<f", bytes(data_section[t_pos:t_pos + 4]))[0])
    t_pos += 4
    X = float(struct.unpack("<f", bytes(data_section[t_pos:t_pos + 4]))[0])
    t_pos += 4
    Y = float(struct.unpack("<f", bytes(data_section[t_pos:t_pos + 4]))[0])
    t_pos += 4
    Z = float(struct.unpack("<f", bytes(data_section[t_pos:t_pos + 4]))[0])

    temp_dic = {
        "W": round(W, 3),
        "X": round(X, 3),
        "Y": round(Y, 3),
        "Z": round(Z, 3)
    }
    quaternion_list.append(temp_dic)

    quaternion = {
        "quat": quaternion_list
    }

    return quaternion


def _parse_data_packet_0xA0(data_section: list, node_num=None):
    acc_list = []

    # for pos in range(node_num):
    pos = 0
    t_pos = pos * 6
    X = int(struct.unpack("<h", bytes(data_section[t_pos:t_pos + 2]))[0])
    t_pos += 2
    Y = int(struct.unpack("<h", bytes(data_section[t_pos:t_pos + 2]))[0])
    t_pos += 2
    Z = int(struct.unpack("<h", bytes(data_section[t_pos:t_pos + 2]))[0])

    temp_dic = {
        "X": round(X, 8),
        "Y": round(Y, 8),
        "Z": round(Z, 8)
    }
    acc_list.append(temp_dic)

    acc = {
        "acc": acc_list
    }

    return acc


def _parse_data_packet_0xB0(data_section: list, node_num=None):
    gyr_list = []

    # for pos in range(node_num):
    pos = 0
    t_pos = pos * 6
    X = int(struct.unpack("<h", bytes(data_section[t_pos:t_pos + 2]))[0])
    t_pos += 2
    Y = int(struct.unpack("<h", bytes(data_section[t_pos:t_pos + 2]))[0])
    t_pos += 2
    Z = int(struct.unpack("<h", bytes(data_section[t_pos:t_pos + 2]))[0])

    temp_dic = {
        "X": round(X, 8),
        "Y": round(Y, 8),
        "Z": round(Z, 8)
    }
    gyr_list.append(temp_dic)

    gyr = {
        "gyr": gyr_list
    }

    return gyr


def _parse_data_packet_0xC0(data_section: list, node_num=None):
    mag_list = []

    # for pos in range(node_num):
    pos = 0
    t_pos = pos * 6
    X = int(struct.unpack("<h", bytes(data_section[t_pos:t_pos + 2]))[0])
    t_pos += 2
    Y = int(struct.unpack("<h", bytes(data_section[t_pos:t_pos + 2]))[0])
    t_pos += 2
    Z = int(struct.unpack("<h", bytes(data_section[t_pos:t_pos + 2]))[0])

    temp_dic = {
        "X": round(X, 3),
        "Y": round(Y, 3),
        "Z": round(Z, 3)
    }
    mag_list.append(temp_dic)

    mag = {
        "mag": mag_list
    }

    return mag


def _parse_data_packet_0xD0(data_section: list, node_num=None):
    eul_list = []

    # for pos in range(node_num):
    pos = 0
    t_pos = pos * 6
    Pitch = struct.unpack("<h", bytes(data_section[t_pos:t_pos + 2]))[0]
    Pitch = Pitch / 100
    t_pos += 2
    Roll = struct.unpack("<h", bytes(data_section[t_pos:t_pos + 2]))[0]
    Roll = Roll / 100
    t_pos += 2
    Yaw = struct.unpack("<h", bytes(data_section[t_pos:t_pos + 2]))[0]
    Yaw = Yaw / 10

    temp_dic = {
        "Pitch": round(Pitch, 8),
        "Roll": round(Roll, 8),
        "Yaw": round(Yaw, 8)
    }
    eul_list.append(temp_dic)

    euler = {
        "euler": eul_list
    }

    return euler


def _parse_data_packet_0x91(data_section: list, node_num=None):
    pos = 0
    id_temp_list = []
    timestamp_temp_list = []
    acc_temp_list = []
    gyr_temp_list = []
    mag_temp_list = []
    eul_temp_list = []
    quat_temp_list = []

    # id
    id = data_section[pos]
    id_dic = {
        "": id
    }

    id_temp_list.append(id_dic)
    pos += 1
    # reserved
    pos += 6
    # timestamp
    timestamp = int(struct.unpack("<I", bytes(data_section[pos:pos + 4]))[0])
    timestamp_dic = {
        "(s)": round(timestamp / 1000, 3)
    }

    timestamp_temp_list.append(timestamp_dic)
    pos += 4
    # acc
    acc_X = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    acc_Y = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    acc_Z = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    acc_dic = {
        "X": round(acc_X, 8),
        "Y": round(acc_Y, 8),
        "Z": round(acc_Z, 8)
    }
    acc_temp_list.append(acc_dic)
    # gyr
    gyr_X = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    gyr_Y = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    gyr_Z = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    gyr_dic = {
        "X": round(gyr_X, 8),
        "Y": round(gyr_Y, 8),
        "Z": round(gyr_Z, 8)
    }
    gyr_temp_list.append(gyr_dic)
    # mag
    mag_X = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    mag_Y = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    mag_Z = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    mag_dic = {
        "X": round(mag_X),
        "Y": round(mag_Y),
        "Z": round(mag_Z)
    }
    mag_temp_list.append(mag_dic)
    # eul
    eul_Roll = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    eul_Pitch = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    eul_Yaw = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    eul_dic = {
        "Roll": round(eul_Roll, 8),
        "Pitch": round(eul_Pitch, 8),
        "Yaw": round(eul_Yaw, 8)
    }
    eul_temp_list.append(eul_dic)

    # quat
    quat_W = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    quat_X = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    quat_Y = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    quat_Z = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
    pos += 4
    quat_dic = {
        "W": round(quat_W, 3),
        "X": round(quat_X, 3),
        "Y": round(quat_Y, 3),
        "Z": round(quat_Z, 3),
    }
    quat_temp_list.append(quat_dic)

    temp_dic_list = {
        "id": id_temp_list,
        "timestamp": timestamp_temp_list,
        "acc": acc_temp_list,
        "gyr": gyr_temp_list,
        "mag": mag_temp_list,
        "euler": eul_temp_list,
        "quat": quat_temp_list
    }
    return temp_dic_list


def _parse_data_packet_0x93(data_section: list, node_num=None):
    pos = 0
    id_temp_list = []
    timestamp_temp_list = []
    acc_temp_list = []
    gyr_temp_list = []

    # id
    id = data_section[pos]
    id_temp_list.append(id)
    pos += 1
    # reserved
    pos += 6
    # timestamp
    timestamp = int(struct.unpack("<I", bytes(data_section[pos:pos + 4]))[0])
    timestamp_temp_list.append(timestamp)
    pos += 4
    # acc
    acc_X = int(struct.unpack("<h", bytes(data_section[pos:pos + 2]))[0])
    pos += 2
    acc_Y = int(struct.unpack("<h", bytes(data_section[pos:pos + 2]))[0])
    pos += 2
    acc_Z = int(struct.unpack("<h", bytes(data_section[pos:pos + 2]))[0])
    pos += 2
    acc_dic = {
        "X": acc_X,
        "Y": acc_Y,
        "Z": acc_Z
    }
    acc_temp_list.append(acc_dic)
    # gyr
    gyr_X = int(struct.unpack("<h", bytes(data_section[pos:pos + 2]))[0])
    pos += 2
    gyr_Y = int(struct.unpack("<h", bytes(data_section[pos:pos + 2]))[0])
    pos += 2
    gyr_Z = int(struct.unpack("<h", bytes(data_section[pos:pos + 2]))[0])
    pos += 2
    gyr_dic = {
        "X": gyr_X,
        "Y": gyr_Y,
        "Z": gyr_Z
    }
    gyr_temp_list.append(gyr_dic)

    temp_dic = {
        "id": id_temp_list,
        "timestamp": timestamp_temp_list,
        "acc": acc_temp_list,
        "gyr": gyr_temp_list,
    }
    return temp_dic


rel_node_num = 0
module_node_num = 0


def _parse_data_packet_0x62(data_section: list, node_num=None):
    global rel_node_num
    global module_node_num
    global data_packet_properties
    id_temp_list = []
    gwid_temp_list = []
    timestamp_temp_list = []
    acc_temp_list = []
    gyr_temp_list = []
    mag_temp_list = []
    eul_temp_list = []
    quat_temp_list = []
    pos = 0

    gwid = data_section[0]
    id_dic = {
        "": gwid
    }
    gwid_temp_list.append(id_dic)

    cnt = data_section[1]
    rel_node_num = cnt
    module_node_num = cnt
    data_packet_properties[0x62]["data_len"] = 5 + (76 * cnt)
    pos += 2
    # reserved
    pos += 5
    # 0x91 packet
    for node in range(cnt):
        # packet id
        pos += 1

        # id
        id = data_section[pos]
        id_dic = {
            "": id
        }
        id_temp_list.append(id_dic)
        pos += 1
        # reserved
        pos += 6
        # timestamp
        timestamp = int(struct.unpack("<I", bytes(data_section[pos:pos + 4]))[0])
        timestamp_dic = {
            "(s)": round(timestamp / 1000, 3)
        }

        timestamp_temp_list.append(timestamp_dic)
        pos += 4

        # acc
        acc_X = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        acc_Y = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        acc_Z = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        acc_dic = {
            "X": round(acc_X, 8),
            "Y": round(acc_Y, 8),
            "Z": round(acc_Z, 8)
        }
        acc_temp_list.append(acc_dic)
        # gyr
        gyr_X = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        gyr_Y = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        gyr_Z = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        gyr_dic = {
            "X": round(gyr_X, 8),
            "Y": round(gyr_Y, 8),
            "Z": round(gyr_Z, 8)
        }
        gyr_temp_list.append(gyr_dic)
        # mag
        mag_X = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        mag_Y = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        mag_Z = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        mag_dic = {
            "X": round(mag_X),
            "Y": round(mag_Y),
            "Z": round(mag_Z)
        }
        mag_temp_list.append(mag_dic)
        # eul
        eul_Roll = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        eul_Pitch = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        eul_Yaw = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        eul_dic = {
            "Roll": round(eul_Roll, 8),
            "Pitch": round(eul_Pitch, 8),
            "Yaw": round(eul_Yaw, 8)
        }
        eul_temp_list.append(eul_dic)

        # quat
        quat_W = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        quat_X = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        quat_Y = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        quat_Z = float(struct.unpack("<f", bytes(data_section[pos:pos + 4]))[0])
        pos += 4
        quat_dic = {
            "W": round(quat_W, 3),
            "X": round(quat_X, 3),
            "Y": round(quat_Y, 3),
            "Z": round(quat_Z, 3),
        }
        quat_temp_list.append(quat_dic)

        temp_dic_list = {
            "id": id_temp_list,
            "timestamp": timestamp_temp_list,
            "acc": acc_temp_list,
            "gyr": gyr_temp_list,
            "mag": mag_temp_list,
            "euler": eul_temp_list,
            "quat": quat_temp_list
        }

    temp_dic = {
        "GWD": gwid_temp_list,
        "id": id_temp_list,
        "timestamp": timestamp_temp_list,
        "acc": acc_temp_list,
        "gyr": gyr_temp_list,
        "mag": mag_temp_list,
        "euler": eul_temp_list,
        "quat": quat_temp_list
    }
    return temp_dic


rel_node_num = 0
module_node_num = 0


# only for 400hz acc and gyro
def _parse_data_packet_0x63(data_section: list, node_num=None):
    global rel_node_num
    global module_node_num
    global data_packet_properties
    id_temp_list = []
    timestamp_temp_list = []
    acc_temp_list = []
    gyr_temp_list = []

    pos = 0
    gwid = data_section[0]
    cnt = data_section[1]
    rel_node_num = cnt
    module_node_num = cnt
    data_packet_properties[0x63]["data_len"] = 5 + (24 * cnt)
    pos += 2
    # reserved
    pos += 5
    # 0x93 packet
    for node in range(cnt):
        # packet id
        pos += 1

        # id
        id = data_section[pos]
        id_dic = {
            "": id
        }
        id_temp_list.append(id_dic)
        pos += 1
        # reserved
        pos += 6
        # timestamp
        timestamp = int(struct.unpack("<I", bytes(data_section[pos:pos + 4]))[0])
        timestamp_dic = {
            "(s)": round(timestamp / 1000, 3)
        }

        timestamp_temp_list.append(timestamp_dic)
        pos += 4

        # acc
        acc_X = float(struct.unpack("<h", bytes(data_section[pos:pos + 2]))[0]) / 1000
        pos += 2
        acc_Y = float(struct.unpack("<h", bytes(data_section[pos:pos + 2]))[0]) / 1000
        pos += 2
        acc_Z = float(struct.unpack("<h", bytes(data_section[pos:pos + 2]))[0]) / 1000
        pos += 2

        acc_dic = {
            "X": round(acc_X, 8),
            "Y": round(acc_Y, 8),
            "Z": round(acc_Z, 8)
        }
        acc_temp_list.append(acc_dic)
        # gyr
        gyr_X = float(struct.unpack("<h", bytes(data_section[pos:pos + 2]))[0]) / 10
        pos += 2
        gyr_Y = float(struct.unpack("<h", bytes(data_section[pos:pos + 2]))[0]) / 10
        pos += 2
        gyr_Z = float(struct.unpack("<h", bytes(data_section[pos:pos + 2]))[0]) / 10
        pos += 2
        gyr_dic = {
            "X": round(gyr_X, 8),
            "Y": round(gyr_Y, 8),
            "Z": round(gyr_Z, 8)
        }
        gyr_temp_list.append(gyr_dic)

    temp_dic = {
        "GWD": gwid_temp_list,
        "id": id_temp_list,
        "timestamp": timestamp_temp_list,
        "acc": acc_temp_list,
        "gyr": gyr_temp_list,
    }
    return temp_dic


data_packet_properties = {
    # id
    0x90: {
        "type": "id",
        "id_len": 1,
        "data_len": 1,
        "parse method": _parse_data_packet_0x90,
        "gw_data": False
    },

    # acc
    0xA0: {
        "type": "acc",
        "id_len": 1,
        "data_len": 6,
        "parse method": _parse_data_packet_0xA0,
        "gw_data": False
    },
    # gyr
    0xB0: {
        "type": "gyr",
        "id_len": 1,
        "data_len": 6,
        "parse method": _parse_data_packet_0xB0,
        "gw_data": False
    },
    # mag
    0xC0: {
        "type": "mag",
        "id_len": 1,
        "data_len": 6,
        "parse method": _parse_data_packet_0xC0,
        "gw_data": False
    },
    # float_eul
    0xD0: {
        "type": "euler",
        "id_len": 1,
        "data_len": 6,
        "parse method": _parse_data_packet_0xD0,
        "gw_data": False
    },
    # quat
    0xD1: {
        "type": "quat",
        "id_len": 1,
        "data_len": 16,
        "parse method": _parse_data_packet_0xD1,
        "gw_data": False
    },
    # imusol
    0x91: {
        "type": "imusol",
        "id_len": 1,
        "data_len": 76,
        "parse method": _parse_data_packet_0x91,
        "gw_data": False
    },
    # gwimusol
    0x62: {
        "type": "gwsol",
        "id_len": 1,
        "data_len": 76 * 1,
        "parse method": _parse_data_packet_0x62,
        "gw_data": True
    },

    # imusol only raw
    0x93: {
        "type": "imusol_raw",
        "id_len": 1,
        "data_len": 24,
        "parse method": _parse_data_packet_0x93,
        "gw_data": False
    },

    # gwsol only raw
    0x63: {
        "type": "gwsol_raw",
        "id_len": 1,
        "data_len": 24 * 1,
        "parse method": _parse_data_packet_0x63,
        "gw_data": True
    }
}


def crc16_update(buffer_list, cal_len, cal_pos, crc=0):
    for temp_j in range(cal_len):
        byte = buffer_list[temp_j + cal_pos]
        crc ^= byte << 8
        crc &= 0xffffffff
        for temp_i in range(8):
            temp = crc << 1
            temp &= 0xffffffff
            if (crc & 0x8000):
                temp ^= 0x1021
                temp &= 0xffffffff
            crc = temp

    return (crc & 0xffff)


SampleRate = 0
SamplesReceived = 0
prevSamplesReceived = 0
sample_rate_alive_flag = True


def sample_rate_timer_cb(sample_timer):
    global SampleRate, SamplesReceived, prevSamplesReceived, sample_rate_alive_flag

    SampleRate = SamplesReceived - prevSamplesReceived
    prevSamplesReceived = SamplesReceived
    print("每秒幀率：", SampleRate, datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f'))

    if sample_rate_alive_flag == True:
        sample_timer = threading.Timer(1.00, sample_rate_timer_cb, args=(sample_timer,))
        sample_timer.start()


def sample_rate_timer_close():
    global sample_rate_alive_flag
    sample_rate_alive_flag = False


# 找到幀頭
def find_frameheader(buffer_list: list):
    # 循環查找，直至拋出異常
    while True:
        # 查找幀頭的第一個標識符0x5a,若未找到，將會拋出ValueError異常
        try:
            header_ind = buffer_list.index(0x5a)
        except ValueError:
            raise FIFrame_NotCompleted_Exception

        if header_ind + 1 > len(buffer_list) - 1:
            raise FIFrame_NotCompleted_Exception

        if buffer_list[header_ind + 1] == 0xa5:
            # 找到幀頭標識符0x5aa5，返回幀頭位置
            return header_ind
        else:
            # 未找到幀頭標識符，切片繼續查找
            buffer_list = buffer_list[header_ind + 1:]


# 驗證獲取長度
def _get_frame_length(buffer_list, header_pos):
    return int(struct.unpack("<h", bytes(buffer_list[header_pos + 2:header_pos + 4]))[0])


# 驗證長度是否合法
def _verify_frame_length(buffer_list: list, header_pos):
    # 獲取到幀長度
    frame_len = int(struct.unpack("<h", bytes(buffer_list[header_pos + 2:header_pos + 4]))[0])
    # 判斷幀長度是否合法
    if frame_len >= 1024:
        raise FIFrame_ErrorFrame_Exception
    elif frame_len + header_pos + 6 > len(buffer_list):
        raise FIFrame_NotCompleted_Exception


# 驗證crc是否正確
def _verify_frame_crc(buffer_list, header_pos=0):
    # 獲取到幀長度
    frame_len = int(struct.unpack("<h", bytes(buffer_list[header_pos + 2:header_pos + 2 + 2]))[0])
    # 獲取幀內的crc
    f_crc = int(struct.unpack("<H", bytes(buffer_list[header_pos + 4:header_pos + 4 + 2]))[0])
    # 計算幀的crc
    cal_crc = crc16_update(buffer_list, 4, header_pos, 0)
    cal_crc = crc16_update(buffer_list, frame_len, header_pos + 6, cal_crc)

    if cal_crc != f_crc:
        raise FIFrame_ErrorFrame_Exception


# 截取一條完整且合法的幀，並將幀頭幀尾返回
def intercept_one_complete_frame(buffer_list):
    # 找幀頭
    header_pos = find_frameheader(buffer_list)
    try:
        frame_len = int(struct.unpack("<H", bytes(buffer_list[header_pos + 2:header_pos + 2 + 2]))[0])
    except struct.error:
        raise FIFrame_NotCompleted_Exception
    end_pos = header_pos + 5 + frame_len
    # 驗證幀長度
    _verify_frame_length(buffer_list, header_pos)
    # 驗證crc
    _verify_frame_crc(buffer_list, header_pos)

    return header_pos, end_pos


# 從完整幀中獲取資訊
def extraction_information_from_frame(frame_list: list, inf_fifo, report_datatype: dict = None):
    # 幀率統計
    global SamplesReceived
    global rel_node_num
    global module_node_num
    SamplesReceived = SamplesReceived + 1
    # 處理數據幀
    data_dic = {}
    pos = 0

    data_frame_list = frame_list[6:]

    # 遍歷解析數據段內包含的數據
    while len(data_frame_list) > 0:
        if data_frame_list[0] in data_packet_properties:

            temp_dic = data_packet_properties[data_frame_list[0]]["parse method"](data_frame_list[1:], module_node_num)

            try:
                if report_datatype[data_packet_properties[data_frame_list[0]]["type"]] == True:
                    data_dic.update(temp_dic)
                else:
                    pass

            except KeyError:
                pass

            if data_packet_properties[data_frame_list[0]]["gw_data"] == True:
                rel_node_num = module_node_num
            else:
                rel_node_num = 1

            id_len = data_packet_properties[data_frame_list[0]]["id_len"]

            if data_frame_list[0] == 0x62:
                data_len = 76 * rel_node_num + 8
            elif data_frame_list[0] == 0x63:
                data_len = 24 * rel_node_num + 8

            else:
                data_len = data_packet_properties[data_frame_list[0]]["data_len"] * rel_node_num

            data_frame_list = data_frame_list[id_len + data_len:]
        else:
            # raise FIFrame_ErrorFrame_Exception
            data_frame_list = data_frame_list[1:]

    inf_fifo.put(data_dic)
