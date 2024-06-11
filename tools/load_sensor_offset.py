import os
import json

home_path = os.path.expanduser("~")

# Note: this path is for the control SDK installed on the host machine, may change depending on the installation path
origin_sensor_offset_file_path = home_path + "/RoCS/bin/pythonscripts/absAngle.json"
origin_sensor_offset_dict = {}
target_sensor_offset_file_path = "./sensor_offset.json"
target_sensor_offset_dict = {
    # for GR1T1 and GR1T2
    "192.168.137.170": None,
    "192.168.137.171": None,
    "192.168.137.172": None,
    "192.168.137.173": None,
    "192.168.137.174": None,
    "192.168.137.175": None,
    "192.168.137.150": None,
    "192.168.137.151": None,
    "192.168.137.152": None,
    "192.168.137.153": None,
    "192.168.137.154": None,
    "192.168.137.155": None,
    "192.168.137.190": None,
    "192.168.137.191": None,
    "192.168.137.192": None
}


def load_sensor_offset():
    global origin_sensor_offset_dict, target_sensor_offset_dict

    # load origin sensor offset dict
    print("Loading origin sensor offset file at path: ", origin_sensor_offset_file_path)

    if os.path.exists(origin_sensor_offset_file_path):
        with open(origin_sensor_offset_file_path, "r") as f:
            origin_sensor_offset_dict = json.load(f)
    else:
        print("Sensor offset file not found. Please check the path!")
        return

    print("Origin sensor offset loaded: ")
    print(json.dumps(origin_sensor_offset_dict, indent=4))

    # update target sensor offset dict
    for key in target_sensor_offset_dict:
        target_sensor_offset_dict[key] = origin_sensor_offset_dict[key]["angle"]

    print("Target sensor offset updated: ")
    print(json.dumps(target_sensor_offset_dict, indent=4))

    # save target sensor offset dict
    print("Saving target sensor offset file at path: ", target_sensor_offset_file_path)
    with open(target_sensor_offset_file_path, "w") as f:
        json.dump(target_sensor_offset_dict, f, indent=4)


if __name__ == "__main__":
    load_sensor_offset()
