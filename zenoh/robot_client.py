import threading
import time

import msgpack_numpy as m
import numpy as np
import zenoh
from rich.console import Console
from rich.pretty import pprint
from rich.prompt import Confirm, Prompt
from rich.table import Table
from robot_rcs_gr.sdk import ControlGroup, RobotClient

m.patch() # Patch msgpack_numpy to handle numpy arrays
zenoh.init_logger()  # Initialize zenoh logger

# Initialize console for rich logging and printing
console = Console() 
log = console.log
print = console.print

# Set control frequency
FREQ = 150


def record(client: RobotClient):
    '''
    How to use the record function:
    1. Move to the start position and press enter to set the start position.
    2. Press enter to start recording; robot arms can move freely.
    3. Move to the final position and press enter again to finish recording.
    4. The trajectory will be stored as a npy file; use play to replay it.
    '''

    traj = []

    # Disable the force applied to the motor so the robot can move freely
    client.set_enable(False) 

    time.sleep(1)

    # Prompt user to move to start position
    reply = Prompt.ask("Move to start position and press enter")
    if reply == "":
        client.update_pos() # Update position before enabling the motor
        time.sleep(0.1)
        client.set_enable(True) # # Enable force applied into motor, and motor could not move freely 
        time.sleep(1)

        # Print out joint information at the starting position
        for sensor_type, sensor_data in client.states.items():
            for sensor_name, sensor_reading in sensor_data.items():
                if sensor_type == "joint":
                    print(sensor_type + "/" + sensor_name, sensor_reading.tolist())
    else:
        return
    time.sleep(0.5)

    # Confirm if the user wants to start recording
    reply = Confirm.ask("Start recording?")

    if not reply:
        return
    

    '''
    Two threads aiming to:
    1. Keep adding the joint positions to the traj list.
    2. Prompt the user to stop recording.
    '''

    # client.update_pos()
    client.set_enable(False)
    time.sleep(1)
    event = threading.Event()

    def task():
        while not event.is_set():
            client.loop_manager.start()
            traj.append(client.joint_positions.copy())
            client.loop_manager.end()
            client.loop_manager.sleep()

    # Start recording thread
    thread = threading.Thread(target=task)
    thread.daemon = True
    thread.start()

    # Prompt user to stop recording
    reply = Prompt.ask("Press enter to stop recording")
    if reply == "":
        event.set()
        thread.join()

        client.update_pos()
        time.sleep(0.1)
        client.set_enable(True)

        # Save recorded trajectory
        np.save("record.npy", traj)
        return traj


def play(recorded_traj: list[np.ndarray], client: RobotClient):
    '''
    Move_joint function:
    Three argument: joint position, time duration for movement, and blocking
    Notice: Could access other funciton while blcoking = True
    time duration will change the robot moving speed, it means the time that robot take to finish the task
    '''

    client.set_enable(True)
    time.sleep(1)

    # Move to the first position
    first = recorded_traj[0]
    client.move_joints(ControlGroup.ALL, first, 2.0, blocking=True)

    # Move along the trajectory stored in the list
    for pos in recorded_traj[1:]:
        client._move_to(pos)
        time.sleep(1 / FREQ)
    time.sleep(1)

    # Disable the force applied into motor
    client.set_enable(False)
    


if __name__ == "__main__":
    client = RobotClient(FREQ)
    time.sleep(0.5)
    while True:
        task = Prompt.ask(
            "What do you want the :robot: to do?",
            choices=[
                "enable", # Enable the force applied into motor
                "disable", # Disable the force applied into motor 
                "set_home", # Get sensor offsets and save to `sensor_offset.json`, it could be used to calibrating all the absolute encoder
                "set_zero", # Reboot all the motor and go back to zero position
                "print_states", # Print out the motor status and information
                "move_to_default", # Move to default position
                "record", # Recording the movement of the robot joint as a npy file
                "play", # Replay the task recorded in the record.npy
                "abort", # Stop any movement that robot is doing right now
                "exit", # Exit the robot client control
            ],
        )

        if task == "enable": 
            client.set_enable(True)

        elif task == "disable":
            client.set_enable(False)
        
        elif task == "set_home":
            client.set_home()

        elif task == "set_zero":
            client.reboot()

        elif task == "move_to_default":
            client.set_enable(True)
            time.sleep(0.5)

            # move joint to default position that defined for each robot
            # while blocking = False means user could not use any function before the movement done
            client.move_joints(
                ControlGroup.LEFT_ARM,
                client.default_positions[ControlGroup.LEFT_ARM.slice],
                2.0,
                blocking=False,
            )
        
        elif task == "abort": 
            client.abort()

        elif task == "print_states":
            #
            table = Table("Type", "Data", title="Current :robot: state")
            for sensor_type, sensor_data in client.states.items():
                for sensor_name, sensor_reading in sensor_data.items():
                    print(sensor_type + "/" + sensor_name, sensor_reading.tolist())
                    table.add_row(
                        sensor_type + "/" + sensor_name,
                        str(np.round(sensor_reading, 3)),
                    )
            print(table)

        elif task == "record": 
            traj = record(client)
            pprint(traj)
        
        elif task == "play": 
            rec = np.load("record.npy", allow_pickle=True)
            play(rec, client)
        
        elif task == "exit": 
            import sys

            client.close()
            sys.exit(0)
        time.sleep(0.5)


    # client.spin()
    # time.sleep(1)
    # client.set_enable(False)
