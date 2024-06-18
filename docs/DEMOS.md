# Wiki-GRx-Deploy Demos

## Calibration

The first step to run the demo code, is to make sure the `sensor_offset.json` is filled with your machine absolute encoder value,
instead of the value in this repository (may be different from your machine).

Every machine has its own home position encoder value,
which should be set with the machine power on and the joint fixed at the pin position.

After you have finished the machine physical calibration process (move all joints to the pin position),
you can run the demo code in this repository, this demo code will record its absolute encoder value and stored in the `sensor_offset.json` file.

```
python demo_set_home.py --rcs_config=./config/config_xxx.yaml
```

## Efficient Core Disable

To have a better performance of the robot control system, we suggest disable the efficient cores of the computer.

To disable the efficient cores, you need to enter the BIOS of your computer, and disable the efficient core.
On GR1T1 and GR1T2, you can press the `esc` key when the computer is booting up to enter the BIOS.

In the BIOS, you can find the efficient core setting, and set the number of efficient cores to 0.

## GRx Series Robot

Different robot models have different configurations, and the demo code is divided into different parts according to the robot model.
But all config files are all in the `config` folder.

When you run the demo code, you should specify the config file in the command line.

### GR1

#### GR1-T1

run demo with following command:

```
python demo_xxx.py --rcs_config=./config/config_GR1_T1.yaml
```

#### GR1-T2

run demo with following command:

```
python demo_xxx.py --rcs_config=./config/config_GR1_T2.yaml
```

---

## Demos

### demo.py

This demo script establishes a control system for a robotic unit, setting a desired control frequency, managing the robot state, and executing control commands based on this state.
The control loop is configured to run indefinitely, continuously adjusting the robot's parameters in real time.

#### Detailed Explanation

1. Imports and Setup:
    - The script begins by importing necessary modules: sys, time, and specific versions of robot_rcs and robot_rcs_gr.
    - It prints out the versions of these imported modules to ensure the correct versions are being used.

2. Control System Initialization:
    - ControlSystem and RobotInterface classes are imported, which are pivotal for managing the robot's control systems and interfacing with the hardware.
    - A target control frequency of `50Hz` is defined, meaning the control loop will execute 50 times per second.
    - The control system is set to development mode using `ControlSystem().dev_mode()`.

3. Main Control Loop:
    - Two dictionaries, state_dict and control_dict, are initialized to store the current state of the robot and the control commands respectively.
    - State Update:
      The robot's current state is obtained through RobotInterface().instance.control_loop_intf_get_state().
      This state includes various parameters such as IMU data (quaternion, Euler angles, angular velocity, linear acceleration), joint data (position, velocity, torque), and base data (position,
      velocity).
    - Control Commands:
      The control commands are updated in control_dict. This includes setting control modes, proportional gain (kp), derivative gain (kd), and desired positions for each joint of the robot.
      The control modes are set as PD (Proportional-Derivative) control, indicated by the mode value 5.
      Gains (kp and kd) are set to predefined values to define the responsiveness and damping of the control system.
      The positions for all joints are set to 0 degrees, implying the robot's default or neutral position.
    - Control Execution:
      The updated control commands are sent to the robot through RobotInterface().instance.control_loop_intf_set_control(control_dict).
    - Timing Management:
      The script calculates the time taken for one cycle of the control loop and determines the remaining time to sleep to maintain the desired frequency of 50Hz.
      If the loop takes longer than expected, it does not sleep to catch up.

#### Key Features and Goals

- Real-Time Control:
    - The script aims to ensure real-time responsiveness by maintaining a consistent control loop frequency.
- Modularity and Customization:
    - Users can customize the control algorithm and adjust parameters such as control gains and desired positions.
- Development Mode:
    - The development mode facilitates testing and debugging by providing more control and feedback during development.

- Potential Improvements
    - The code mentions a TODO for upgrading the control frequency to 1000Hz, suggesting future enhancement for more responsiveness.
    - There is also a placeholder for more accurate control frequency management, which can be implemented for precise timing.

#### Running the Demo

To run the demo, execute the script using the command:

```
python demo.py --rcs_config=./config/config_xxx.yaml
```

### demo_print_state.py

This demo script demonstrates how to print the robot's state information, including IMU data, joint data, and base data.
The script continuously prints the state information in the console, providing real-time feedback on the robot's status.

To run the demo, execute the script using the command:

```
python demo_print_state.py --rcs_config=./config/config_xxx.yaml
```

### demo_set_home.py

This demo script demonstrates how to set the robot's home position by recording the current joint positions as the home position.

To run the demo, execute the script using the command:

```
python demo_set_home.py --rcs_config=./config/config_xxx.yaml
```

### demo_servo_off.py

This demo script demonstrates how to turn off the robot's servo motors, effectively disabling the robot's movement.
The script sends a command to turn off the servo motors. Once the motors are off, the robot will lie down and please make sure the robot is in a safe position.

To run the demo, execute the script using the command:

```
python demo_servo_off.py --rcs_config=./config/config_xxx.yaml
```

### demo_servo_on.py

This demo script demonstrates how to turn on the robot's servo motors, enabling the robot to move.
The script sends a command to turn on the servo motors. Once the motors are on, all motors will be in high impedance mode, and the robot can be controlled.

To run the demo, execute the script using the command:

```
python demo_servo_on.py --rcs_config=./config/config_xxx.yaml
```

### demo_move_position.py

This demo script demonstrates how to make the robot move to the default standing position.
The script sends commands to set the desired joint positions for the robot to move.

To run the demo, execute the script using the command:

```
python demo_move_position.py ./config/config_xxx.yaml
```

## RL Demos

> **Notice**:
> - Currently, the RL demo codes are still under development, and the walking demo is not stable.
> - The RL demo codes right now is just a simple demo to show how to use the RL algorithm to control the robot.
> - We just want to provide **a simplest demo** to show how to use the RL algorithm to control the robot.
> - Its performance is not good enough, we are working on it to improve the performance.

### Preparation

To run the rl demo codes (`rl_stand.py` and `rl_walk.py`), you should follow the steps below:

1. Run the `demo_rl_stand.py` to make the robot stand up.
2. Run the `demo_rl_walk.py` to make the robot walk.

### demo_rl_stand.py

Before you run the demo code `demo_rl_walk.py`, you should run the `demo_rl_stand.py` to make the robot stand up.

To run the demo, execute the script using the command:

```
python demo_rl_stand.py --rcs_config=./config/config_xxx.yaml
```

### demo_rl_walk.py

This demo code outlines a sophisticated control loop for a robotic system, incorporating reinforcement learning to determine target joint positions.
It ensures real-time control with accurate sensor feedback and adaptable timing management.
This framework provides a robust foundation for further development and refinement of robotic control systems.

#### Detailed Explanation

1. Imports and Setup
    - Module Imports:
        - The script imports essential Python libraries such as os, sys, time, and torch for neural network operations.
        - It also imports specific versions of the robot_rcs and robot_rcs_gr modules, which include essential components for the robot control system and interface.

    - Version Information:
        - The script prints out the version information of the imported robot_rcs and robot_rcs_gr modules for validation.

    - Control System Initialization:
        - The control system and robot interface are initialized through the ControlSystem and RobotInterface classes.
        - The target control frequency is set to `50Hz`, defining the robot control loop's execution rate.
        - The control system is set to development mode using `ControlSystem().dev_mode()`.

2. Main Control Loop
    - State Update:
        - Within the control loop, the robot’s current state is obtained using RobotInterface().instance.control_loop_intf_get_state().
        - The state dictionary includes detailed information on the robot’s IMU data (quaternion, Euler angles, angular velocity, linear acceleration), joint data (position, velocity, torque), and
          base data (position and velocity).

    - RL Algorithm (algorithm_rl_walk):
        - The RL-based walking algorithm algorithm_rl_walk is used to calculate the joint target positions.
        - The function considers sensory inputs such as IMU data and joint measurements to compute the desired joint positions.

    - Control Commands:
        - The control commands are updated in control_dict which includes:
        - Control Mode (set to 4 for position control).
        - Proportional Gain (kp) and Derivative Gain (kd) values for various joints.
        - The desired joint positions as returned from the RL algorithm.

    - Control Execution:
        - The updated control commands are sent to the robot via RobotInterface().instance.control_loop_intf_set_control(control_dict).

    - Timing Management:
        - The script calculates the duration of each control loop cycle and determines the required sleep time to maintain the 50Hz control frequency.
        - A conditional ensures the script adjusts if the loop takes longer than expected.

3. RL Algorithm Functions
    - Load Actor Model:
        - If the actor model is not already loaded, the script loads it from a specified file using torch.load.
        - The actor is an instance of the ActorCriticMLP class, which predicts actions based on observations.

    - Compute Observations and Actions:
        - The function computes various tensors for IMU data, joint positions, and velocities.
        - An observation tensor is created which includes the command, projected gravity, IMU angular velocity, controlled joint positions and velocities, and the last action.
        - The actor uses these observations to predict the next action (target joint positions).

    - Update Joint Target Positions:
        - The predicted actions are converted to the joint target positions.
        - The positions are adjusted based on the default joint positions and are returned as a list.

To run the demo, execute the script using the command:

```
python demo_rl_walk.py --rcs_config=./config/config_xxx.yaml
```

---

### More Info about Robot_RCS and Robot_RCS_GR

More information about Robot_RCS and Robot_RCS_GR framework:

(Still in progress... 💪🏻)

## Info level

The print level of the information are separated into 5 levels:

- Info Level: white color word, for general information.
- Highlight Level: green color word, for important information.
- Debug Level: red color word, for debug information.
- Warning Level: blue color word, yellow background, for warning information.
- Error Level: white color word, red background, for error information. (**When happen, should stop the program and check the error.**
  )

---

Thank you for your interest in the Fourier Intelligence GRx Robot Repositories.
We hope you find this resource helpful in your robotics projects!