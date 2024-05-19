# Wiki-GRx

![](pictures/1.png)

## Setup Environment

1. Install Conda environment
    - Miniconda download:
        - `https://docs.anaconda.com/free/miniconda/index.html`
    - Miniconda install:
        - `bash ./Miniconda3-latest-Linux-x86_64.sh`

2. Create conda environment:
    - Notice: we should create the environment with Python 3.11, because all the libraries are created with such version.
        - `conda create -n wiki-grx python=3.11`

3. Install necessary environment:
    - Use `requirements.txt`:
        - `pip install -r ./requirements.txt`

Till now, if no error presents, the environment for develop Fourier GRx robot is ready.
You can now run the demos in the repository.

## Calibration

The first step to run the demo code, is to make sure the `sensor_offset.txt` is with your machine encoder value,
instead of the value in this repository.

Every machine has its own home position encoder value,
which should be set with the machine power on and the joint fixed at the pin position.

After you have finished the calibration process, you can run the demo code in this repository.

```
python demo_calibration.py config_xxx.yaml
```

---

## GRx Series Robot

### GR1

#### GR1-T1

run demo with following command:

```
python demo_xxx.py config_GR1_T1.yaml
```

#### GR1-T2

run demo with following command:

```
python demo_xxx.py config_GR1_T2.yaml
```

--- 

## Demos

### demo.py

This demo script establishes a control system for a robotic unit, setting a desired control frequency, managing the robot state, and executing control commands based on this state.
The control loop is configured to run indefinitely, continuously adjusting the robot's parameters in real time.

#### Detailed Explanation

- Imports and Setup:
    - The script begins by importing necessary modules: sys, time, and specific versions of robot_rcs and robot_rcs_gr.
    - It prints out the versions of these imported modules to ensure the correct versions are being used.

- Control System Initialization:
    - ControlSystem and RobotInterface classes are imported, which are pivotal for managing the robot's control systems and interfacing with the hardware.
    - A target control frequency of 50Hz is defined, meaning the control loop will execute 50 times per second.
    - The control system is set to development mode using ControlSystem().dev_mode().

- Main Control Loop:
    - Two dictionaries, state_dict and control_dict, are initialized to store the current state of the robot and the control commands respectively.
    - State Update:
      The robot's current state is obtained through RobotInterface().instance.control_loop_intf_get_state().
      This state includes various parameters such as IMU data (quaternion, Euler angles, angular velocity, linear acceleration), joint data (position, velocity, torque), and base data (position, velocity).
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
python demo.py config_xxx.yaml
```

### More Info about Robot_RCS and Robot_RCS_GR
