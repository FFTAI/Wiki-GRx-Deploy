# Wiki-GRx-Deploy

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

---

## Calibration

The first step to run the demo code, is to make sure the `sensor_offset.json` is filled with your machine absolute encoder value,
instead of the value in this repository.

Every machine has its own home position encoder value,
which should be set with the machine power on and the joint fixed at the pin position.

After you have finished the machine physical calibration process (move all joints to the pin position),
you can run the demo code in this repository, this demo code will record its absolute encoder value and stored in the `sensor_offset.json` file.

```
python demo_calibration.py ./config/config_xxx.yaml
```

## Info level

The print level of the information are separated into 5 levels:

- Info Level: white color word, for general information.
- Highlight Level: green color word, for important information.
- Debug Level: red color word, for debug information.
- Warning Level: blue color word, yellow background, for warning information.
- Error Level: white color word, red background, for error information.

---

## GRx Series Robot

Different robot models have different configurations, and the demo code is divided into different parts according to the robot model.
But all config files are all in the `config` folder.

When you run the demo code, you should specify the config file in the command line.

### GR1

#### GR1-T1

run demo with following command:

```
python demo_xxx.py ./config/config_GR1_T1.yaml
```

#### GR1-T2

run demo with following command:

```
python demo_xxx.py ./config/config_GR1_T2.yaml
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
python demo.py ./config/config_xxx.yaml
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

#### Running the Demo

To run the demo, execute the script using the command:

```
python demo_rl_walk.py ./config/config_xxx.yaml
```

#### Training Procedure

The training code for getting the policy `.pt` file, allowing the robot to walk is from repository:

- Gitee: https://gitee.com/FourierIntelligence/wiki-grx-gym.
- Github: https://github.com/FFTAI/Wiki-GRx-Gym.

---

### More Info about Robot_RCS and Robot_RCS_GR

More information about Robot_RCS and Robot_RCS_GR framework can be found in the online document:

- https://github.com/FFTAI/fftai.github.io?tab=readme-ov-file.

(Still in progress...)

---

Thank you for your interest in the Fourier Intelligence GRx Robot Repositories.
We hope you find this resource helpful in your robotics projects!