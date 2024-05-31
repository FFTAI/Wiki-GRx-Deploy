# Wiki-GRx-Deploy Demos

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

---

## RL Demos

### Preparation

To run the rl demo codes (`rl_stand.py` and `rl_walk.py`), you should follow the steps below:

1. Reinstall the robot IMU sensor, because the robot IMU sensor is installed upside down by default.
   But in our current repository lib, we set the IMU sensor to be installed upright.

![](./pictures/imu_upright.png)

(Need sometime to finish the compatibility of the IMU sensor installation upside down, working on it. 💪🏻)

2. Run the `demo_rl_stand.py` to make the robot stand up.
3. Run the `demo_rl_walk.py` to make the robot walk.

### demo_rl_stand.py

Before you run the demo code `demo_rl_walk.py`, you should run the `demo_rl_stand.py` to make the robot stand up.

To run the demo, execute the script using the command:

```
python demo_rl_stand.py ./config/config_xxx.yaml
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
python demo_rl_walk.py ./config/config_xxx.yaml
```

---

### PD Conversion

The actuators on GR1T1 and GR1T2 are with FSA v1 firmware, which only supports position, velocity, and current control.
So, if you want to use the PD control, you should convert the PD control parameters to the position control parameters.
Which means you should convert the PD control's kp and kd to the position control loop's kp and velocity control loop's kp.

We provide a script to convert the PD control parameters to the position control parameters in the `tools` folder named `pd_conversion.py`.

You can input the PD control's kp and kd in your simulation environment,
and then use this script to do the conversion.

```
python pd_conversion.py
```

---

### More Info about Robot_RCS and Robot_RCS_GR

More information about Robot_RCS and Robot_RCS_GR framework can be found in the online document:

- https://github.com/FFTAI/fftai.github.io?tab=readme-ov-file.

(Still in progress... 💪🏻)


---

Thank you for your interest in the Fourier Intelligence GRx Robot Repositories.
We hope you find this resource helpful in your robotics projects!