[简体中文](README.md) | English

# Fourier-GRX GRMini Development Interface Documentation

## Robot System

The control system of the Fourier-GRX GRMini robot runs on a main control computer with Ubuntu 22.04 LTS.

### Login Methods

#### Local Login
Connect an HDMI monitor and USB keyboard/mouse to the robot's control computer. The system desktop will automatically load after startup.

- **Username**: `gr2m25jaxxxx` (where `xxxx` is the last 4 digits of the robot's serial number)
- **Password**: `fftai2015`

#### Remote Login
After startup, the robot automatically activates a hotspot. Connect to this hotspot via a phone or computer:

- **Hotspot Name**: `gr2m25jaxxxx` (where `xxxx` is the last 4 digits of the serial number)
- **Hotspot Password**: `66668888`

Once connected, use SSH to log into the main control computer:
- **Username**: Same as hotspot name
- **Password**: `fftai2015`

> **Note**:  
> Some robots may not have auto-hotspot configured. In such cases, use the wired network port:
> - **Wired IP Address**: `192.168.137.220`  
> SSH credentials remain the same.

### Program Launch

Use the following command to start the robot control program:

```bash
# On the robot's control computer:
# 1. Connect a gamepad to the USB port.
# 2. Launch the main program
fourier-grx start
```

After startup, use the gamepad to control the robot. (The button mappings depend on the gamepad model.)

![joystick.jpg](picture/joystick.jpg)

### Secondary Development

#### Supported Environments

| OS               | Python Version | Tested | Passed |
|-------------------|----------------|--------|--------|
| Ubuntu 22.04 LTS  | Python 3.11    | ✅      | ✅      |
| Windows           | Python 3.11    |        |        |
| MacOS             | Python 3.11    |        |        |

#### Environment Setup

Use the following command to automatically configure the conda environment:

```bash
# On the robot's control computer:
fourier-grx setup_conda

# Activate the environment:
conda activate fourier-grx

# For manual setup, dependencies are available in:
$HOME/fourier-grx/whl
```

#### Example Code

Clone the example repository:

```bash
git clone https://gitee.com/FourierIntelligence/wiki-grx-mini
```

Recommended to clone into `$HOME` directory.

---

## Fourier-GRX APIs

Two types of APIs are provided:

- **User APIs**: High-level control via Zenoh (https://zenoh.io/) after starting `fourier-grx`.
- **Developer APIs**: Low-level hardware access (requires running on the robot's main control computer).

### User APIs

These APIs allow high-level control over the robot via Zenoh communication. They can be used from any computer on the same network.

#### Examples:
- `demo_servo_on`: Enable all joints.
- `demo_servo_off`: Disable all joints.
- `demo_clear_fault`: Clear joint errors.
- `demo_set_home`: Set current positions as joint zero positions.
- `demo_test_joint`: Joint motion test.
- `demo_ready_state`: Enter "ready" standing posture.
- `demo_rl_walk`: Enable walking mode (gamepad-controlled).

#### Running Examples

```bash
# On the robot's control computer:
conda activate fourier-grx
python $HOME/fourier-grx/whl/run.py --config=$HOME/fourier-grx/config/grmini1/config_GRMini1_{MODEL}_sdk.yaml

# On any networked computer:
conda activate fourier-grx
python $HOME/wiki-grx-mini/user/demo_{EXAMPLE}.py
```

### Developer APIs

These provide low-level hardware access and require execution on the robot's main control computer.

#### Examples:
- `demo_print_state`: Print robot status.
- `demo_servo_on/off`: Joint enable/disable.
- `demo_set_home`: Zero position calibration.
- `demo_set_pid`: Configure joint PID parameters.
- `demo_ready_state`: Ready posture.
- `demo_rl_walk`: Walking mode.

#### Running Examples

```bash
conda activate fourier-grx
python $HOME/wiki-grx-mini/developer/demo_{EXAMPLE}.py --config=$HOME/fourier-grx/config/grmini1/config_GRMini1_{MODEL}_sdk.yaml
```

---

## API Protocols (User)

### Zenoh Communication Keys

- `fourier-grx/dynalink_interface/task/server`: Task status monitoring
- `fourier-grx/dynalink_interface/grx/server`: Detailed robot status
- `fourier-grx/dynalink_interface/task/client`: Task commands
- `fourier-grx/dynalink_interface/grx/client`: Detailed control parameters

#### Task/Server Keys

| Key                    | Description               | Type  | Details                         |
|------------------------|---------------------------|-------|---------------------------------|
| `flag_task_in_process` | Task active flag          | bool  | 0: Inactive, 1: Active          |
| `robot_task_state`     | Current task state        | int   | Updated when `flag_task_command_update` is set |
| `robot_task_substate`  | Task substate             | int   |                                 |

#### GRX/Server Keys

| Key                        | Description           | Type  | Details                      |
|----------------------------|-----------------------|-------|------------------------------|
| `robot_error_codes`        | Error codes           | int   | 0: No error                  |
| `robot_battery_percentage` | Battery level (%)     | int   | 0-100                        |
| `robot_charging_level`     | Battery level indicator | int   | 1: Low, 2: Medium, 3: High   |
| `robot_charging_state`     | Charging status       | int   | 0: Not charging, 1: Charging |

#### Task/Client Keys

| Key                        | Description          | Type  | Details                     |
|----------------------------|----------------------|-------|-----------------------------|
| `flag_task_command_update` | Command update flag  | bool  | 0: No update, 1: Update     |
| `robot_task_command`       | Task command ID      | int   | See command table below     |

**Task Commands** (Partial):

| Command              | Value | Description                          |
|----------------------|-------|--------------------------------------|
| TASK_SERVO_OFF       | 36    | Disable all joints                   |
| TASK_SERVO_ON        | 35    | Enable all joints                    |
| TASK_CLEAR_FAULT     | 34    | Clear joint errors                   |
| TASK_SET_HOME        | 3000  | Set zero positions                   |
| TASK_TEST_JOINT      | 3003  | Joint motion test                    |
| TASK_READY_STATE     | 3011  | Enter ready posture                  |
| TASK_RL_WALK        | 3530  | Enable walking mode                  |

#### GRX/Client Keys

Virtual control interfaces:

| Key                                       | Description                  | Type               | Details                |
|-------------------------------------------|------------------------------|--------------------|------------------------|
| `virtual_joystick_button_up`              | Virtual UP button            | int                | 0: Released, 1: Pressed |
| `virtual_joystick_axis_left`              | Left joystick position       | array(float, float)| Range: [-1, 1]         |
| ... (Other virtual controls follow similar structure) |                              |                    |                        |

---

## API Protocols (Developer)

### State Dictionary

| Key                    | Description                  | Type                          |
|------------------------|------------------------------|-------------------------------|
| `imu_quat`             | IMU quaternion               | array(float[4])               |
| `imu_euler_angle`      | IMU Euler angles             | array(float[3])               |
| `joint_position`       | Joint positions (deg)        | array(float[N])               |
| ... (Other state keys follow) |                              |                               |

### Control Dictionary

| Key                   | Description                  | Type                          | Details                     |
|-----------------------|------------------------------|-------------------------------|-----------------------------|
| `control_mode`        | Control mode                 | int                           | 0: None, 1: Current, etc.   |
| `position`            | Position target (deg)        | array(float[N])               |                             |
| `pd_control_kp`       | PD control P gain            | array(float[N])               |                             |
| ... (Other control keys follow) |                              |                               |                             |

---

## FAQ

- **Self-check fails with "Unable to access actuator"**:  
  Verify actuator power/network connections. Check Ethernet static IP configuration.  
- **Internet access**: Use wired connection in DHCP mode, then revert to static IP (`192.168.137.200`).  
- **Disable auto-hotspot**:  
  ```bash
  sudo systemctl stop rocs-wifi  # Temporary
  sudo systemctl disable rocs-wifi  # Permanent
  ```
- **Control frequency**:  
  - User APIs: 50Hz  
  - Developer APIs: Up to 500Hz  
- **Timeout warnings**: Check IPv6 settings and cable connections.  
- **GLIBC_2.33 missing**: Upgrade to Ubuntu 22.04 or use compatible build tools.  

---

## Changelog

- **2025-03-04**:  
  - Zenoh upgraded to 1.0.1  
  - `TASK_RL_WALK` command updated to 3530 (requires `fourier-grx` ≥ 2.2.5)  
- **2025-03-24**:  
  - Compatibility updates for `fourier-grx` ≥ 2.3.0  

---

## Acknowledgments

- Zenoh team for the distributed data framework: https://zenoh.io/