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

## 参考文档

Please refer to the document [Fourier-GRX](https://fourier-grx.github.io) for more details.

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