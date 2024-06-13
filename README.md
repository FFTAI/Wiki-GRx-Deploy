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
        - `conda create -n wiki-grx-deploy python=3.11`
        - `conda activate wiki-grx-deploy`

3. Install [RBDL](https://github.com/rbdl/rbdl):
    The RBDL dependencies requires some extra setup, please refer to the [[docs/rbdl_installation.md]] guide for more details.

3. Install necessary environment:
    - Change directory to the workspace directory.
    - Use `requirements.txt` to install required libraries:
        - `pip install -r ./requirements.txt`

Till now, if no error presents, the environment for develop Fourier GRx robot is ready.
You can now run the demos in the repository.


## Demos

For more demo details, please refer to the `DEMOS.md` file.

[DEMOS.md](docs/DEMOS.md)

## Tools

For more tools details, please refer to the `TOOLS.md` file.

[TOOLS.md](docs/TOOLS.md)

## Known Issues

For known issues, please refer to the `KNOWN_ISSUES.md` file.

[KNOWN_ISSUES.md](docs/KNOWN_ISSUES.md)

## Change Logs

- 2024-06-11:
    - **Fix**: fix the bug calculating the IMU quaternion. In last version, we use the euler angles to calculate the quaternion, which will cause the quaternion not continuous.
      Now we use the IMU original quaternion to calculate the new quaternion, which will be continuous.
    - **Update**: adapt the upside down installation of the IMU. Now the IMU data at GR1T1 and GR1T2 will be correct, without the necessity of changing the IMU installation. 😊
    - **Update**: update the library files `robot_rcs` and `robot_rcs_gr` to the latest version.

- 2024-06-05:
    - **Fix**: fix the bug calculating the parallel wrist using the wrong joint angles. In last version, we use the wrong joint indexes to get the joint angles, which will cause the parallel wrist
      calculation wrong.
    - **Update**: update the library files `robot_rcs` and `robot_rcs_gr` to the latest version.
    - **Update**: allow use `ControlSystem().developer_mode(servo_on=True)` to **servo on/off** the robot at the same time when call the developer mode. In default, the robot will servo off when call
      the developer mode.

- 2024-05-31:
    - **Fix**: fix the bug of running calibration will cause robot moving crazily. Now the robot will not be servo on when running calibration, which will be more safe.
    - **Update**: Now when run into developer mode, the robot will not servo on. In the last version, the robot will servo on when call the `ControlSystem().dev_mode()`.
    - **Update**: state estimator will not be enabled in default, because we found that the state estimator is not accurate enough as we expected. So we disable it in default.
        - Current state estimator is based on the joint position, velocity and torque, and the IMU data. But the IMU data can have high noise during walking, which will cause the state estimator not
          accurate enough.
        - What is a suggest way is using neural network to estimate the state, which will be more accurate and robust. We may implement to our library in the future.
    - **Update**: update the library files `robot_rcs` and `robot_rcs_gr` to the latest version.

---

Thank you for your interest in the Fourier Intelligence GRx Robot Repositories.
We hope you find this resource helpful in your robotics projects!