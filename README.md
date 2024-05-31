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

3. Install necessary environment:
    - Use `requirements.txt`:
        - `pip install -r ./requirements.txt`

Till now, if no error presents, the environment for develop Fourier GRx robot is ready.
You can now run the demos in the repository.

## Demos

For more demo details, please refer to the `DEMOS.md` file.

[DEMOS.md](DEMOS.md)

## Change Logs

- 2024-05-31:
    - **Fix**: fix the bug of running calibration will cause robot moving crazily. Now the robot will not be servo on when running calibration, which will be more safe.
    - **Update**: Now when run into developer mode, the robot will not servo on. In the last version, the robot will servo on when call the `ControlSystem().dev_mode()`.
    - **Update**: state estimator will not be enabled in default, because we found that the state estimator is not accurate enough as we expected. So we disable it in default.
        - Current state estimator is based on the joint position, velocity and torque, and the IMU data. But the IMU data can have high noise during walking, which will cause the state estimator not
          accurate enough.
    - **Update**: the library files `robot_rcs` and `robot_rcs_gr` to the latest version.

## Known Issues

For known issues, please refer to the `KNOWN_ISSUES.md` file.

[KNOWN_ISSUES.md](KNOWN_ISSUES.md)

---

Thank you for your interest in the Fourier Intelligence GRx Robot Repositories.
We hope you find this resource helpful in your robotics projects!