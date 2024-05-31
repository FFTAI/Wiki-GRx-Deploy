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

---

## Calibration

The first step to run the demo code, is to make sure the `sensor_offset.json` is filled with your machine absolute encoder value,
instead of the value in this repository (may be different from your machine).

Every machine has its own home position encoder value,
which should be set with the machine power on and the joint fixed at the pin position.

After you have finished the machine physical calibration process (move all joints to the pin position),
you can run the demo code in this repository, this demo code will record its absolute encoder value and stored in the `sensor_offset.json` file.

```
python demo_set_home.py ./config/config_xxx.yaml
```

## Info level

The print level of the information are separated into 5 levels:

- Info Level: white color word, for general information.
- Highlight Level: green color word, for important information.
- Debug Level: red color word, for debug information.
- Warning Level: blue color word, yellow background, for warning information.
- Error Level: white color word, red background, for error information. (**When happen, should stop the program and check the error.**
  )

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

For more demo details, please refer to the `DEMOS.md` file.

[DEMOS.md](DEMOS.md)

---

## Change Logs

- 2024-05-31:
    - **Change** when run into developer mode, the robot will not servo on. In the last version, the robot will servo on when call the `ControlSystem().dev_mode()`.
    - **Fix** the bug of running calibration will cause robot moving crazily. Now the robot will not be servo on when running calibration, which will be more safe. And the moving crazily bug is fixed.
    - **Change** state estimator not be enabled in default, because we found that the state estimator is not accurate enough as we expected. So we disable it in default.
        - Current state estimator is based on the joint position, velocity and torque, and the IMU data. But the IMU data can have high noise during walking, which will cause the state estimator not
          accurate enough.
    - **Update** the lib `robot_rcs` and `robot_rcs_gr` to the latest version.

---

Thank you for your interest in the Fourier Intelligence GRx Robot Repositories.
We hope you find this resource helpful in your robotics projects!