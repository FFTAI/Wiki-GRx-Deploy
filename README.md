# Wiki-GRx

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

## GR1

run demo with following command:

```
python demo_xxx.py config_GR1_T1.yaml
```

## GR2

run demo with following command:

```
python demo_xxx.py config_GR1_T2.yaml
```