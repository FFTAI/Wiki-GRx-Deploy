# Wiki-GRx-Deploy Tools

### Load Robot Absolute Encoder Calibration Value

In case you don't want to calibrate the robot's absolute encoder by yourself using the pin tool, you can load the calibration value directly from your machine.

We provide a script to load the calibration value in the `tools` folder named `load_calibration.py`.

You can use this script to load the calibration value directly.

```
python load_calibration.py
```

> **Notice**:
> - Make sure you have set the right path to the calibration file in the script.

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

> **Notice**:
> - We may upgrade our actuators on GR1T1 and GR1T2 to FSA v2 firmware soon in the future, which supports PD control directly.
    > All our actuators on GR1T1 and GR1T2 support OTA upgrade.
