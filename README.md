# Wiki-GRx
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