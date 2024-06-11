# Wiki-GRx-Deploy Known Issues

## IMU Serial Port Issue

1. `IMU Error Frame` Issue:
    - When running the demo scripts, users may encounter an issue where the IMU serial port is broken or overloaded.
      When this issue occurs, the IMU sensor may not be able to communicate with the system, leading to errors in the IMU data:
      `IMU Error Frame`.
    - This issue can be resolved by restarting the system or reconnecting the IMU sensor.
    - As the GRx robot controller is covered inside, reboot the machine may be a better solution.

## Joystick Issue

1. `pygame error: Invalid joystick device number` Issue:
    - When running the demo scripts, users may encounter an issue where the joystick device number is invalid.
    - Causes:
        - This issue may occur when the joystick is not properly connected or recognized by the system.
        - This issue is know to also occur when the user try to use **ssh** to connect to the robot and run the demo scripts.
          This is because the `pygame` library cannot detect the joystick device number correctly in this scenario.
    - Solution:
        - Check the joystick connection and ensure that it is properly connected to the system.
        - Restart the system and reconnect the joystick to see if the issue persists.
        - For users who are using **ssh** to connect to the robot, it is recommended to run the demo scripts directly on the robot computer.
