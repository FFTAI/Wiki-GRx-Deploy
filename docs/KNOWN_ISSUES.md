# Wiki-GRx-Deploy Known Issues

## GLIBC Issues

1. `GLIBC` Version Issue:
    - When running the demo scripts, users may encounter an issue where the `GLIBC` version is not compatible with the system.
        - `libc.so.6: version 'GLIBC_2.33' not found`
    - Causes:
        - This issue may occur when the `GLIBC` version required by the package is not compatible with the system.
        - This issue can also occur when the `GLIBC` version is not properly installed or is missing from the system.
    - Solution:
        - Check the `GLIBC` version required by the package and ensure that it is compatible with the system.
        - Reinstall the `GLIBC` version required by the package to see if the issue persists.
    - GLIBC upgrade:
        - `ldd --version`: Check the current GLIBC version.
        - `sudo apt-get update`: Update the system.
        - `sudo apt-get upgrade libc6`: Upgrade the GLIBC version.
        - `sudo reboot`: Reboot the system.
        - `ldd --version`: Check the new GLIBC version again.

> **Notice**:
> - Upgrade the `GLIBC` version with caution, as it may affect the system stability.
> - If you still meet the issue after upgrading the `GLIBC` version, please contact us for further assistance.

## RBDL Issues

Please refer to the [RBDL Installation Guide](./rbdl_installation.md) for more details.

## IMU Serial Port Issues

1. `IMU Error Frame` Issue:
    - When running the demo scripts, users may encounter an issue where the IMU serial port is broken or overloaded.
      When this issue occurs, the IMU sensor may not be able to communicate with the system, leading to errors in the IMU data:
      `IMU Error Frame`.
    - This issue can be resolved by restarting the system or reconnecting the IMU sensor.
    - As the GRx robot controller is covered inside, reboot the machine may be a better solution.

## Joystick Issues

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

## Package Metadata Issues

1. `Package Metadata` not found Issue:
    - When running the demo scripts, users may encounter an issue where the package metadata is not found.
    - Causes:
        - This issue may occur when the package metadata is not properly installed or is missing from the system.
        - This issue can also occur when the package files provided is not compiled on the similar system.
    - Solution:
        - Reinstall the package metadata by running the installation script provided in the repository.
        - **Contact us for re-uploading the package metadata files if the issue persists.**
