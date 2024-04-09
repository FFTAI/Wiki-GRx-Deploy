# fi_rcs - Fourier Intelligence Robot Control System Interface

`fi_rcs` is the official Python interface for the Fourier Intelligence Robot Control System (RCS), providing essential tools and functionalities necessary for controlling and interfacing with FI robots.

## Features

- Intuitive and easy-to-use API for robot control.
- Support for IMU data acquisition through serial communication.
- Joystick interface capabilities via pygame library.

## Requirements

- Python 3.11.*
- NumPy
- PySerial v3.5
- Pygame

`fi_rcs` is tested and confirmed to work with Python version 3.11.*.

## Installation

You can install `fi_rcs` using pip:

```bash
pip install fi_rcs
```

Make sure you are running Python 3.11 as it is the required version for this package.

## Usage

After installing, you can import `fi_rcs` in your Python scripts as follows:

```python
import fi_rcs
```

Further documentation detailing the use of the API and interaction with FI robots will be provided separately.

## Dependencies

- The `pyserial` package is utilized for serial communication, which allows for the interaction with IMU sensors.
- The `pygame` package is used to capture joystick inputs for manual robot control.

These dependencies are automatically installed with `fi_rcs`.

## Support

For any technical issues or questions, please contact Fourier Intelligence Ltd.:

- **Author**: Fourier Intelligence Ltd.
- **Email**: xin.chen@fftai.com

## License

`fi_rcs` is provided under the MIT License. See LICENSE file for the full text.

## Contributing

Development of `fi_rcs` happens on our Gitee page. Contributions to the project are welcome.

---

We are continuously working to improve the `fi_rcs` library and we appreciate any feedback or contributions to the project.

Remember to replace dummy texts with actual information if necessary. For example, if there's a GitHub page for the project or other contact details, you should include them. Additionally, if there is more extensive documentation or a set of examples on how to use `fi_rcs`, those could be linked or included as well.

You should also create the actual `LICENSE` file with the MIT License text if it's not already present in the project's repository.
