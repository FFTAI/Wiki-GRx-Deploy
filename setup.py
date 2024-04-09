from setuptools import setup, find_packages

setup(
    name='fi_rcs',
    version='1.0.1',
    description='Fourier Intelligence(FI) Robot Control System (RCS) Interface',
    author='Fourier Intelligence Ltd.',
    author_email='xin.chen@fftai.com',

    packages=find_packages(),
    install_requires=[
        'numpy',
        # for IMU
        # https://pyserial.readthedocs.io/en/latest/pyserial_api.html
        'pyserial==3.5',  # python3.8 无法使用 pyserial==2.7
        # for joystick
        'pygame',
        'nuitka',
    ],
    python_requires='==3.11.*',  # 这就是设置 Python 版本的地方
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3.11',
    ],
)
