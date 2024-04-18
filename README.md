# Robot-RCS-GR 代码仓库说明

本仓库为 Robot-RCS-GR 代码仓库，包含了 Robot-RCS 针对通用机器人开发的所有代码。

## 文件结构说明

- robot_rcs.cpython-311-x86_64-linux-gnu.so Robot-RCS 的基础核心代码库
- parallel_ankle.cpython-311-x86_64-linux-gnu.so 针对 GR1T1 的并联脚踝代码库
- robot_rcs_base: Robot-RCS 使用的示例代码
    - 基础部分：
        - config: 配置文件存放文件夹
        - webots: webots仿真环境文件夹
            - controllers/webots_gr1t1/webots_gr1t1.py: webots GR1T1 控制器入口
        - fi_robot_interface.py 傅利叶智能机器人接口类，用于选定初始化的机器人类型
        - fi_robot_fftai.py 傅利叶智能机器人基础类
        - fi_robot_gr1t1.py 傅利叶智能机器人 GR1T1 类，继承自 fi_robot_fftai.py 【提供基本站立复位功能示例代码】
        - fi_robot_gr1t1_algorithm.py 傅利叶智能机器人 GR1T1 算法类，进行基本的站立控制算法计算
    - 定制部分：
        - fi_robot_gr1t1_customize.py 傅利叶智能机器人 GR1T1 定制类，继承自 fi_robot_gr1t1.py 【供用户修改】
        - fi_robot_gr1t1_customize_algorithm.py 傅利叶智能机器人 GR1T1 定制算法类 【供用户修改】
        - fi_webots_gr1t1.py 傅利叶智能机器人 GR1T1 webots仿真类，用于仿真环境的初始化和控制 【供用户修改】
- main_mp.py: 主程序入口
    - 实机调用方法：`python main_mp.py ./robot_rcs_base/config/config_GR1T1.json`

## 开发环境搭建

需要注意的是，为了提高运行效率，我们将 python 运行环境从 3.8 升级到了 3.11，且升级后的代码不再兼容 3.8 版本。
因此，需要在安装开发环境时，确保 python 为 3.8 以上版本。

在每一个文件夹中，都针对不同的机器人提供了相应的开发环境搭建脚本 setup.py。通过运行 setup.py，可以安装相应的开发环境。

```
pip install -e . -i https://pypi.tuna.tsinghua.edu.cn/simple
```

其中，部分代码库是必装，部分代码库是可选装。

- robot_rcs_base: 必装

## 上机流程

1. 安装依赖：
    - 安装依赖库
    ```
    pip install -e . -i https://pypi.tuna.tsinghua.edu.cn/simple
    ```

2. 运行程序：
    - 运行主程序
    ```
    python main_mp.py ./robot_rcs_base/config/config_GR1T1.json
    ```

3. 位置校准：
    - 对于下肢和腰部的关节，使用插销将机器人的下肢、腰部固定在初始角度。
    - 对于上肢和头部的关节，根据关节卡槽的位置，将机器人的上肢、头部手动转动到初始角度。
    - 运行程序，此时程序启动，但是机器人关节执行器不会使能运动。
    - 通过手柄 L1 按键，切换到 SET_HOME 模式。并按下手柄 L2 按钮，执行编码器位置校准指令，程序会将编码器的数值存储到 sensor_offset.txt 文件中。
    - 拔掉插销，使机器人的关节可以自由运动。
    - 重新运行程序，此时程序启动，但是机器人关节执行器不会使能运动。
    - 通过手柄 L1 按键，切换到 STNAD 模式，机器人会根据 sensor_offset.txt 文件中的编码器数值，将机器人的关节位置校准到初始位置。
    - 查看机器人的关节位置，确保机器人的关节位置校准正确。

4. 运行定制算法程序：
    - 确保根据 fi_robot_gr1t1_customize.py 和 fi_robot_gr1t1_customize_algorithm.py 文件，修改了机器人的定制算法程序。
    - 运行程序，此时程序启动，但是机器人关节执行器不会使能运动。
    - 通过手柄 L1 按键，切换到 STNAD 模式，机器人会根据 sensor_offset.txt 文件中的编码器数值，将机器人的关节位置校准到初始位置。
    - 查看机器人的关节位置，确保机器人的关节位置校准正确。
    - 通过手柄 L1 按键，切换到 CUSTOMIZE 模式，机器人根据用户定制的算法程序，执行相应的动作。

## Nuitka 打包程序

1. 安装 python 3.11 版本：
    - 因为 nuitka 最高只支持 python 3.11 版本，所以需要安装 python 3.11 版本。

2. 安装 libpython-static 依赖：
    - Conda 环境下打包需要安装 libpython-static 依赖。
    ```
    conda install libpython-static
    ```

3. 安装 patchelf 依赖：
    - Ubuntu 环境下打包需要安装 patchelf 依赖。
    ```
    conda install patchelf
    ```
    - Ubuntu 下安装 ccahe 依赖：
    ```
    sudo apt-get install ccache
    ```

4. 安装 nuitka：
    - 安装 nuitka 依赖
    ```
    pip install nuitka
    ```

5. 运行 nuitka 打包程序：
    - 运行 nuitka 打包程序，将代码打包成可执行文件。
    ```
    nuitka3 --standalone --onefile --plugin-enable=numpy,torch main_mp.py
    ```
    - Nuitka 目前已经支持 numpy 和 torch 的打包，可以直接使用 nuitka 打包程序。
   ```
   Nuitka-Plugins:WARNING: numpy: This plugin has been deprecated, do not enable it anymore.
   Nuitka-Plugins:WARNING: torch: This plugin has been deprecated, do not enable it anymore.

   nuitka3 --standalone --onefile main_mp.py
   ```
