[English](README.en.md) | 简体中文

# Fourier-GRX GRMini 开发接口说明

## 机器人系统

Fourier-GRX GRMini 机器人的控制系统运行在一台主控电脑上，主控电脑系统为 Ubuntu 22.04 LTS。

### 登录系统

#### 本地登录

连接机器人控制电脑的 HDMI 显示器和 USB 键盘鼠标，开机后会自动进入系统桌面。

用户名为 `gr2m25jaxxxx`, `xxxx` 为机器人的后四位序列号，密码为 `fftai2015`。

#### 远程登录

机器人开机后，系统会自动启动机器人主控系统的热点信号，用户可以通过手机或电脑连接机器人的热点信号。

- 热点名称为 `gr2m25jaxxxx`, `xxxx` 为机器人的后四位序列号。
- 热点密码为 `66668888`。

连接完成后，可以通过 `ssh` 服务登录到机器人的主控电脑，登录用户名为 `机器人热点信号名称`，密码为 `fftai2015`。

> **说明**：
> 部分机器人没有配置自动热点模式，因此搜不到热点信号，此时可以考虑用有线网络方式连接机器人。
> 机器人的有线网口 IP 地址为 `192.168.137.220`, ssh 登录信息与无线方式相同。

### 程序启动

机器人提供了 `fourier-grx start` 命令用于机器人控制程序启动。

```bash
# 在机器人主控电脑上：
# 1. 准备好手柄，连接到机器人主控电脑的 USB 端口。
# 2. 启动 fourier-grx 主程序
fourier-grx start
```

程序启动完成后，即可使用手柄控制机器人完成相应的任务。（图片为 XBOX 键位手柄，具体按键功能对应关系与所用手柄种类相关）

![joystick.jpg](picture/joystick.jpg)

### 二次开发

#### 二次开发环境支持

| 系统环境             | Python 环境   | 已测试 | 测试通过 |
|------------------|-------------|-----|------|
| Ubuntu 22.04 LTS | Python 3.11 | ✅   | ✅    |
| Windows          | Python 3.11 |     |      |
| MacOS            | Python 3.11 |     |      |

#### 开发环境搭建

机器人提供了 `fourier-grx setup_conda` 命令用于一键配置 conda 开发环境。

```bash
# 在机器人主控电脑上：
fourier-grx setup_conda

# 程序运行完成后，会搭建出一个名为 `fourier-grx` 的 conda 环境，可以通过以下命令激活该环境
conda activate fourier-grx

# 如果希望自主搭建开发环境，可以在 $HOME/fourier-grx/whl 中找到依赖库文件进行手动安装。
```

#### 同步示例程序

可以通过 git 同步机器人的二次开发接口示例程序，同步命令为：

```bash
git clone https://gitee.com/FourierIntelligence/wiki-grx-mini
```

建议同步到 `$HOME` 目录下，同步完成后，可以通过 `cd $HOME/wiki-grx-mini` 进入该目录查看。

---

## Fourier-GRX 开发接口

开发接口分为两类：

- 针对高层应用的接口 **user**
- 针对底层开发的接口 **developer**

两者的区别在于：

- **user** 接口是在启动了 fourier-grx 主程序后，通过 zenoh (https://zenoh.io/) 接口进行通信，发送指令数据并返回状态信息，主要用于用户对机器人进行高层控制。
- **developer** 接口是直接调用 fourier-grx 底层二次开发接口，可以直接获取到底层的状态信息，用于开发者对机器人进行底层开发。

### user

user 目录下的接口是为了方便用户使用 Fourier-GRX 而设计的接口，
这些接口更多是对 Fourier-GRX 系列机器人内部已有算法的调用。

> **说明**：
> user 接口的开发使用了 zenoh 进行通信，因此可以在任意一台与机器人同一局域网的电脑上进行开发。

目前提供的开发示例有：

- `demo_servo_on`: 机器人全关节上电使能。
- `demo_servo_off`: 机器人全关节下电失能。
- `demo_clear_fault`: 清除机器人全关节报警。当机器人出现报警时，可以通过此接口清除报警。
- `demo_set_home`: 设置机器人全关节零位位置为当前位置，用于标定机器人关节零位。
- `demo_test_joint`: 机器人关节运动功能测试，用于检测机器人关节是否能够正常运动。
- `demo_ready_state`: 机器人运行到 **准备状态**，为微曲膝关节的站立姿态。
- `demo_rl_walk`: 机器人运动到 **行走状态**，可以用手柄控制机器人行走。

#### 示例程序运行方法

```bash
# 在机器人主控电脑上
# 1. 启动 fourier-grx 主程序
conda activate fourier-grx  # 激活 conda 环境
python $HOME/fourier-grx/whl/run.py --config=$HOME/fourier-grx/config/grmini1/config_GRMini1_{具体机型}_sdk.yaml  # 启动 fourier-grx 主程序

# 在机器人主控电脑上或与机器人同局域网内的任意一台电脑上
# 2. 启动 user 接口示例
conda activate fourier-grx  # 激活 conda 环境
python $HOME/wiki-grx-mini/user/demo_{具体示例}.py  # 启动示例
```

如果是在远程电脑上控制机器人，建议使用 Terminal 开多窗口，方便查看机器人状态信息，确保任务有正确被执行。
如下图所示：

![img.png](picture/img.png)

### developer

developer 目录下的接口是为了方便开发者对 Fourier-GRX 系列机器人进行底层开发而设计的接口，
这些接口更多是对 Fourier-GRX 系列机器人内部底层硬件的调用。

> **说明**：
> 目前为了保证接口的数据有效性和数据传输的实时性，developer 接口的开发需要在机器人的主控电脑上完成。
> 因此，建议开发者在对机器人足够熟悉后再使用该接口进行开发。

目前提供的开发示例有：

- `demo_print_state`: 打印机器人状态信息。
- `demo_servo_on`: 机器人全关节上电使能。
- `demo_servo_off`: 机器人全关节下电失能。
- `demo_set_home`: 设置机器人全关节零位位置为当前位置，用于标定机器人关节零位。
- `demo_set_pid`: 设置机器人关节 PID 参数。
- `demo_ready_state`: 机器人运行到 **准备状态**，为微曲膝关节的站立姿态。
- `demo_rl_walk`: 机器人运动到 **行走状态**，可以用手柄控制机器人行走。

#### 示例程序运行方法

```bash
# 在机器人主控电脑上
# 1. 启动 developer 接口示例
conda activate fourier-grx  # 激活 conda 环境
python $HOME/wiki-grx-mini/developer/demo_{具体示例}.py --config=$HOME/fourier-grx/config/grmini1/config_GRMini1_{具体机型}_sdk.yaml  # 启动示例
```

---

## 参考文档

请参阅文档 [Fourier-GRX](https://fourier-grx.github.io) 以获取更多详细信息。

---

## 更新日志

- 2025-03-04:
    - 升级 zenoh 版本到 1.0.1
    - 使用的 rl_walk 指令改为 3530 (请确保机器人主程序已更新到版本 >= 2.2.5)
- 2025-03-24:
    - 升级 fourier-grx >= 2.3.0 程序，旧程序存在兼容性问题，适配新版本程序接口

---

## 感谢

- Zenoh 团队提供的 zenoh 分布式系统的数据共享和协作平台。https://zenoh.io/