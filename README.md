# Robot-RCS-GR 代码仓库说明

本仓库为 Robot-RCS-GR 代码仓库，包含了 Robot-RCS 针对通用机器人开发的所有代码。

## 文件结构

- robot-rcs: Robot-RCS 的基础核心代码
- robot-rcs-fld: Robot-RCS 的通用机器人 FDB 系列代码
- robot-rcs-fld: Robot-RCS 的通用机器人 FLD 系列代码
- robot-rcs-flw: Robot-RCS 的通用机器人 FLW 系列代码
- robot-rcs-gr: Robot-RCS 的通用机器人 GR 系列代码

## 开发环境搭建

需要注意的是，为了提高运行效率，我们将 python 运行环境从 3.8 升级到了 3.12，且升级后的代码不再兼容 3.8 版本。
因此，需要在安装开发环境时，确保 python 为 3.8 以上版本。

在每一个文件夹中，都针对不同的机器人提供了相应的开发环境搭建脚本 setup.py。通过运行 setup.py，可以安装相应的开发环境。

```
pip install -e . -i https://pypi.tuna.tsinghua.edu.cn/simple
```

其中，部分代码库是必装，部分代码库是可选装。

- robot-rcs: 必装
- robot-rcs-fld: 选装
- robot-rcs-fld: 选装
- robot-rcs-flw: 选装
- robot-rcs-gr: 选装

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

## Notice

Remember to disable efficient core. 记得关闭能效核心。

## 程序运行指定特定 CPU

在Python中，您可以通过几种方式控制程序运行在哪个CPU上，但这通常涉及到操作系统级别的调用或库。

1. 使用os模块

    - 在Unix-like的系统中（例如Linux或macOS），您可以使用os模块来设置进程的亲和性，即指定程序应该运行在哪个CPU上。
   ```
   import os
   
   def set_affinity(pid, cpu):
       os.system("taskset -cp {} {}".format(cpu, pid))
   
   pid = os.getpid()  # 获取当前进程的ID
   cpu = "0"  # 假设我们想把程序绑定到CPU 0上
   set_affinity(pid, cpu)
   ```

    - `taskset` 是一个命令行工具，用于设置或检索Linux进程的CPU亲和性。

2. 使用第三方库

   有些第三方库，如 `psutil`，也提供了设置进程亲和性的功能。使用这些库可能会更方便，同时可以跨平台工作。
    - 安装psutil（如果还没有安装）:
   ```
   pip install psutil
   ```
    - 然后在Python代码中设置亲和性：
   ```
   import psutil
   
   p = psutil.Process()  # 获取当前进程
   p.cpu_affinity([0])  # 设置进程的CPU亲和性，只运行在CPU 0上
   ```

3. 使用 multiprocessing 模块

   如果您的程序是多线程或多处理的，并且您想要控制特定的线程或子进程运行在特定的CPU上，您可能需要看看multiprocessing模块。

   multiprocessing模块没有提供直接设置CPU亲和性的API，但您可以在创建的进程中使用上文的os或psutil模块的方法来设置。
   ```
   import multiprocessing
   import os
   
   def worker():
       pid = os.getpid()
       cpu = "0"
       os.system("taskset -cp {} {}".format(cpu, pid))
       # 在这里执行你的任务

   # 创建子进程
   p = multiprocessing.Process(target=worker)
   p.start()
   p.join()
   ```

   这里同样是在进程函数内部使用taskset来设置CPU亲和性。

   > 注意:
   > 直接操作系统级别的CPU亲和性需要操作系统的支持，并且可能需要额外的权限。在Windows上，您可能需要使用不同的方法，如调用Windows API（例如 SetProcessAffinityMask）。
   > 还需要注意的是，除非有非常特定的理由，否则手动设置CPU亲和性很少是必要的，现代操作系统通常非常擅长在可用的CPU之间分配进程，而不会有人为干预。如果您没有遇到因为并发或性能问题而必须将进程绑定到特定CPU的情况，最好让操作系统管理这些复杂的调度问题。

## 隔离 CPU

在Ubuntu（或其他基于Linux的系统）上，如果您想要让一个CPU核心完全空闲，您可以使用CPU隔离（isolcpus）的方法。这可以通过在Grub配置中设置内核引导参数来实现。
隔离CPU后，内核调度器将不会在这些CPU上运行任何非绑定进程。只有特定地被指定运行在这些CPU上的进程才会在这些核心上执行。

下面的步骤展示了一般的流程：

1. 编辑Grub配置文件：
    - 打开终端，运行以下命令来编辑Grub配置文件:
   ```
   sudo nano /etc/default/grub
   ```

2. 修改GRUB_CMDLINE_LINUX：
    - 在打开的编辑器中，找到GRUB_CMDLINE_LINUX这一行，添加isolcpus参数。例如，如果您想隔离CPU 1（核心编号通常从0开始），您应该添加isolcpus=1。请注意，如果该行已经包含其他参数，您应该在参数间添加空格。
   ```
   GRUB_CMDLINE_LINUX="...其他参数... isolcpus=1"
   ```

3. 更新Grub并重启系统：
    - 保存文件并关闭编辑器。然后运行以下命令来更新Grub配置，并重启以应用更改：
   ```
   sudo update-grub
   sudo reboot
   ```

4. 重启系统后，核心将被隔离，不会执行常规任务。
    - 手动绑定特定任务：
      如果您现在想要在隔离的CPU上运行特定任务，您可以使用前面提到的方法，如taskset或psutil模块来将进程绑定到这个CPU上。

> 请注意，isolcpus参数的一个副作用是在隔离的CPU上可能无法均衡负载。不建议长期隔离核心，除非您非常确定这样做的原因。
> 此外，这种更改是全局性的，并会影响到操作系统的所有任务调度。确保您了解这样做的影响，特别是对于多线程应用和系统稳定性。
> 最后，由于存在多种Linux内核版本和配置，具体步骤可能会根据系统���异。在执行此类操作之前，建议阅读更多相关文档，并在理解其原理和后果的情况下进行操作。