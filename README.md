# User Guide
[Fourier Intelligence Inc.](https://www.fftai.cn/)

#### 介绍
此代码为StateEstimator的测试例程

**Code**
 - **CMakeLists.txt**		
 - **inputdata.txt**		(状态估计输入值)
 - **main.cpp**			
 - **model.json**			(GR1T1模型参数)
 - **StateEstimator**	(状态估计器)
 - **ThirdParty**			(所用到的第三方库)


#### 说明

1. inputdata.txt为采集的左右腿 **十二个电机** 的 **位置**、**速度**和**力矩**，在main中解析并以400Hz的频率（机器人控制频率）发送给状态估计器。前5秒为机器人启动状态，获取的状态会有误差。
2. 状态估计器 StateEstimator
	(1) 输入： 十二个电机的位置、速度和力矩、IMU数据(欧拉角、角速度和加速度)
	(2) 输出： 12 * 6 的矩阵用于存放估计状态值，包含各位置的角度、角速度、角加速度、位移、速度、加速度、力矩和力

---
	
estState.block(0, 0, 4, 6) - **浮动基状态值** 
	
| **φ(x)** | **φ(y)** | **φ(z)** | **p(x)** | **p(y)** | **p(z)** |
| -------- | -------- | -------- | -------- | -------- | -------- |
| **ω(x)** | **ω(y)** | **ω(z)** | **v(x)** | **v(y)** | **v(z)** |
| **α(x)** | **α(y)** | **α(z)** | **a(x)** | **a(y)** | **a(z)** |
| **null** | **null** | **null** |  **null** | **null** | **null** |

其中，角度、角速度、角加速度和浮动基的线速度都是基于世界坐标系的，位置量指当前时刻浮动基到支撑脚的位置差


estState.block(4, 0, 4, 6): **左脚状态值**
	
| **φ(x)** | **φ(y)** | **φ(z)** | **p(x)** | **p(y)** | **p(z)** |
| -------- | -------- | -------- | -------- | -------- | -------- |
| **ω(x)** | **ω(y)** | **ω(z)** | **v(x)** | **v(y)** | **v(z)** |
| **α(x)** | **α(y)** | **α(z)** | **a(x)** | **a(y)** | **a(z)** |
| **τ(x)** | **τ(y)** | **τ(z)** | **F(x)** | **F(y)** | **F(z)** |

estState.block(8, 0, 4, 6): **右脚状态值**
	
| **φ(x)** | **φ(y)** | **φ(z)** | **p(x)** | **p(y)** | **p(z)** |
| -------- | -------- | -------- | -------- | -------- | -------- |
| **ω(x)** | **ω(y)** | **ω(z)** | **v(x)** | **v(y)** | **v(z)** |
| **α(x)** | **α(y)** | **α(z)** | **a(x)** | **a(y)** | **a(z)** |
| **τ(x)** | **τ(y)** | **τ(z)** | **F(x)** | **F(y)** | **F(z)** |

注意，左右脚的状态是基于base计算的（base为运动学计算基点），对于世界坐标系还需要额外转换
具体细节请参考[RBDL官网](https://rbdl.github.io/index.html)
