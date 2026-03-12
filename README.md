

# **<u>FOC 学习文档</u>** 

本文档写于<mark>2025.1.24</mark>，便于笔者巩固有关FOC的技能。包含关于无刷电机的硬件控制原理、SVPWM技术与不同<u>PID</u>（速度环、位置环、电流环）的代码编写过程，基于**SimpleFOC**库进行代码编写，在代码的编写过程中，包含自己对于FOC控制的理解，从灯哥开源的Deng FOC库开始一步步搭建可移植性强的FOC计算库，并使用**2208无刷电机搭配AS5600编码器**进行实验，便于后续的项目电机开发。

Github : https://github.com/kisawvan?tab=packages

email : 17673687312@163.com

## 一 、FOC硬件控制原理
### 1.1 无刷电机（BLDC）简介

无刷电机（BLDC）因其**高效率、低维护、长寿命**多应用与自动化控制领域。顾名思义，无刷电机和有刷电机最基本的差别在于**是否包含电机内电刷即无需使用电刷进行换向**，从而消除了电刷的磨损问题。电机的内部结构可以简单地看做由定子和转子组成，定子通常为电机内部绕中心轴均匀分布的绕组线圈，转子则为嵌有多级永磁铁的旋转部分，能够绕轴进行旋转。由高中的知识我们可以知道，电机的工作原理是通过控制电流的方向，由电生磁的原理使定子产生不同方向的磁场，从而使转子在磁场中发生定向转动。通过外部的 MCU 微控制器和 MOS 管，合理的控制这些线圈的通断顺序，就能够驱动**转子**进行旋转。

![img](https://pic1.zhimg.com/v2-25efe2b9148c66db9a4f73a6368b174a_1440w.jpg)

我们首先明确两个重要的基本物理量：

- 机械角度。这个很好理解。即电机定子在整个空间内0°~360°的角度范围，通常为位置控制时我们的期望控制量。
- 定子极对数（**PP**）。一般来说，定子嵌有多级磁极。一对N-S极即为一对磁极。
- 电角度。这个理解起来比抽象。在数学关系上，**电机电角度=机械角度*极对数**；在物理关系上，以电磁场周期变化为基础定义的角度，定义一对磁极（N-S）对应电角度0°~360°，**电角度反映了绕组中电流或感应电动势变化的周期性**。通常来说，我们转动未通电的电机，产生电磁感应现象，通过示波器采集电机三相电流线的电压波形，转动一周后，可以发现示波器上出现多个sin函数的波形，我们将每个产生sin函数的周期视作电机定子的一对极转动电角度一周所需的周期。

无刷电机控制可以分为有感和无感两种，其本质差别在于**是否使用编码器作为位置传感器实时读取电机机械角度的绝对位置**。无感多用设置观测器（如自动控制原理中的基本的<u>龙伯格观测器、滑膜观测器</u>等）或检测反电动势以及建立电机观测模型等方式用以替代直接检测位置的方式，通过间接手段实现对于电机位置的观测并进行控制量的补偿。由于观测器设计基于自动控制原理和simulink仿真，本文暂时先对与有位置传感器的电机FOC控制方式进行讨论。

### 1.2 FOC控制原理

**FOC**，全称 **Field-Oriented Control**，即磁场定向控制，也被称为矢量控制。是目前无刷电机（BLDC）和永磁同步电机（PMSM）高效控制的最优方法之一。其核心思想是将电机定子的**电流**分解为两个正交的分量：用于产生**磁场**的**激励磁分量**d轴和用于产生**转矩**的**转矩分量**q轴，通过对电机的转矩和磁场进行独立的控制，从而达到快速响应和精确控制的目的。

一般来说，我们将电机定子绕组的线圈简化为**三相电流/电压输出的模型**。通过<u>Park逆变换和Clark逆变换</u>，我们得以对这个模型进行进一步简化，实现以少变量控多变量、多维问题简化为低维问题的工程数学思想。

![BQACAgUAAyEGAASHRsPbAAERZitppoS9BfuKNNzPOTysP0lOaQYf6QACAiIAAt7rOVUUGfyGtBAiAAE6BA.png](https://img.remit.ee/api/file/BQACAgUAAyEGAASHRsPbAAERZitppoS9BfuKNNzPOTysP0lOaQYf6QACAiIAAt7rOVUUGfyGtBAiAAE6BA.png)

总的来说，FOC控制的实现步骤如下：

1. 对电机**三相电流**进行**采样**得到`Ia`,`Ib`,`Ic` 
2. 将`Ia`,`Ib`,`Ic` 经过**Clark变换**得到`Iα`,`Iβ`
3. 将`Iα`,`Iβ` 经过**Park变换**得到`Iq`,`Id`
4. 计算 `Iq`,`Id`和其设定值`Iq_ref`,`Id_ref `的误差
5. 将上述误差输入两个PID（只用到PI）控制器，得到输出的控制电压 `Uq`,`Ud`
6. 将`Uq`,`Ud` 进行**反Park变换**得到 `Uα,``Uβ`
7. 用`Uα`,`Uβ` 合成电压空间矢量，输入**SVPWM模块**进行调制，输出该时刻三个半桥的状态编码值（后文有提到）
8. 通过MCU按照前面输出的编码值控制三相逆变器的MOS管开关，驱动电机
9. 循环上述步骤

下面来简单说明一下Park变换和Clarke变换的内容以及计算后得到的关系：

#### 1.2.1 Clarke变换

**克拉克变换**是将由电机定子简化后的三相线圈的对应相位相差120°的相电流/电压简化到两相静止坐标系**α-β**的过程，进行一个简单的坐标轴投影计算。最终得到以下变化：

![BQACAgUAAyEGAASHRsPbAAERYh1ppY1nrF7h7L1-RDILJxLZEUnRgAACTyMAAtluMVXBfT6dJgcLlzoE.png](https://img.remit.ee/api/file/BQACAgUAAyEGAASHRsPbAAERYh1ppY1nrF7h7L1-RDILJxLZEUnRgAACTyMAAtluMVXBfT6dJgcLlzoE.png)

写成**矩阵形式**得到：

![BQACAgUAAyEGAASHRsPbAAERYjhppY9Am4aw3JhBaQbEJ26c789F7wACcCMAAtluMVV99m7mUVYHgzoE.png](https://img.remit.ee/api/file/BQACAgUAAyEGAASHRsPbAAERYjhppY9Am4aw3JhBaQbEJ26c789F7wACcCMAAtluMVV99m7mUVYHgzoE.png)

由上可知，得到的前后`**Iα**并不相等。我们可以将经过变换后的**Iα**乘以**2/3**，用以简化运算，此步骤得到的电流称为克拉克变换的**等幅值形式**。

为什么要进行等幅值变换呢？主要出于以下工程便利性：

- **所见即所得：** 变换后的幅值等于相幅值，便于监控和调试。
- **保护设定简单：** 过流/过压阈值无需换算。

- **SVPWM 匹配：** 电压矢量长度与相电压峰值对应，调制逻辑直观。

部分相关从业者同样认为，等幅值变换同样是出于对保证变换后电功率守恒的必要操作，在这里不做过多赘述。

#### 1.2.2 Park变换

**帕克变换**是通过固定的静止坐标系**α-β**，通常为转子的**机械角度**，并通过固定在转子上的**d-q**旋转坐标系，得到转子的**电角度**，取d轴和α轴的夹角，将静止坐标系的**Iα**和**Iβ**向旋转坐标系的两轴进行固定，得到用以计算磁激励分量的**Id**和用以计算电机转矩的转矩分量**Iq**。

![img](https://pic4.zhimg.com/v2-d15d057327992a5c50016aea5bb7201b_1440w.jpg)

变换公式如下：

![BQACAgUAAyEGAASHRsPbAAERYjJppY6yCEB8qG90k6RajW9OjFMY3wACaiMAAtluMVVTbAsS8EDK0DoE.png](https://img.remit.ee/api/file/BQACAgUAAyEGAASHRsPbAAERYjJppY6yCEB8qG90k6RajW9OjFMY3wACaiMAAtluMVVTbAsS8EDK0DoE.png)

同样的，得到公式的**矩阵形式**：

![BQACAgUAAyEGAASHRsPbAAERYjNppY7GF2LjdylVV44-NZo0g8NztAACayMAAtluMVXtiJaFIy70gDoE.png](https://img.remit.ee/api/file/BQACAgUAAyEGAASHRsPbAAERYjNppY7GF2LjdylVV44-NZo0g8NztAACayMAAtluMVXtiJaFIy70gDoE.png)

由线性代数的知识我们清楚，`Id`，`Iq`与`Iα`，`Iβ之间作用了一个旋转矩阵。也就是说，这个*d−q* **坐标系**是始终跟着转子旋转的。

## 二、SVPWM调制原理

### 2.1 浅析SPWM和SVPWM

如果说 **FOC（磁场定向控制）** 是电机控制系统的 **“大脑”**，负责高精度的策略决策与力矩计算；那么 **SPWM 和 SVPWM** 技术就相当于系统的 **“嘴巴”**，负责将抽象的控制指令转化为具体的执行动作。

FOC 的核心价值在于通过Clark/Park坐标变换解耦电流分量，经过PI闭环控制调节后计算出**理想的电压矢量参考值**；而 SPWM 与 SVPWM 则是**调制层的执行手段**，负责将这些电压参考值映射为逆变器开关三相桥臂所需的 **PWM 占空比与开关序列**。

**SPWM**的全称是（Sinusoidal PWM，正弦脉冲宽度调制）。它用一段幅值相等的脉冲序列去等效正弦波，因为两段脉冲信号的面积相同，其对外表现得效果是相同的。SPWM就是将正弦调制波与三角载波作用，得到开关管的开关信号，从而等效出三相正弦电压，例如下图为某一相的方波电压的等效为正弦电压。将该电压输入电机中，可以控制电机运转。

![img](https://pica.zhimg.com/v2-ff0190d21523781bb022334cb3752114_1440w.jpg)

可以看出，SPWM的设计思路是：如果在电机定子的三相绕组中通入**相位互差120°**的正弦波，那么会在空间上产生一个旋转的磁动势，从而带动转子旋转。SPWM就是通过设置开关管的通断，分别产生互差120°三相正弦电压，使其在电机中产生三相电流，从而形成所需的**旋转磁动势**。

但实际上SPWM的工作范围是无法覆盖整个供电电压范围的。<u>由于对电机的做功仅仅取决于最高和最低电压差</u>，因此，SPWM仍有约**13.4%**的不做功的值。由此，SVPWM出现的目的，正是为了在SPWM的波形基础上，使波形能够覆盖整个供电电压范围，提高能量利用效率。

SVPWM的思路则是一开始就从**结果量**入手。由于SPWM最终的目的就是为了产生一个空间上的旋转磁动势，那么SVPWM就直接从该处入手，不拘泥于单独产生三相电压，而是通过设置开关管的通断，直接在电机中形成一个旋转的电压矢量，从而产生一个旋转的磁动势。

SVPWM最终的三相调制波表现为**马鞍波**。即SVPWM的调制波为马鞍波。

![img](https://pic3.zhimg.com/v2-cad7d3d9e91516db949a939840d4e74c_1440w.jpg)

最终，将获得的PWM波分别作用在三相对应的上下桥臂上，逆变器输出的三相电压便可以在空间上合成一个**旋转电压**，形成旋转磁场，实现对电机的控制。

### 2.2 扇区判断与电压矢量选用

SVPWM的实现过程分为：

- 判断过程：根据转子位子和采集到的电流数据确定需给定的`Uα`、`Uβ` → 判断`Uα`、`Uβ`合成电压U*的扇区。

- 执行过程：根据扇区选择产生`U*` 所需用到的电压矢量 → 计算开关管的保持时间 → 通过开关管的开断生成`U*`。

由图。图中为控制电机定子绕组的三相电压的**三相逆变电路**，实现将直流信号转变为三相的交流信号。设定控制**u、v、w**每相的两个MOS管状态如下：当上MOS管断开时，相的状态为**H**;下MOS管断开时，相的状态为**L**。由MOS管**导通情况的不同组合**，能够得到**8种**不同的三相状态。下面的电路中，红色为导通电路，视作对应绕组线圈的状态为**1**。反之为**0**。

![img](https://pic1.zhimg.com/v2-c20243476eef80b5f225764ef69f9894_1440w.jpg)

将空间电压矢量六边形用L1、L2、L3进行划分：

![img](https://picx.zhimg.com/v2-4a6f840398e68c66617f11088242d737_1440w.jpg)

对应的A、B、C有对应的01作为数值，那么将不同的电压矢量可视作**ABC**。由此，根据两个值相同，一个值变化的情况下，将相邻的扇区划分不同的区域：

- 以**L1**为中心轴进行划分：

当 `Uβ−√3Uα>0` ，令A=0，此时合成电压位于Ⅱ、Ⅲ、Ⅳ扇区；

当`Uβ−√3Uα<0` ，令A=1，此时合成电压位于Ⅴ、Ⅵ、Ⅰ扇区。

- 以**L2**为中心轴进行划分：

当 `Uβ+√3Uα>0`，令B=0，此时合成电压位于Ⅵ、Ⅰ、Ⅱ扇区；

当`Uβ+√3Uα<0 `，令B=1，此时合成电压位于Ⅲ、Ⅳ、Ⅴ扇区。

- 以**L3**为中心轴进行划分：

当 `Uβ>0`，令C=1，此时合成电压位于Ⅰ、Ⅱ、Ⅲ扇区；

当`Uβ<0` ，令C=0，此时合成电压位于Ⅳ、Ⅴ、Ⅵ扇区。

定义<mark> N=1∗A+2∗B+4∗C</mark>，则可以根据N的值来判断扇区。

![BQACAgUAAyEGAASHRsPbAAERZpJpppk-BJc3PR2gBPH-2AK0uVElMwACjiIAAt7rOVXDFMXoNmo0FzoE.png](https://img.remit.ee/api/file/BQACAgUAAyEGAASHRsPbAAERZpJpppk-BJc3PR2gBPH-2AK0uVElMwACjiIAAt7rOVXDFMXoNmo0FzoE.png)

根据基尔霍夫定理的KCL，当三相状态为**111**或**000**时，不满足基尔霍夫定理，绕组线圈由于无电流通过无法产生磁场，电机在磁场变向过程中在这两处无法受到对应产生的力。因此，除开这两种状态，剩下的6种导通情况囊括电机转动一周的角度，即将电机转子的不同所在的位置划分为6个**扇区**，对应的电压矢量状态设计为U0~U7。对工程便利性的考虑，在变换电路时尽量只控制电路中一个MOS管发生电压正反转，即**最少切换**的原则。矢量0(000)和矢量7(111)作为**零矢量**则通常作为相邻电压矢量变换时候的过渡状态，

![电机控制——聊聊SPWM和SVPWM](https://picx.zhimg.com/70/v2-39c88f5622fff8dc22037bb50b668a10_1440w.image?source=172ae18b&biz_tag=Post)

![img](https://pica.zhimg.com/v2-64b04d9530871e69f1c75c5c506875da_1440w.jpg)

### 2.3 六步换相原理与对比

根据上一节，了解了FOC控制中是如何通过控制三相逆变电路中桥臂的导通情况控制<u>U、V、W</u>三相实现不同磁通量的合成，从而控制转子定向转动的。针对需求的不同转动方向、角度大小、快慢等，都需要由已知的绕组线圈导通状态与实际磁通量方向关系进行一定的策略考量。

按照下图右侧的表格，调整通电线圈的顺序（即左图**绿色**箭头），就可以合成出相应的磁通量（即左图**棕色**箭头），从而使得由永磁体制作的**转子**，被这个合成出来的**等效磁通量**持续牵引，进而驱动其不间断的进行旋转：

![img](http://www.uinio.com/Embedded/FOC/5.gif)

这种方式合成的**磁通量**只存在六个方向，因而被称作**六步换相法**（每一次控制两个相，只能旋转 `60°` 度）。这种控制方式粗暴简单，对MCU的算力要求较低，但在平常测试时肉眼可见电机有明显的抖动，无法"丝滑"地转动，的工作噪音也会比较大。因此，六步换相的应用实在较少，通常只会短暂出现在电机的死区应用情况。

### 2.4 扇区发波（七段式序列）

分配开关管时间一般遵循以下原则：

1）一般采用**7段式**SVPWM算法。

2）为了使电压谐波较小，每次**只动一个开关**。

3）开关管开关周期具有**对称性**。

4）**同时**用到U0和U7。

5）每个周期**从零矢量开始，从零矢量结束**。

![img](https://pica.zhimg.com/v2-002c567e58f6185e48a3f2b1b99ac18c_1440w.jpg)

从矢量合成的原理可知，矢量圆中的任意非零矢量，无论作用先后，都可以利用与它相邻的两个基础矢量合成而来。

假设我们需要在 **第 VI 扇区** 进行合成，该扇区由基本矢量 **V4 (100)** 和 **V5 (101)** 组成。若直接在两个非零矢量之间频繁跳变，或者在扇区切换时处理不当，可能会导致多相桥臂同时动作，增加开关损耗与噪声。为了遵循 **最少切换原则**（即每次状态切换仅改变一相桥臂的通断），引入**零矢量**作为过渡缓冲。通过将发波过程分解为多个步骤，构建对称的**七段式序列**，可以确保每一步仅有一相发生翻转。

以从 **V4 (100)** 过渡到 **V5 (101)** 为例，标准的优化发波序列如下：

1. **起始态**：插入零矢量 **V0 (000)**（所有下桥臂导通）。

2. **第一步**：切换**U相**，状态变为**矢量4 (100)**。（仅 U 相上桥臂导通）

3. **第二步**：切换**W相**，状态变为**矢量5(101)**。（U、W 相上桥臂导通）

4. **中心态**：切换**V相**，进入零矢量**7 (111)**。（所有上桥臂导通，作为对称中心）

5. **第三步**：反向切换**V相**，回到**矢量5 (101)**。

6. **第四步**：反向切换**W相**，回到**矢量4 (100)**。

7. **结束态**：反向切换**U相**，回到**V0 (000)**。五段式相较于七段式有着更多的PWM调试种类，在这里暂时不做过多赘述。按上述原则，可以得到开关管具体的保持时间：

   ![img](https://pic3.zhimg.com/v2-9cd764bd7cc4275aa24a3309e03698fc_1440w.jpg)
   
   算出具体的时间分配后，**需要将该时间信号传递给逆变电路的6个开关管，传递方式为生成PWM波。**

## 三、电机的闭环三环代码控制 

#### 3.1 位置闭环控制

位置闭环控制是 FOC 控制架构中的最外层控制环。其核心目标是**使电机的转子轴能够精确且稳定地跟踪给定的目标机械角度**。在实际应用中，它常用于需要精确角度控制的场景，例如机器人关节、云台等。此处使用的是较为简单的力-位控制：

![BQACAgUAAyEGAASHRsPbAAERYjFppY6G8SRhisMoJev0U8YdwNW-igACZyMAAtluMVVJZFAKqMOrDjoE.png](https://img.remit.ee/api/file/BQACAgUAAyEGAASHRsPbAAERYjFppY6G8SRhisMoJev0U8YdwNW-igACZyMAAtluMVVJZFAKqMOrDjoE.png)

其控制逻辑实现可分为以下几步：

1. **给定目标角度与方向**：由上位机或轨迹规划模块提供一个期望的机械角度 `target_angle`，同时设置`Dir`为1或-1设置电机编码器读取方向，确定电机磁极对数`pp`。

2. **获取实时反馈**：通过IIC通信获取编码器（如 AS5600）读取到的电机当前的真实机械角度 `current_angle`。

3. **计算误差（error）**：计算目标角度与实际角度之间的差值 `angle_error = target_angle - current_angle`。

4. **电角度计算：**

   将电机编码器的机械角度转换为 FOC 控制所需的电角度。

   ```C
   float electricAngle(void)
   {
   	return normalizeAngle((GetAngle_NoTrack() * pp * Dir) - zero_electric_Angle);
   }
   ```

5. **PID 调节器运算**：将角度误差值输入到一个**位置环 PID 控制器**中。经过比例-积分-微分运算后，输出一个控制量。值得注意的是，在标准的三环控制架构（位置环-速度环-电流环）中，位置环的输出直接作为**速度环的期望值**。但在单位置环的简化实现中，该输出量通常直接作为 `Uq`的参考值。在FOC控制算法中，通常认为输出力矩与电压等效。

   ```C
   #define  limit            6.3
   #define  Output_ramp      10000
   
   //限幅
   float  _constrain(float amt, float low, float high)    
   {
   	return ((amt<low)?(low):((amt)>(high)?(high):(amt))); //三元运算符
   }
   
   unsigned long Timestamp_Last = 0.0;
   float Last_Error = 0.0;
   float Last_intergration = 0.0;
   float Last_Output = 0.0;
   float PID_Controller(float Kp, float Ki, float Kd, float Error)
   {
   	float Ts = 0.0;
   	uint32_t Timestamp = SysTick->VAL;
     if(Timestamp < Timestamp_Last) Ts = (float)(Timestamp_Last - Timestamp)/9*1e-6;
   	else
   		Ts = (0xFFFFFF - Timestamp + Timestamp_Last)/9*1e-6;
   	
   
   if(Ts<=0 || Ts > 0.05f) Ts = 0.001;
   
   float proportion = Kp * Error;//P环
   
   float intergration = Last_intergration + Ki * 0.5f * Ts * Error;//I环
   intergration = _constrain(intergration, -limit, limit);
   
   float differential = Kd * (Error - Last_Error)/Ts;//D环
   
   float Output = proportion + intergration + differential;
   Output = _constrain(Output, -limit, limit);
   //每一次到位后更新数据
   Last_Error = Error;
   Last_intergration = intergration;
   Last_Output = Output;
   Timestamp_Last = Timestamp;
   
   return Output;
   
   }
   ```

   我们已知偏差公式：**error=预期位置-偏差位置**。假设我们需要根据角度α求出一个力矩Uq（整个过程我们认为力矩和压降是等效的），使得Uq能够让电机产生力矩，控制电机转子回位。

   **执行 FOC 运算**：将计算出的 `Uq` 与 `Ud`（通常设为零，以最大化利用磁阻转矩，即 `Id=0` 控制策略）以及当前的**电角度**一同送入 `SetPhaseVoltage` 函数。该函数负责执行反 Park 变换和空间矢量合成，最终通过 PWM 模块对输出波进行调制驱动电机。

   ```C
   void SetPwm(float Ua, float Ub, float Uc)
   {
   	float U_a=0.0;
   	float U_b=0.0;
   	float U_c=0.0;
   
   U_a = constrain(Ua, 0.0f, voltage_limit);
   U_b = constrain(Ub, 0.0f, voltage_limit);
   U_c = constrain(Uc, 0.0f, voltage_limit);
   
   dc_a = constrain(U_a / voltage_power_supply, 0.0f, 1.0f);
   dc_b = constrain(U_b / voltage_power_supply, 0.0f, 1.0f);
   
   dc_c = constrain(U_c / voltage_power_supply, 0.0f, 1.0f);
   PWM_Channel1(dc_a * 4800.0f);  // 频率15k
   PWM_Channel2(dc_b * 4800.0f);
   PWM_Channel3(dc_c * 4800.0f);
   
   }
   /FOC核心算法，克拉克逆变换/帕克逆变换
   float test_angle = 0.0;
   float last_test_angle = 0.0;
   void SetPhaseVoltage(float Uq, float Ud, float angle_el)
   {
   //	angle_el = normalizeAngle(angle_el);
   	test_angle = angle_el - last_test_angle;
   	
   	Ualpha = -Uq*sin(angle_el);
   	Ubeta = Uq*cos(angle_el);
   	
   	Ua = Ualpha + voltage_power_supply / 2;
   	Ub = (sqrt(3)*Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
   	Uc = -(Ualpha + sqrt(3)*Ubeta) / 2 + voltage_power_supply / 2;
   	
   	SetPwm(Ua,Ub,Uc);
   	
   	last_test_angle = angle_el;
   }
   ```

由以上代码。通过串口发送位置量给单片机（范围0~6.28），即可实现电机到位。用手掰动电机旋转，能够感受到明显的阻力，即为电机通过位置闭环控制进行误差修正所产生的回复力矩。

#### 3.2 速度闭环控制 

一般来说，速度环既能作为电机**单独的精确控速**使用，同样也能与位置环向配合，实现<u>位置环-速度环-力矩环</u>，控制转子在回位时的速度。相较于不同的，位置闭环控制要求的是编码器返回位置量，速度闭环控制则是速度量。但是，编码器无法直接读取电机的准确速度，因此，需要一些外部程序辅助编码器对电机位置信息进行处理。FOC部分的代码基本保持不变。

![BQACAgUAAyEGAASHRsPbAAERYkJppZBfPKpR4A-8tBczh_bYahmGlwACfCMAAtluMVUOeVV1qbc8MToE.png](https://img.remit.ee/api/file/BQACAgUAAyEGAASHRsPbAAERYkJppZBfPKpR4A-8tBczh_bYahmGlwACfCMAAtluMVUOeVV1qbc8MToE.png)

1.**获取编码器速度**

```C
float GetVelocity(void)
{
	float dt = 0.0; //时间差
	float Vel_ts = SysTick -> VAL; //读取SysTick计数器当前时间戳
	//检查计数器是否溢出
	if(Vel_ts < Last_Vel_ts) dt = (Last_Vel_ts - Vel_ts)/9*1e-6f;
	else dt = (0xFFFFFF - Vel_ts + Last_Vel_ts)/9*1e-6f;
	
//防dt过小出现零除，导致数据溢出
if(dt < 0.0001) dt = 10000; 
float Vel_Angle = GetAngle();
float dv = Vel_Angle - Vel_Last_Angle;
//数值微分，计算顺瞬时角速度
float velocity = (Vel_Angle - Vel_Last_Angle)/dt;
//更新时间戳与上次角度
Last_Vel_ts = Vel_ts;
  Vel_Last_Angle = Vel_Angle;
  return velocity;
}
```

速度值就可以间接得到了。但是，此时得到的速度信号相对来说噪声与波动较大。需要引入一个低通滤波对信号进行**平滑处理**。

#### 3.2.1滞后一阶低通滤波器设计

```C
// 低通滤波器部分代码，时间戳dt越短，越采取以前的值
// 滤波器会有相位滞后的影响
float y = 0;
float Lowpassfilter_sim(float x)
{
	float out = 0.9*x + 0.1*y;
	y = x;
	return out;
}
uint32_t Last_Timesamp = 0.0;
float Last_y = 0.0;
float Lowpassfilter(float Tf, float x)
{
	float dt = 0.0;
uint32_t Timesamp = SysTick->VAL;
//计算两次循环的时间间隔并转换为秒
if(Timesamp < Last_Timesamp) dt = (float)(Last_Timesamp - Timesamp)/9*1e-6;
else
	dt = (float)(0xFFFFFF - Timesamp + Last_Timesamp)/9*1e-6;
if(dt<0.0||dt==0) dt = 0.0015f;
else if(dt>0.005f)
{
	Last_y = x;
	Last_Timesamp = Timesamp;
	return x;
}
//低通滤波的主要函数实现
float alpha = Tf / (Tf + dt);
float y = alpha * Last_y + (1.0f - alpha) * x;
Last_y = y;
Last_Timesamp = Timesamp;
return y;
}
```

观察主要的低通滤波实现函数，`Tf`为自己通过调试确定的一个时间常数，与截止频率`fc`成反比。输出的`y`是经过低通滤波修正后的加权速度信号。当时间戳`dt`跳变过大时，滤波器会调整`alpha`变小，即会更少地采用前一个采样点的值。注意一点，`Tf`是根据有效信号频率来的，截止频率处信号幅值衰减为原来的**0.707**。设置太低，相对应的截止频率信号幅值变低，相位滞后；设置太高，滤波效果差。

#### 3.2.2速度PI控制器

```C
float PID_Controller(float Kp, float Ki, float Kd, float Error)
{
	float Ts = 0.0;
	uint32_t Timestamp = SysTick->VAL;
  if(Timestamp < Timestamp_Last) Ts = (float)(Timestamp_Last - Timestamp)/9*1e-6;
	else
		Ts = (0xFFFFFF - Timestamp + Timestamp_Last)/9*1e-6;
if(Ts<=0 || Ts > 0.05f) Ts = 0.001;
float proportion = Kp * Error;//P环
//tustin变换的离散积分计算公式
float intergration = Last_intergration + Ki * 0.5f * Ts * Error;//I环
intergration = _constrain(intergration, -limit, limit);
float differential = Kd * (Error - Last_Error)/Ts;//D环
float Output = proportion + intergration + differential;
Output = _constrain(Output, -limit, limit);
Last_Error = Error;
Last_intergration = intergration;
Last_Output = Output;
Timestamp_Last = Timestamp;
return Output;
}
```

该PI控制环首先通过SysTick定时器差值计算采样周期Ts并处理计数器回绕与异常限幅以确保离散化时间基准准确，比例项P直接将当前误差乘以Kp实现快速响应，积分项I采用Tustin梯形变换，简单来说就是梯形面积近似积分的数值方法：

```
Last_intergration + Ki * 0.5f * Ts * Error
```

，相比前向欧拉法具有更高精度与稳定性，能有效消除稳态误差，同时积分累加值与最终输出均经过`_constrain`函数限幅处理以防止积分饱和与执行器超限，最后更新`Last_Error`、`Last_intergration`及`Timestamp_Last`等静态状态变量供下一周期迭代使用，从而构成完整的数字PI闭环控制逻辑。

通过在原有的闭环控制添加与修改这些项，就能得到一个对速度进行闭环控制的电机。

#### 3.3 电流闭环控制    

前面两个闭环控制有一个共同点，就是都是使用了力矩环作为**内环控制**，而在FOC控制中，力矩又与电压存在着较强的等效关系。但是，牵涉到电压，就难免需要考虑到电机工作时间过长引起的发热，导致相内阻变大的问题，而我们给定的`Uq`并没有及时调整，导致相电流`Iq`从而会使得力矩发生衰减。因此，电流环的工作就是需要对相电流进行实时的采样与监控。

显然，通过单片机IO引脚配置的ADC采样我们就能够得到实时的数值，虽然是电压值，但通过欧姆定律就能进行简单的转换成三相电流，再通过克拉克变换和帕克变换就能得到ADC读取的`Iq`值，即电流传感器的电流采样值，并需要与算法中的`Iq`即目标值进行区分。

下面是电流采样工作的代码实现：

```C
void CurrSense_Init()
{
	AD_Init(); //初始化ADC模块
	DriftOffsets();
    _shunt_resistor = 0.01; //采样电阻数值 
    amp_gain = 50; //电流传感器放大倍数
    vlots_to_amps = 1.0f / _shunt_resistor / amp_gain;  
    //得到测取的相电流数值
    gain_a = vlots_to_amps * -1;
    gain_b = vlots_to_amps * -1;
    gain_c = vlots_to_amps;
    //注意三相电流汇入同一节点
}
```

读取全部三相电流，由**KCL**，实际上我们只需要读取两相的电流：

```C
struct CurrentDetect GetPhaseCurrent()
{
	struct CurrentDetect current;
	float tran_vol_a = (float)Samp_volts[0]*_ADC_CONV;
	float tran_vol_b = (float)Samp_volts[1]*_ADC_CONV;
	

    current.I_a = (tran_vol_a - offset_ia)*gain_a;
    current.I_b = (tran_vol_b - offset_ib)*gain_b;
    current.U_a = (tran_vol_a - offset_ia) ;
    current.U_b = (tran_vol_b - offset_ib) ;
    return current;
}
```

实际上，电流传感器模块并非**理想器件**，在通常情况下可能发生零漂。因此，需要进行**零漂检测**，得到零漂检测值，即上面代码的`offset_ia、offset_ib`，需要减去零漂值对采样结果进行更加精确化的修正：

```C
void DriftOffsets()
{
	uint16_t detect_rounds = 1000; //采样多轮零漂值
	for(int i = 0; i < detect_rounds; i++)
	{
		offset_ia += Samp_volts[0]*_ADC_CONV;
		offset_ib += Samp_volts[1]*_ADC_CONV;
	}
	offset_ia = offset_ia / detect_rounds;
	offset_ib = offset_ib / detect_rounds;
}
```

后续在FOC的代码中并未有过多修改。只需在转化成控制对象`Uq`的代码中设定电流PID控制环内部error（即`Dir *(Target - Cur_q )`的值）为目标电流-实际电流，并产生一个期望的修复力矩即可。

```C
void Set_CurTorque(float Target)
{
	struct CurrentDetect Current = GetPhaseCurrent();
	float Cur_q = Lowpassfilter(0.01, cal_Iq_Id(Current.I_a, Current.I_b, electricAngle()));
	float Iq = PID_Controller(Kp, Ki, Kd, Dir *(Target - Cur_q ));
    SetPhaseVoltage(Iq, 0, electricAngle());
}
```

### <mark>参考**Reference**</mark>

[1]【自制FOC驱动器】深入浅出讲解FOC算法与SVPWM技术 - 知乎]  稚晖君(https://zhuanlan.zhihu.com/p/147659820)

[2]【直流无刷电机】BLDC六步换向法控制原理及Matlab/Simulink仿真分析 - 知乎](https://zhuanlan.zhihu.com/p/609517326)

[3]  FOC开环速度代码的撰写](http://dengfoc.com/#/dengfoc/灯哥手把手教你写FOC算法/4序FOC开环速度代码的撰写)

[4]  电机控制——聊聊SPWM和SVPWM - 知乎](https://zhuanlan.zhihu.com/p/627810239) 

[5] [剖析无刷电机的 FOC 磁场定向控制算法 - UinIO.com 电子技术实验室](http://www.uinio.com/Embedded/FOC/#more)

[6] 电机控制——聊聊SPWM和SVPWM(https://zhuanlan.zhihu.com/p/627810239)
