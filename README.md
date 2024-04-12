# BalanceSimulation

#### 介绍

本人为湖南大学跃鹿战队电控组成员，24赛季平衡步兵电控负责人。本开源项目实现**五连杆轮足机器人**在**MATLAB** 和 **Webots** 两个平台的仿真运行，其中各个模块解耦，软件架构清晰，同时含有丰富的注释，非常适合平衡步兵控制系统的仿真学习，具体运行效果可戳B站视频：
[MATLAB 仿真运行](https://www.bilibili.com/video/BV1q64y1p7My/?spm_id_from=333.999.0.0&vd_source=473002b338ff914cf873eb7360e432a6)
[Webots 仿真运行1](https://www.bilibili.com/video/BV1gm411Q7tt/?spm_id_from=333.999.0.0&vd_source=473002b338ff914cf873eb7360e432a6)
[Webots 仿真运行2](https://www.bilibili.com/video/BV1wz421R7hf/?spm_id_from=333.999.0.0&vd_source=473002b338ff914cf873eb7360e432a6)


#### 功能实现

- **MATLAB**
    基本平衡、腿长控制、抵抗外部干扰、通过盲道、1.2米17度飞坡
- **Webots**
    基本平衡、抵抗外部干扰、腿长控制、航向控制、20度坡面保持底盘水平、1.2米20度飞坡、20cm稳定跳跃、行进过程中跳跃

#### 安装教程

该项目 MATLAB 和 Webots 实现版本均为 **R2023b**，请勿使用低于该版本的软件运行。


#### 使用说明

- **MATLAB 使用说明**
    - 使用 matlab 文件夹下 LQR_calc.m 文件修改平衡步兵相应参数并**计算 LQR 反馈增益**。
    - vmc.m 实现平衡步兵**五连杆映射**，但本项目并未使用该文件，可通过该文件进行相应学习。
    - wheel_leg.slx 包含轮腿机器人的物理模型 **Model**，腿部控制器 **Controller**，速度估计模块 **SpeedEstimation**。
    - **Reference** 模块可给定平衡步兵的参考输入，**Interference** 模块可给定外部干扰，**Model** 中 **Slope** 和 **Blind** 模块分别为坡道和盲道，可根据需要取消对应注释。

- **Webots 使用说明**
    - Webots 项目使用 **VScode** 进行代码编写，**Webots 内置编译器**进行编译调试。
    - 需修改 **webots/.vscode/c_cpp_properties.json** 中 webots库头文件和动态链接库对应目录，同时需要修改 **Makefile** 文件中 **INCLUDE** 头文件路径。
    - 使用 webots 文件夹下 LQR_calc.m 文件修改平衡步兵相应参数并**计算 LQR 反馈增益**。
    - balance_controller.c 中 `main()` 仅添加初始化函数 `BalanceInit()` 和任务运行函数`BalanceTask()`，具体函数实现请看 **balance.c** 文件。 

#### 开源参考

1. [哈尔滨工程大学创梦之翼战队RoboMaster平衡步兵机器人控制系统设计](https://zhuanlan.zhihu.com/p/563048952)
2. [湖南大学跃鹿战队平衡步兵开源代码](https://gitee.com/hnuyuelurm/balance_chassis)
3. V. Klemm et al., "Ascento: A Two-Wheeled Jumping Robot," 2019 International Conference on Robotics and Automation (ICRA), 2019, pp. 7515-7521, doi: 10.1109/ICRA.2019.8793792.
4. [平衡步兵嵌入式技术文档](https://bbs.robomaster.com/forum.php?mod=viewthread&tid=22867)# ECE445-balanced-car
