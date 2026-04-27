# 《四足机器人控制算法——建模、控制与实践》学习路线图

> 创建时间：2026-04-27　|　最后更新：2026-04-27
> 学习者背景：机械工程硕士，BMS 嵌入式开发，机器人业余爱好者
> 学习目标：面试/求职准备，掌握机器人仿真基础并在本地环境跑通
> 硬件情况：WSL2 PC（x86 Ubuntu, 运行 Gazebo 仿真），无真实机器人
> 环境方案：Ubuntu 18.04 + ROS Melodic + Gazebo，纯 WSL2 单机
> 配套书籍：《四足机器人控制算法——建模、控制与实践》（共 11 章）

---

## 一、书本章节 ↔ 仓库代码完整对照表

| 章节 | 内容 | 对应代码文件 | 关键函数/类 |
|------|------|------------|-----------|
| **Ch1** 概述与实践准备 | 背景与课程准备 | `README.md` | — |
| **Ch2** 关节电机 | 电机控制 | `unitree_actuator_sdk/` 全部 | 电机SDK API |
| **Ch3** 仿真与控制框架 | ROS/Gazebo、面向对象、FSM | `launch/gazeboSim.launch` | 仿真启动 |
| | | `src/FSM/FSM.cpp`, `FSMState.cpp` | FSM 状态注册/切换 |
| | | `src/interface/IOROS.cpp`, `IOSDK.cpp` | I/O 接口 |
| | | `CMakeLists.txt` | 编译配置 |
| **Ch4** 刚体运动学 | 旋转矩阵、欧拉角 | `include/common/`（数学工具） | 旋转/变换矩阵 |
| **Ch5** 单腿运动学与静力学 | ★ 核心 | | |
| | 5.1 正运动学 | `src/common/unitreeLeg.cpp` | `calcF()` 正解 |
| | 5.2 逆运动学 | `src/common/unitreeLeg.cpp` | `calcQ()` / `calcInvQ()` 逆解 |
| | 5.3 微分运动学 | `src/common/unitreeLeg.cpp` | 雅可比矩阵 |
| | 5.4 摆动腿控制 | `src/control/BalanceCtrl.cpp` | 摆动相轨迹 |
| | 5.5 实践：让腿动起来 | `src/FSM/State_SwingTest.cpp` | 单腿摆动测试 |
| **Ch6** 四足运动学与动力学 | | | |
| | 6.1 四足运动学 | `src/common/unitreeRobot.cpp` | 12 关节→机身整体 |
| | 6.2~6.4 动力学 & 简化模型 | `src/control/BalanceCtrl.cpp` | 机身动力学方程 |
| | 6.5 实践：原地姿态控制 | `src/FSM/State_FixedStand.cpp` | 站立控制 |
| **Ch7** 状态估计器 | | | |
| | 7.1~7.9 KF 理论 | `src/control/Estimator.cpp` | `run()` 估计更新 |
| | 7.4 二次规划 | `src/quadProgpp/QuadProg++.cc` | QP 求解器 |
| | 7.10 实践 | `src/control/Estimator.cpp` | 完整估计器 |
| **Ch8** 平衡控制器 | ★ 核心 | | |
| | 8.1 位姿反馈控制 | `src/control/BalanceCtrl.cpp` | `bodyPostureCtrl()` |
| | 8.2 足端力控制 | `src/control/BalanceCtrl.cpp` | 足端力计算/分配 |
| | 8.3~8.5 逆向动力学 & QP | `src/control/BalanceCtrl.cpp` + `quadProgpp/` | 力优化分配 |
| | 8.6 实践 | `src/FSM/State_BalanceTest.cpp` | 平衡测试状态 |
| **Ch9** 步态与轨迹规划 | ★ 核心 | | |
| | 9.1 常用步态 | `src/Gait/WaveGenerator.cpp` | 触地相序生成 |
| | 9.2 落脚点规划 | `src/Gait/GaitGenerator.cpp` | 足端落点 |
| | 9.3 摆动足端轨迹 | `src/Gait/GaitGenerator.cpp` | 摆动相轨迹插值 |
| | 9.4 实践 | `src/Gait/` 全部 | 步态代码 |
| **Ch10** 行走控制器 | ★ 整合核心 | | |
| | 10.1~10.2 State_Trotting | `src/FSM/State_Trotting.cpp` | Trot 状态主逻辑 |
| | 10.3 实时进程 | `src/control/ControlFrame.cpp` | `run()` 1kHz 循环 |
| | 10.4 运行流程 | `main.cpp` → `ControlFrame` → `State_Trotting` | 完整调用链 |
| **Ch11** 感知与导航 | ROS 导航栈 | `unitree_move_base/` | move_base 集成 |

---

## 二、分阶段学习路线

### [ ] Phase 0：环境搭建（1~2天）

**对应书籍：** 第 3 章（3.1 ROS/Gazebo、3.5 编译、3.4 仿真实践）

**目标：** 让机器人在 Gazebo 中站起来并走两步

---

**0.1 确认 WSL2 已安装并启用 WSLg（图形界面支持）**

以管理员身份打开 PowerShell 或 Windows Terminal，执行：

```powershell
wsl --install -d Ubuntu-18.04
# 如果已安装 WSL2，确认版本：
wsl -l -v
```

确保 WSL 版本为 `2` 且具有 WSLg 支持（Windows 10 21H2+ / Windows 11）。验证：

```bash
# 在 WSL2 终端中
echo $DISPLAY
# 应输出类似 :0，表示 GUI 可用
```

---

**0.2 安装 ROS Melodic + Gazebo**

```bash
# 1) 配置 Ubuntu 软件源（如果 WSL 的 18.04 默认源失效）
sudo sed -i 's/archive.ubuntu.com/old-releases.ubuntu.com/g' /etc/apt/sources.list
sudo apt update && sudo apt upgrade -y

# 2) 添加 ROS 源
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update

# 3) 安装 ROS Melodic 完整版（含 Gazebo）
sudo apt install ros-melodic-desktop-full -y

# 4) 安装构建工具
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y

# 5) 初始化 rosdep
sudo rosdep init
rosdep update

# 6) 环境变量
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 7) 安装额外的 Gazebo ROS 包（本工程依赖）
sudo apt install ros-melodic-gazebo-ros-control ros-melodic-ros-controllers ros-melodic-controller-manager ros-melodic-joint-state-controller -y
```

验证：
```bash
roscore
# Ctrl+C 退出后无报错即成功
gazebo --version
# 应显示 Gazebo 9.x
```

---

**0.3 创建工作区并克隆代码**

```bash
# 创建工作区
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src

# 克隆三个仓库
git clone https://github.com/unitreerobotics/unitree_guide.git
git clone https://github.com/unitreerobotics/unitree_ros.git
git clone https://github.com/unitreerobotics/unitree_ros_to_real.git

# unitree_ros_to_real 只有一个 unitree_legged_msgs 需要编译，其他包不要
# 创建隔离目录（只保留 msgs）
cd ~/catkin_ws/src
# 方法：用 CATKIN_IGNORE 屏蔽不需要的包
touch unitree_ros_to_real/unitree_legged_real/CATKIN_IGNORE
```

---

**0.4 编译**

```bash
cd ~/catkin_ws

# 首次编译（关闭 DEBUG 避免 Python2 依赖问题）
catkin_make -DCATKIN_MAKE=ON -DSIMULATION=ON -DREAL_ROBOT=OFF -DDEBUG=OFF -DMOVE_BASE=OFF
```

如果编译失败，参见 **[常见故障排查](#五常见故障排查)**。

---

**0.5 启动仿真**

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch unitree_guide gazeboSim.launch
```

Gazebo 窗口打开，出现 Go1 机器人模型（暂时悬空，等待控制器下发放下）。

---

**0.6 启动控制器**

另开一个 WSL2 终端：

```bash
source ~/catkin_ws/devel/setup.bash
./devel/lib/unitree_guide/junior_ctrl
```

机器人应落下躺在地面，终端打印 FSM 状态信息。

---

**0.7 键盘控制机器人行走**

在控制器终端中（确保终端窗口处于**激活/聚焦**状态）：

| 按键 | 功能 |
|------|------|
| `2` | Passive → FixedStand（站立） |
| `4` | FixedStand → Trotting（开始行走） |
| `W` `A` `S` `D` | 前进/左移/后退/右移 |
| `J` `L` | 左转/右转 |
| `空格` | 停止行走，恢复站立 |

> **如果按键无反应**：先点击控制器终端窗口使其获得焦点，再按按键。

---

**0.8 (可选) 一键启动脚本**

```bash
cat > ~/start_go1_sim.sh << 'EOF'
#!/bin/bash
SESSION="go1_sim"

# 终端1：启动 Gazebo
gnome-terminal -- bash -c "source ~/catkin_ws/devel/setup.bash; roslaunch unitree_guide gazeboSim.launch; exec bash"
sleep 5

# 终端2：启动控制器
gnome-terminal -- bash -c "source ~/catkin_ws/devel/setup.bash; ~/catkin_ws/devel/lib/unitree_guide/junior_ctrl; exec bash"
EOF
chmod +x ~/start_go1_sim.sh
```

---

### [ ] Phase 1：程序骨架 — main → 控制循环 → FSM（2天）

**对应书籍：** 第 3 章（3.2 面向对象、3.3 有限状态机）

**阅读顺序：**
1. `src/main.cpp` — 程序入口，初始化 ControlFrame 和 FSM
2. `src/control/ControlFrame.cpp` — `run()` 主循环：Input → Estimator → FSM → Output
3. `src/FSM/FSM.cpp` — 状态注册与切换逻辑
4. `src/FSM/FSMState.cpp` — 状态基类（enter/run/exit）

**关键问题（面试）：**
- 控制循环的主频是多少？（1kHz）
- FSM 状态切换的触发条件是什么？（键盘按键 / 自动条件）
- 画出一个完整的控制循环流程图

---

### [ ] Phase 2：站立控制（1天）

**对应书籍：** 第 3 章 3.4 + 第 6 章 6.5

**阅读顺序：**
1. `src/FSM/State_Passive.cpp` — 初始态，关节不输出力
2. `src/FSM/State_FixedStand.cpp` — 固定姿态站立（PD 控制 + 重力补偿）
3. `src/FSM/State_FreeStand.cpp` — 柔顺站立

---

### [ ] Phase 3：单腿运动学 ★（2天）

**对应书籍：** 第 4 章 + 第 5 章

**配合阅读：**
| 书本章节 | 代码文件 | 核心概念 |
|---------|---------|---------|
| 4.1~4.6 旋转矩阵/欧拉角 | `include/common/` 头文件 | 坐标变换基础 |
| 4.7 实践 | `src/common/unitreeLeg.cpp` | C++ 矩阵运算实现 |
| 5.1 正运动学 | `unitreeLeg::calcF()` | 关节角 → 足端位置 |
| 5.2 逆运动学 | `unitreeLeg::calcQ()` / `calcInvQ()` | 足端位置 → 关节角 |
| 5.3 微分运动学 | `unitreeLeg` 中的雅可比 | 关节速度 → 足端速度 |
| 5.5 实践 | `src/FSM/State_SwingTest.cpp` | 单腿摆动可调参 |

**面试重点（⭐⭐⭐）：**
- Go1 机器人腿部构型（髋关节横滚/俯仰 + 膝关节俯仰）
- 逆运动学几何推导过程
- 雅可比矩阵在力控制中的作用

---

### [ ] Phase 4：整机运动学与动力学（2天）

**对应书籍：** 第 6 章

**阅读顺序：**
1. `src/common/unitreeRobot.cpp` — 12 个关节的统一管理
2. `src/control/BalanceCtrl.cpp` 中动力学相关部分
3. `src/FSM/State_FixedStand.cpp` — 姿态控制如何作用于四条腿

---

### [ ] Phase 5：状态估计器（2天）

**对应书籍：** 第 7 章

**阅读顺序：**
1. `src/control/Estimator.cpp` — `run()` 入口
2. 理解估计了哪些状态量：机身姿态（roll/pitch/yaw）、高度、速度
3. 卡尔曼滤波器的实现
4. `src/quadProgpp/QuadProg++.cc` — QP 求解器（为 Ch8 打基础）

**面试重点（⭐⭐）：**
- 状态估计器使用了哪些传感器？(IMU + 足端力)
- 估计了哪些状态？
- KF 的预测和更新步骤

---

### [ ] Phase 6：平衡控制器 ★★（3天）

**对应书籍：** 第 8 章

**阅读顺序：**
1. `src/control/BalanceCtrl.cpp` 整体结构
   - `bodyPostureCtrl()` — 机身姿态反馈 PD
   - `bodyHeightCtrl()` — 高度控制
   - 足端力计算与分配
2. QP 求解器的调用 — 如何将力分配转化为优化问题
3. `src/FSM/State_BalanceTest.cpp` — 单轴平衡测试

**面试重点（⭐⭐⭐）：**
- 虚拟模型控制（VMC）的基本思想
- 为什么需要 QP 优化？（约束：摩擦锥、足端不能拉力）
- 力 → 关节力矩的映射（通过雅可比转置）

---

### [ ] Phase 7：步态与轨迹规划 ★★（2天）

**对应书籍：** 第 9 章

**阅读顺序：**
| 文件 | 功能 | 对应书籍小节 |
|------|------|------------|
| `WaveGenerator.cpp` | 四足触地相序生成（频率/占空比/相位） | 9.1 常用步态 |
| `GaitGenerator.cpp` | 步态生成（落脚点 + 足端轨迹） | 9.2 + 9.3 |
| `FeetEndCal.cpp` | 足端位置 → 关节角（调用 IK） | 5.2 |

**关键概念（面试 ⭐⭐⭐）：**
- Trot 步态：对角腿同时触地（1-3, 2-4），相位差 0.5
- 摆动相轨迹：Bezier 曲线 / 多项式插值
- 支撑相：足端相对机身匀速后退
- 波形发生器的工作原理：周期方波信号控制触地/摆动

---

### [ ] Phase 8：行走控制器整合 ★★（2天）

**对应书籍：** 第 10 章

**阅读顺序：**
1. `src/FSM/State_Trotting.cpp` — Trot 状态主逻辑
2. 理解第 10 章的**运行流程**：键盘输入 → 步态参数调整 → 步态生成 → 平衡控制 → 关节指令
3. 将 Phase 1~7 的知识串联成一条完整的链路

**完整调用链：**
```
main()
  → ControlFrame::run()          [1kHz 实时循环]
    → IOROS::recv()              [接收传感器/遥控数据]
    → Estimator::run()           [状态估计]
    → FSM::run()                 [当前状态运行]
      → State_Trotting::run()
        → GaitGenerator::run()   [步态生成]
        → BalanceCtrl::run()     [平衡控制]
    → IOROS::send()              [发送关节指令]
```

---

### [ ] Phase 9：感知与导航（可选，1天）

**对应书籍：** 第 11 章

**阅读：** `unitree_move_base/` 配置、launch 文件、worlds

---

## 三、面试重点优先级

| 优先级 | 模块 | 面试常问题 |
|--------|------|-----------|
| ⭐⭐⭐ | Ch5 + `unitreeLeg` | 正/逆运动学代码实现细节 |
| ⭐⭐⭐ | Ch8 + `BalanceCtrl` | 虚拟力如何分配到四足、QP 的作用 |
| ⭐⭐⭐ | Ch9 + `Gait/` 三文件 | Trot 步态相位、波形生成原理 |
| ⭐⭐ | Ch7 + `Estimator` | KF 估计了什么状态量 |
| ⭐⭐ | Ch10 + `State_Trotting` | 整个控制循环的流程 |
| ⭐ | Ch3 + FSM 框架 | 状态机设计模式、状态切换条件 |

---

## 四、学习建议

1. **阅读方法：** 一章书 → 对应代码 → 动手实验，交替推进
2. **调试技巧：**
   - 改 `CMakeLists.txt` 中 `DEBUG ON` 开启 Python 调试输出（需先 `sudo apt install python python-numpy`）
   - 改 `ControlFrame` / `GaitGenerator` 中的步态参数（步高、步长、频率）
   - 每次只改一个参数，观察 Gazebo 中机器人的变化
3. **代码风格：** 仓库使用 C++11，面向对象设计，ROS 通信 + 共享内存
4. **开发环境：** 本项目代码可在 WSL2 Ubuntu 中直接用 VS Code 打开（`code .` 启动 WSL2 版 VS Code），或使用 WMware / VirtualBox / Docker 等其他方案

---

## 五、常见故障排查

### Gazebo 启动黑屏或闪退

```bash
# 检查 Gazebo 是否正常安装
gazebo --verbose

# 如果是 WSLg 问题，确认 Windows 版本 >= 21H2
# 查看 Windows 设置 → 系统 → 关于 → Windows 规格

# 如果 Gazebo 模型下载失败导致卡住：
# 离线启动测试（跳过模型下载）
gazebo --verbose /usr/share/gazebo-9/worlds/empty.world
```

### catkin_make 找不到 unitree_legged_msgs

```bash
# 确认 msgs 包在 src 目录下
ls ~/catkin_ws/src/unitree_ros_to_real/unitree_legged_msgs/

# 确认没有 CATKIN_IGNORE 误删
# 只屏蔽 unitree_legged_real，不要屏蔽 unitree_legged_msgs
ls ~/catkin_ws/src/unitree_ros_to_real/unitree_legged_real/CATKIN_IGNORE
```

### 编译报错 "Python2::Python not found"

本工程 DEBUG 模式需要 Python2。两种方案：

**方案 A（推荐）：** 编译时关闭 DEBUG
```bash
catkin_make -DDEBUG=OFF
```

**方案 B：** 安装 Python2 并开启 DEBUG
```bash
sudo apt install python python-numpy -y
catkin_make -DDEBUG=ON
```

### 键盘控制无响应

- 确认点击了运行 `junior_ctrl` 的终端窗口使其获得焦点
- 数字键请按主键盘区数字，非小键盘
- 如果 `WASD` 无反应：尝试先按 `2`（站起），再按 `4`（开始行走），最后按 `WASD` 移动
- 终端如有乱码输出，按 `Ctrl+C` 重启控制器

### roslaunch 找不到包

```bash
# 确认 source 了 setup.bash
source ~/catkin_ws/devel/setup.bash
echo $ROS_PACKAGE_PATH
# 应包含 ~/catkin_ws/src

# 如果找不到 go1_description / unitree_gazebo / unitree_controller：
# 确认 unitree_ros 仓库已克隆并编译成功
ls ~/catkin_ws/src/unitree_ros/
```

### 机器人加载后悬空或掉出地面

Gazebo 物理引擎偶发现象。关闭 Gazebo 窗口重新 `roslaunch` 通常可解决。如果频繁出现，检查：

```bash
# 确认世界文件存在
rospack find unitree_gazebo
ls $(rospack find unitree_gazebo)/worlds/earth.world
```

---

## 六、附录：仓库整体架构图

```
unitree_guide/                   ← 核心控制器（ROS 包）
├── CMakeLists.txt               ← 编译配置（支持 仿真/实机/Go1/A1）
├── launch/gazeboSim.launch      ← Gazebo 仿真启动
├── src/
│   ├── main.cpp                 ← 程序入口
│   ├── common/                  ← 机器人模型与工具
│   │   ├── unitreeRobot.cpp     ← 整机模型（12 关节）
│   │   ├── unitreeLeg.cpp       ← 单腿模型（FK/IK/Jacobian）
│   │   └── LowPassFilter.cpp    ← 低通滤波器
│   ├── control/                 ← 核心控制算法
│   │   ├── ControlFrame.cpp     ← 主控制循环
│   │   ├── Estimator.cpp        ← 状态估计器（KF）
│   │   └── BalanceCtrl.cpp      ← 平衡控制器（VMC + QP）
│   ├── FSM/                     ← 有限状态机
│   │   ├── FSM.cpp              ← 状态机框架
│   │   ├── FSMState.cpp         ← 状态基类
│   │   ├── State_Passive.cpp    ← 被动（初始态）
│   │   ├── State_FixedStand.cpp ← 固定站立
│   │   ├── State_FreeStand.cpp  ← 柔顺站立
│   │   ├── State_Trotting.cpp   ← ★ 行走（Trot）
│   │   └── State_*.cpp          ← 其他测试状态
│   ├── Gait/                    ← 步态规划
│   │   ├── WaveGenerator.cpp    ← 波形发生器（触地相序）
│   │   ├── GaitGenerator.cpp    ← 步态生成器（轨迹规划）
│   │   └── FeetEndCal.cpp       ← 足端位置计算
│   ├── interface/               ← I/O 接口
│   │   ├── IOROS.cpp            ← ROS 通信
│   │   ├── IOSDK.cpp            ← Unitree SDK 通信
│   │   ├── KeyBoard.cpp         ← 键盘控制
│   │   └── WirelessHandle.cpp   ← 遥控器
│   └── quadProgpp/              ← QP 求解器库
│       └── QuadProg++.cc
├── include/                     ← 头文件
└── library/                     ← Unitree SDK 库

unitree_actuator_sdk/            ← 关节电机驱动 SDK（对应 Ch2）

unitree_move_base/               ← ROS 导航栈集成（对应 Ch11）
```
