# 增强型 ROS 2 机器人仿真工作空间

一个专业级的 ROS 2 差速驱动机器人仿真工作区，具备高级传感器集成、参数化配置和全面可视化工具。

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

## 🚀 概述

本项目基于 ROS 2 和 Gazebo 实现了完整的机器人仿真生态系统，主要特性包括：

- **参数化机器人设计**：可完全配置的差速驱动机器人，带激光雷达和相机
- **多环境支持**：默认世界和自定义六边形竞技场环境
- **高级传感器集成**：带噪声建模和真实物理的增强型激光雷达
- **计算机视觉**：基于YOLOv8的实时目标检测系统
- **传统SLAM**：基于 slam_toolbox 的2D激光SLAM系统
- **AI增强视觉SLAM**：基于 ORB-SLAM3 的深度学习参数优化系统
- **自主导航**：完整的 Nav2 导航栈，支持路径规划与避障
- **专业配置管理**：基于 YAML 的参数配置系统
- **全面可视化**：预配置的 RViz 开发与导航视图
- **模块化架构**：机器人描述与仿真环境分离

## 📑 目录

- [🚀 概述](#-概述)
- [📋 先决条件](#-先决条件)
- [🛠️ 安装](#-安装)
- [🎮 基本使用](#-基本使用)
- [🎯 RViz 可视化使用指南](#-rviz-可视化使用指南)
- [🤖 ORB-SLAM3 AI 系统](#-orb-slam3-ai-系统)
- [🚀 ORB-SLAM3 AI 完整使用指南](#-orb-slam3-ai-完整使用指南)
- [🍎 苹果检测系统](#-苹果检测系统)
- [🗺️ 导航系统](#-导航系统)
- [📊 性能监控](#-性能监控)
- [🔧 配置管理](#-配置管理)
- [📖 API参考](#-api参考)

## 🎯 使用流程概览

1. **基础体验** → [快速开始](#-快速开始-1分钟体验) (1分钟)
2. **AI训练** → [模型训练](#训练数据收集与模型训练) (10分钟)
3. **完整系统** → [启动所有功能](#运行ai增强slam系统) (5分钟)
4. **性能优化** → [调优指南](#性能调优与故障排除) (可选)

## 📋 先决条件

### 系统要求
- **ROS 2**（推荐 Humble/Iron）
- **Gazebo Classic**（11.x）或 **Ignition Gazebo**（6.x+）
- **Ubuntu 20.04+** 或支持 ROS 2 的 **macOS**

### 必需 ROS 2 软件包
```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-gazebo-ros-pkgs \
                 ros-${ROS_DISTRO}-robot-state-publisher \
                 ros-${ROS_DISTRO}-joint-state-publisher-gui \
                 ros-${ROS_DISTRO}-xacro \
                 ros-${ROS_DISTRO}-rviz2
```

### 导航依赖（SLAM 和自主导航所需）
```bash
sudo apt install ros-${ROS_DISTRO}-navigation2 \
                 ros-${ROS_DISTRO}-nav2-bringup \
                 ros-${ROS_DISTRO}-slam-toolbox \
                 ros-${ROS_DISTRO}-turtlebot3-teleop
```

### 计算机视觉依赖（视觉系统所需）
```bash
# Python依赖（苹果检测和ORB-SLAM3 AI）
pip3 install ultralytics
pip3 install opencv-python
pip3 install numpy scipy
pip3 install torch torchvision

# ROS2图像桥接
sudo apt install ros-${ROS_DISTRO}-cv-bridge python3-cv-bridge
```

## 🛠️ 安装

### 配置rosdep官方源（推荐）
如需恢复为ROS官方源（适合VPN或海外环境），请执行：
```bash
# 移除清华镜像环境变量（如之前设置过）
sed -i '/ROSDISTRO_INDEX_URL/d' ~/.bashrc
source ~/.bashrc

# 重新初始化rosdep官方源
sudo rm -f /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init
rosdep update
```

### 工作空间设置
1. **设置工作区：**
   ```bash
   cd ros_ws
   # 生成URDF文件
   xacro src/my_robot_description/urdf/my_robot.urdf.xacro > src/my_robot_description/urdf/model.urdf
   # 安装依赖
   export ROS_DISTRO=humble
   rosdep install --from-paths src --ignore-src -r -y
   # 构建工作空间
   colcon build
   source install/setup.bash
   ```

2. **验证安装：**
   ```bash
   # 检查包是否正确安装
   ros2 pkg list | grep -E "(my_robot|description|simulation)"
   
   # 验证URDF文件
   check_urdf src/my_robot_description/urdf/model.urdf
   ```

3. **测试基本功能：**
   ```bash
   # 无界面模式（推荐用于远程服务器）
   ros2 launch my_robot_description my_robot_sim.launch.py gui:=false
   
   # 有界面模式（本地使用）
   ros2 launch my_robot_description my_robot_sim.launch.py
   
   # 测试ORB-SLAM3 AI系统
   ros2 launch orb_slam3_ai orb_slam3_ai.launch.py
   ```

## 📦 包架构

### 🤖 my_robot_description
**核心机器人定义与可视化包**

```
my_robot_description/
├── config/
│   ├── robot_params.yaml     # 机器人物理参数
├── launch/
│   └── my_robot_sim.launch.py # 基本仿真启动器
├── rviz/
│   ├── robot_view.rviz       # 基本机器人可视化
│   └── navigation_view.rviz  # 导航视图
├── urdf/
│   ├── my_robot.urdf.xacro   # 参数化机器人定义
│   ├── model.urdf            # 编译后的URDF
│   └── model.config          # Gazebo模型配置
└── worlds/
    └── my_robot_world.world  # 默认仿真世界
```

**机器人规格：**
- **底盘**：0.3m × 0.2m × 0.1m，真实质量分布
- **驱动系统**：差速驱动，轮距0.22m
- **轮子**：半径0.05m，增强摩擦建模
- **万向轮**：后部球形支撑
- **传感器**：360°激光雷达 + RGB-D深度相机，带高斯噪声
- **性能**：50Hz控制循环，30Hz相机更新，5Hz激光更新

### 🌍 my_robot_simulation  
**高级仿真环境与启动配置**

```
my_robot_simulation/
├── config/
│   └── simulation_params.yaml  # 环境参数
├── launch/
│   ├── my_hex_arena.launch.py  # 六边形竞技场启动器
│   └── combined.launch.py      # 完整仿真启动
└── worlds/
    └── my_hex_arena.world      # 自定义六边形竞技场
```

**环境特性：**
- **六边形竞技场**：半径10m，墙高2m
- **真实物理**：ODE物理引擎，精确材料属性
- **高级照明**：定向光源，带阴影
- **模块化设计**：YAML配置，易于自定义

### 🧭 my_robot_navigation
**SLAM建图与自主导航包**

```
my_robot_navigation/
├── config/
│   ├── slam_toolbox_config.yaml # SLAM配置
│   └── nav2_params.yaml         # 导航参数
├── launch/
│   ├── slam.launch.py           # SLAM建图模式
│   ├── navigation.launch.py     # 自主导航
│   └── slam_navigation.launch.py # SLAM+导航一体
├── maps/                        # 地图文件
│   └── my_robot_map.yaml        # 默认地图
└── scripts/
    ├── save_map.py             # 地图保存工具
    └── map_manager.py          # 地图管理工具
```

**导航特性：**
- **SLAM建图**：基于slam_toolbox的实时建图
- **定位**：基于AMCL的已知地图定位
- **路径规划**：Nav2全局/局部路径规划
- **避障**：动态障碍物检测与避让
- **地图管理**：地图保存、加载与管理工具
- **多模式支持**：SLAM、导航或组合运行

### 👁️ my_robot_vision
**计算机视觉与目标检测包**

```
my_robot_vision/
├── my_robot_vision/
│   └── apple_detector.py       # YOLOv8苹果检测节点
├── test/                       # 测试文件
├── resource/                   # 资源文件
├── setup.py                    # 包配置
├── package.xml                 # 包元数据
└── requirements.txt            # Python依赖
```

**视觉特性：**
- **YOLOv8检测**：基于深度学习的目标检测
- **OpenCV集成**：实时图像处理和分析
- **ROS2集成**：无缝集成ROS2话题系统
- **HSV备选**：传统颜色检测作为备选方案
- **实时显示**：实时显示检测结果和置信度
- **多模型支持**：支持自定义训练模型

### 🤖 orb_slam3_ai  
**ORB-SLAM3 AI 参数优化包**

```
orb_slam3_ai/
├── orb_slam3_ai/
│   ├── slam_ai_optimizer.py    # AI参数优化器
│   └── orb_slam3_ai_node.py    # 主ROS2节点
├── scripts/
│   └── train_ai_model.py       # AI模型训练脚本
├── models/                     # 训练好的模型存储目录
├── config/
│   └── orb_slam3_params.yaml   # ORB-SLAM3与AI配置
├── launch/
│   └── orb_slam3_ai.launch.py  # 完整系统启动文件
├── rviz/
│   └── orb_slam3_ai.rviz       # SLAM可视化配置
├── setup.py                    # 包配置
└── package.xml                 # 包元数据
```

**AI-SLAM特性：**
- **ORB-SLAM3集成**：基于RGB-D的视觉SLAM算法
- **AI参数优化**：使用预训练ResNet-18进行环境分类
- **自适应调参**：根据环境类型动态调整SLAM参数
- **环境分析**：支持低光照、高纹理、快速运动等场景识别
- **性能监控**：实时监控SLAM性能和AI优化效果
- **ROS2原生**：完整的话题发布与参数管理

## 🎮 使用指南

### 基本机器人仿真
在默认环境启动机器人：
```bash
# 无界面模式（推荐用于远程服务器）
ros2 launch my_robot_description my_robot_sim.launch.py gui:=false

# 有界面模式（本地使用）
ros2 launch my_robot_description my_robot_sim.launch.py
```

**可选参数：**
```bash
ros2 launch my_robot_description my_robot_sim.launch.py \
    world:=/path/to/custom.world \
    use_sim_time:=true \
    gui:=false
```

### 六边形竞技场仿真

**推荐启动方式：**
```bash
cd /home/wb/ros_ws
source install/setup.bash
ros2 launch my_robot_simulation combined.launch.py
```

**高级配置：**
```bash
ros2 launch my_robot_simulation combined.launch.py \
    spawn_x:=2.0 \
    spawn_y:=1.0 \
    spawn_z:=0.3
```

> **注意（兼容性修正）**：
> 在 ROS2 humble 及以下版本，launch 文件中如需按条件启动 gzclient，必须这样写：
> 
> ```python
> from launch.conditions import IfCondition
> ...
> gzclient_cmd = ExecuteProcess(
>     cmd=['gzclient'],
>     output='screen',
>     emulate_tty=True,
>     condition=IfCondition(LaunchConfiguration('gui'))  # 只有 gui:=true 时才启动
> )
> ```
> 
> 不能直接用 `condition=LaunchConfiguration('gui')`，否则会报 'LaunchConfiguration' object has no attribute 'evaluate' 等兼容性错误。

### 机器人控制接口

**速度控制：**
```bash
# 前进（低速）
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.3}, angular: {z: 0.0}}' --once

# 后退
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: -0.3}, angular: {z: 0.0}}' --once

# 左转
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.5}}' --once

# 右转
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: -0.5}}' --once

# 急停
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.0}}' --once

# 连续移动（每秒1Hz）
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.5}, angular: {z: 0.0}}' -r 1
```

**键盘遥控（可选）：**
```bash
# 安装遥控包
sudo apt install ros-$ROS_DISTRO-turtlebot3-teleop
# 启动遥控
ros2 run turtlebot3_teleop teleop_keyboard
```

### 传感器数据监控

**激光雷达：**
```bash
# 查看激光数据
ros2 topic echo /scan --once
# 检查更新频率
ros2 topic hz /scan
# 查看激光参数
ros2 topic info /scan
# 统计激光数据
ros2 topic bw /scan
```

**里程计：**
```bash
# 查看里程计数据
ros2 topic echo /odom --once
# 监控位置变化
ros2 topic echo /odom | grep position
```

**TF树：**
```bash
# 生成TF树视图
ros2 run tf2_tools view_frames
# 实时监控TF变换
ros2 run tf2_ros tf2_echo odom base_link
# 查看TF信息
ros2 topic echo /tf --once
```

**系统状态监控：**
```bash
# 查看所有节点
ros2 node list
# 查看所有话题
ros2 topic list
# 查看所有服务
ros2 service list
# 查看计算图
rqt_graph
```

## 🎯 RViz 可视化使用指南

### 启动方式说明

**🔥 推荐：集成启动**
- 优势：一键启动，包含RViz可视化，配置统一
- 命令：`ros2 launch my_robot_navigation slam_navigation.launch.py mode:=slam rviz:=true`

**🔧 备选：分步启动**
- 优势：灵活控制，可自定义RViz配置
- 命令：先启动系统，再手动启动RViz

### 快速启动预配置视图

**基本机器人视图：**
```bash
rviz2 -d src/my_robot_description/rviz/robot_view.rviz
```

**SLAM建图视图：**
```bash
rviz2 -d src/my_robot_description/rviz/slam_view.rviz
```

**导航视图：**
```bash
rviz2 -d src/my_robot_description/rviz/navigation_view.rviz
```

**完整导航视图：**
```bash
rviz2 -d src/my_robot_description/rviz/full_navigation.rviz
```

### 配合系统启动

**与仿真一起启动：**
```bash
# 终端1: 启动仿真环境
ros2 launch my_robot_simulation combined.launch.py

# 终端2: 启动机器人视图
rviz2 -d src/my_robot_description/rviz/robot_view.rviz
```

**与SLAM一起启动（推荐）：**
```bash
# 终端1: 启动SLAM系统（集成RViz）
ros2 launch my_robot_navigation slam_navigation.launch.py mode:=slam rviz:=true
```

**与SLAM一起启动（分步式）：**
```bash
# 终端1: 启动SLAM系统
ros2 launch my_robot_navigation slam.launch.py

# 终端2: 启动SLAM视图
rviz2 -d src/my_robot_description/rviz/slam_view.rviz
```

**AI SLAM对比视图：**
```bash
# 终端1: 启动传统SLAM（集成RViz）
ros2 launch my_robot_navigation slam_navigation.launch.py mode:=slam rviz:=true

# 终端2: 启动AI SLAM
ros2 run orb_slam3_ai orb_slam3_ai_node
```

### RViz界面操作

#### 重要工具
- **🎯 2D Pose Estimate**: 设置机器人初始位姿（AMCL定位用）
- **🎯 2D Nav Goal**: 设置导航目标点
- **🔍 Select**: 选择3D视图中的物体
- **📷 Focus Camera**: 聚焦相机到选中物体
- **📏 Measure**: 测量两点间距离

#### 视角控制
- **鼠标左键**: 旋转视角
- **鼠标中键/滚轮**: 缩放
- **鼠标右键**: 平移视角

#### 常用显示项配置

**基本机器人显示：**
```
Add -> RobotModel     # 显示机器人3D模型
Add -> TF             # 显示坐标变换树
Add -> LaserScan      # 显示激光雷达扫描 (/scan)
Add -> Image          # 显示相机图像 (/camera/image_raw)
Add -> Odometry       # 显示里程计轨迹 (/odom)
```

**SLAM相关显示：**
```
Add -> Map            # 显示SLAM地图 (/map)
Add -> Path           # 显示轨迹路径
Add -> PointCloud2    # 显示AI SLAM地图点 (/orb_slam3/map_points)
Add -> PoseStamped    # 显示AI SLAM位姿 (/orb_slam3/pose)
```

**导航相关显示：**
```
Add -> Map            # 全局代价地图
Add -> Costmap        # 局部代价地图  
Add -> Path           # 全局规划路径 (/plan)
Add -> Path           # 局部规划路径 (/local_plan)
Add -> Polygon        # 机器人足迹
```

### 实用操作流程

#### SLAM建图操作
```bash
1. 启动SLAM系统和RViz
2. 在RViz中观察：
   - 机器人模型（蓝色/白色）
   - 激光扫描（红色点）
   - 构建中的地图（灰/黑色）
   - 机器人轨迹（绿色线）
3. 使用键盘控制机器人移动建图
4. File -> Save Config保存RViz配置
```

#### 导航操作
```bash
1. 启动导航系统和导航视图
2. 使用"2D Pose Estimate"设置初始位姿：
   - 点击工具
   - 在地图上点击机器人实际位置
   - 拖动设置朝向
3. 使用"2D Nav Goal"设置目标：
   - 点击工具
   - 在地图上点击目标位置
   - 拖动设置目标朝向
4. 观察路径规划和机器人自主导航
```

#### AI SLAM性能对比
```bash
1. 同时启动传统SLAM和AI SLAM
2. 在RViz中添加两套显示：
   - 传统SLAM: /map (绿色)
   - AI SLAM轨迹: /orb_slam3/path (红色)  
   - AI SLAM地图点: /orb_slam3/map_points
3. 观察两种SLAM的实时性能差异
4. 监控话题: /orb_slam3/performance
```

### 故障排除

**RViz启动问题：**
```bash
# 确保显示环境
export DISPLAY=:0

# 检查话题数据
ros2 topic list
ros2 topic echo /scan --once
```

**TF变换错误：**
```bash
# 检查TF树
ros2 run tf2_tools view_frames

# 检查特定变换
ros2 run tf2_ros tf2_echo map base_link

# 在RViz中设置正确的Fixed Frame (通常是"map"或"odom")
```

**性能优化：**
```bash
# 降低话题频率以提升性能
/scan: 5Hz → 2Hz  
/camera/image_raw: 30Hz → 10Hz

# 减少点云显示点数
PointCloud2 -> Size(Pixels): 3 → 1

# 关闭不需要的显示项
```

### 自定义配置保存
```bash
1. 在RViz中调整好所有显示项和视角
2. File -> Save Config As...
3. 保存到: src/my_robot_description/rviz/my_custom.rviz  
4. 下次使用: rviz2 -d src/my_robot_description/rviz/my_custom.rviz
```

## 🤖 ORB-SLAM3 AI 系统

### 系统概述
基于ORB-SLAM3的AI增强型SLAM系统，集成ResNet-18预训练模型进行环境分类，实现自适应参数优化的视觉SLAM。

### 核心特性
- **视觉SLAM**：基于ORB-SLAM3的RGB-D视觉同步定位与建图
- **AI优化**：使用预训练神经网络进行环境分析和参数优化
- **环境分类**：自动识别低光照、高纹理、快速运动等场景
- **自适应参数**：根据环境动态调整ORB特征点数量、尺度因子等
- **性能监控**：实时监控SLAM性能和轨迹质量

### 安装依赖
```bash
# 进入工作空间
cd /home/wb/ros_ws

# 安装PyTorch和相关依赖
pip3 install torch torchvision
pip3 install opencv-python
pip3 install numpy scipy

# 安装ROS2图像桥接
sudo apt update
sudo apt install ros-humble-cv-bridge python3-cv-bridge
```

### 构建ORB-SLAM3 AI包
```bash
# 构建包
colcon build --packages-select orb_slam3_ai

# 设置环境
source install/setup.bash
```

### 启动ORB-SLAM3 AI系统

**完整启动流程（推荐）：**

**方法1 - 集成启动传统SLAM+RViz（推荐）：**

**终端1 - 启动传统SLAM系统（包含RViz可视化）：**
```bash
cd /home/wb/ros_ws
source install/setup.bash
# 启动机器人仿真环境 + 传统slam_toolbox SLAM + RViz可视化
ros2 launch my_robot_navigation slam_navigation.launch.py mode:=slam rviz:=true
```

**终端2 - 启动AI增强ORB-SLAM3系统：**
```bash
cd /home/wb/ros_ws
source install/setup.bash
# 仅启动ORB-SLAM3 AI节点（避免重复启动仿真环境）
ros2 run orb_slam3_ai orb_slam3_ai_node
```

**方法1备选 - 分步启动：**
```bash
# 终端1: 启动SLAM系统
ros2 launch my_robot_navigation slam.launch.py

# 终端2: 手动启动RViz可视化
rviz2 -d src/my_robot_description/rviz/slam_view.rviz
```

**方法2 - 一键启动（会启动重复的仿真环境）：**

**仅启动AI增强ORB-SLAM3（包含完整仿真环境）：**
```bash
cd /home/wb/ros_ws
source install/setup.bash
# 启动机器人仿真环境 + AI增强ORB-SLAM3系统
ros2 launch orb_slam3_ai orb_slam3_ai.launch.py
```

> **注意**: 方法2会启动完整的仿真环境和AI SLAM，如果之前已启动传统SLAM，会产生两个Gazebo窗口。推荐使用方法1进行对比测试。

### 系统验证
```bash
# 检查SLAM话题
ros2 topic list | grep orb_slam3

# 查看位姿发布
ros2 topic echo /orb_slam3/pose --once

# 检查环境分类
ros2 topic echo /orb_slam3/environment_type --once

# 监控性能指标
ros2 topic echo /orb_slam3/performance --once

# 查看所有节点
ros2 node list | grep orb_slam3
```

### 预期结果
- **Gazebo界面**：显示带RGB-D相机的机器人
- **RViz界面**：显示SLAM轨迹、地图点和相机图像
- **AI优化**：实时环境分类和参数调整
- **终端输出**：显示模型加载成功和优化过程

### 故障排除
```bash
# 检查预训练模型下载
ls ~/.cache/torch/hub/checkpoints/

# 测试AI模型加载
python3 -c "import torch; import torchvision; print('PyTorch loaded successfully')"

# 检查相机话题
ros2 topic hz /camera/image_raw /camera/depth/image_raw

# 重启系统组件
ros2 node kill /orb_slam3_ai
```

### ORB-SLAM3 AI模型详细信息

#### 🤖 使用的AI模型

**预训练特征提取器:**
- **模型**: ResNet-18 (ImageNet预训练)
- **位置**: `~/.cache/torch/hub/checkpoints/resnet18-f37072fd.pth`
- **大小**: ~44.7MB
- **用途**: 图像特征提取，输出512维特征向量
- **来源**: PyTorch官方模型库，首次运行自动下载

**自定义参数映射网络:**
- **结构**: 全连接网络 (520 → 256 → 128 → 8)
- **输入**: ResNet特征(512) + 环境特征(8)
- **输出**: 8个SLAM参数的调整值
- **位置**: 内嵌在 `orb_slam3_ai/slam_ai_optimizer.py`

#### 📊 训练方法

**1. 数据收集**
```bash
# 创建训练数据目录
mkdir -p /home/wb/ros_ws/slam_training_data/{images,depth,ground_truth,slam_results}

# 数据收集流程
cd /home/wb/ros_ws
source install/setup.bash

# 运行数据收集脚本
python3 src/orb_slam3_ai/scripts/train_ai_model.py
```

**2. 训练数据格式**
```json
{
  "image_path": "images/scene_001.jpg",
  "depth_path": "depth/scene_001.png", 
  "env_features": [brightness, contrast, texture_density, motion_magnitude, angular_velocity, edge_density, motion_texture, stability],
  "optimal_params": [nFeatures, scaleFactor, nLevels, thRefRatio, KeyFrameLinearity, KeyFrameAngularity, minScore, nSimilarityGroups],
  "performance_score": 0.95,
  "scene_type": "indoor_low_light",
  "motion_type": "slow_translation"
}
```

**3. 模型训练**
```bash
# 准备训练环境
cd /home/wb/ros_ws
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

# 运行训练脚本
python3 src/orb_slam3_ai/scripts/train_ai_model.py

# 训练配置
# - 数据集大小: 10,000+ 样本推荐
# - 训练/验证比例: 8:2
# - 优化器: AdamW (lr=1e-4)
# - 损失函数: MSE Loss
# - 训练轮数: 100 epochs (早停机制)
```

**4. 启发式参数配置**
系统使用专家知识定义的启发式规则作为基础参数：

| 环境类型 | nFeatures | scaleFactor | nLevels | thRefRatio |
|---------|-----------|-------------|---------|------------|
| 低纹理 | 600 | 1.1 | 6 | 0.85 |
| 高纹理 | 1200 | 1.25 | 8 | 0.7 |
| 低光照 | 500 | 1.15 | 6 | 0.8 |
| 快速运动 | 1000 | 1.2 | 7 | 0.75 |
| 正常环境 | 1000 | 1.2 | 8 | 0.75 |

**5. 模型部署**
```bash
# 将训练好的模型复制到部署位置
cp /home/wb/ros_ws/trained_models/best_model.pth /home/wb/ros_ws/src/orb_slam3_ai/models/

# 修改配置文件使用自定义模型
# 编辑 orb_slam3_ai/slam_ai_optimizer.py
# 将 models.resnet18(pretrained=True) 替换为加载自定义模型
```

**6. 性能评估**
```bash
# 运行性能测试
ros2 run orb_slam3_ai performance_test

# 检查AI优化效果
ros2 topic echo /orb_slam3/performance --once
```

## 🚀 ORB-SLAM3 AI 完整使用指南

### ⚡ 快速开始 (1分钟体验)

如果你想快速体验AI增强SLAM系统，可以选择以下任一方式：

**方式1 - 直接启动AI SLAM（一键启动）：**
```bash
# 启动AI增强SLAM系统（包含仿真环境）
cd /home/wb/ros_ws
source install/setup.bash
ros2 launch orb_slam3_ai orb_slam3_ai.launch.py
```

**方式2 - 对比传统SLAM与AI SLAM：**
```bash
# 1. 启动传统SLAM系统
cd /home/wb/ros_ws
source install/setup.bash
ros2 launch my_robot_navigation slam.launch.py &

# 2. 新终端启动AI SLAM节点对比
sleep 5
ros2 run orb_slam3_ai orb_slam3_ai_node &

# 3. 控制机器人移动观察两种SLAM效果对比
sleep 3
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}' --once

# 4. 查看AI优化的SLAM参数
ros2 topic echo /orb_slam3/performance --once
```

系统会自动加载预训练的AI模型，实时分析环境并优化SLAM参数。

### 训练数据收集与模型训练

**1. 仿真环境数据收集**
```bash
# 启动仿真环境
cd /home/wb/ros_ws
source install/setup.bash
ros2 launch my_robot_simulation combined.launch.py

# 新终端：运行数据收集脚本
cd /home/wb/ros_ws
source install/setup.bash
python3 src/orb_slam3_ai/scripts/generate_training_data.py

# 数据将保存到 slam_training_data/simulation_data/
# - images/: RGB图像
# - depth/: 深度图像  
# - training_data.json: 标注数据
```

**2. 开源数据集下载**
```bash
# 下载TUM RGB-D数据集 (快速模式)
python3 src/orb_slam3_ai/scripts/download_datasets.py --quick

# 下载完整数据集 (较大，需要时间)
python3 src/orb_slam3_ai/scripts/download_datasets.py

# 仅下载TUM数据集
python3 src/orb_slam3_ai/scripts/download_datasets.py --tum-only

# 仅下载EuRoC数据集  
python3 src/orb_slam3_ai/scripts/download_datasets.py --euroc-only
```

**3. AI模型训练**
```bash
# 快速训练 (适用于原型测试)
python3 src/orb_slam3_ai/scripts/quick_train.py

# 改进版训练 (推荐)
python3 src/orb_slam3_ai/scripts/improved_train.py

# 完整训练管道
python3 src/orb_slam3_ai/scripts/train_ai_model.py
```

### 训练模型性能指标

| 指标 | 快速模型 | 改进模型 | 目标性能 |
|------|----------|----------|----------|
| 训练损失 | 119566 | 0.308 | <0.5 |
| 预测精度 | N/A | 2.28%-11.29% | <5% |
| 训练时间 | 2分钟 | 5分钟 | <30分钟 |
| 模型大小 | ~1MB | ~2MB | <10MB |
| 推理时间 | <20ms | <50ms | <100ms |

### AI模型架构详情

**改进版模型 (ImprovedSLAMNetwork):**
```
输入: RGB图像(64x64) + 环境特征(8维)

CNN特征提取器:
├── Conv2d(3→32) + BatchNorm + ReLU + MaxPool  
├── Conv2d(32→64) + BatchNorm + ReLU + MaxPool
└── Conv2d(64→128) + BatchNorm + ReLU + AdaptiveAvgPool
    └── Flatten + Linear(2048→256) + ReLU + Dropout(0.3)

参数预测网络:
├── Linear(264→128) + ReLU + BatchNorm + Dropout(0.2)
├── Linear(128→64) + ReLU + BatchNorm + Dropout(0.1)  
└── Linear(64→8) [输出8个SLAM参数]

损失函数: SmoothL1Loss (Huber Loss)
优化器: AdamW (lr=0.001, weight_decay=1e-4)
数据归一化: 基于训练数据统计的Z-score标准化
```

### 运行AI增强SLAM系统

**1. 启动完整系统（推荐方式）**

**选项A - 一键启动AI SLAM：**
```bash
# 启动AI增强SLAM系统（包含仿真环境）
cd /home/wb/ros_ws
source install/setup.bash
ros2 launch orb_slam3_ai orb_slam3_ai.launch.py
```

**选项B - 分步启动进行对比：**
```bash
# 终端1: 启动传统SLAM系统（slam_toolbox）
cd /home/wb/ros_ws
source install/setup.bash
ros2 launch my_robot_navigation slam.launch.py

# 终端2: 启动AI增强ORB-SLAM3节点
cd /home/wb/ros_ws  
source install/setup.bash
ros2 run orb_slam3_ai orb_slam3_ai_node

# 终端3: 启动RViz可视化（可选）
rviz2 -d src/my_robot_description/rviz/slam_view.rviz

# 终端4: 机器人控制（可选）
ros2 run turtlebot3_teleop teleop_keyboard
```

**2. 监控AI优化效果**
```bash
# 查看实时性能指标
ros2 topic echo /orb_slam3/performance

# 查看环境分类结果
ros2 topic echo /orb_slam3/environment_type

# 查看SLAM轨迹
ros2 topic echo /orb_slam3/pose

# 查看地图点云
ros2 topic echo /orb_slam3/map_points
```

**3. 测试不同环境场景**
```bash
# 移动机器人到不同区域测试AI适应性
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}' --once

# 观察参数优化变化  
ros2 param list /orb_slam3_ai_node
```

### 模型文件管理

**重要文件位置:**
```
src/orb_slam3_ai/models/
├── trained/
│   ├── best_slam_model.pth      # 最佳训练模型
│   ├── quick_slam_model.pth     # 快速训练模型  
│   └── slam_param_model_v*.pth  # 版本化模型
├── pretrained/
│   └── resnet18_features.pth    # ResNet-18特征提取器
└── configs/
    ├── model_config_v1.json     # 模型配置v1
    └── model_config_v2.json     # 模型配置v2

slam_training_data/
├── training_data.json           # 合并训练数据
├── simulation_data/             # 仿真数据
│   ├── images/                  # RGB图像
│   ├── depth/                   # 深度图像
│   └── training_data.json       # 仿真标注
└── raw_datasets/                # 原始开源数据集
    ├── TUM_RGB-D/
    └── EuRoC_MAV/
```

**模型版本管理:**
```bash
# 查看当前使用的模型
ls -la src/orb_slam3_ai/models/trained/

# 备份当前最佳模型
cp src/orb_slam3_ai/models/trained/best_slam_model.pth \
   src/orb_slam3_ai/models/trained/backup_$(date +%Y%m%d_%H%M%S).pth

# 切换模型版本 (修改slam_ai_optimizer.py中的model_path)
```

### 性能调优与故障排除

**1. 模型性能优化**
```bash
# 如果训练损失过高，尝试：
# - 增加训练数据量 (目标1000+样本)
# - 调整学习率 (在improved_train.py中修改)
# - 增加训练轮数
# - 使用数据增强

# 如果推理速度慢，尝试：
# - 减小输入图像尺寸 (当前64x64)
# - 使用模型量化
# - 切换到GPU推理
```

**2. 系统故障排除**
```bash
# 检查节点状态
ros2 node list | grep orb_slam3

# 检查话题连接
ros2 topic info /orb_slam3/pose

# 查看节点日志
ros2 node logs orb_slam3_ai_node

# 重新构建包
colcon build --packages-select orb_slam3_ai --cmake-clean-cache
```

**3. AI模型回退机制**
```bash
# 如果训练模型加载失败，系统会自动切换到启发式算法
# 检查回退状态：
ros2 service call /orb_slam3_ai_node/get_model_status \
  std_srvs/srv/Empty

# 强制使用启发式模式 (删除模型文件)：
mv src/orb_slam3_ai/models/trained/best_slam_model.pth \
   src/orb_slam3_ai/models/trained/best_slam_model.pth.bak
```

### 数据集扩展与自定义训练

**1. 添加自定义场景数据**
```bash
# 在不同环境中收集数据
python3 src/orb_slam3_ai/scripts/generate_training_data.py

# 修改运动模式 (编辑generate_training_data.py)
# - 增加复杂运动轨迹
# - 添加不同光照条件
# - 包含更多纹理类型
```

**2. 模型架构实验**
```bash
# 尝试不同网络架构 (修改improved_train.py):
# - 更深的CNN网络
# - 注意力机制
# - 残差连接
# - Transformer编码器

# 超参数调优:
# - 学习率调度
# - 批次大小
# - 正则化强度
# - 损失函数权重
```

### 停止系统
```bash
# 停止所有相关节点
ros2 node kill /orb_slam3_ai_node
pkill gzserver
pkill gzclient

# 或使用Ctrl+C停止各个终端
```

## 🍎 苹果检测系统

### 系统概述
基于YOLOv8的苹果检测系统，集成OpenCV和ROS2，支持实时图像处理和目标检测。

### 安装依赖
```bash
# 进入工作空间
cd /home/wb/ros_ws

# 安装YOLOv8和相关依赖
pip3 install ultralytics
pip3 install opencv-python
pip3 install numpy
pip3 install torch torchvision

# 安装ROS2图像桥接
sudo apt update
sudo apt install ros-humble-cv-bridge python3-cv-bridge
```

### 构建苹果检测包
```bash
# 构建苹果检测包
colcon build --packages-select my_robot_vision

# 设置环境
source install/setup.bash
```

### 启动苹果检测系统

**完整启动流程（推荐）：**

**终端1 - 启动六边形竞技场仿真：**
```bash
cd /home/wb/ros_ws
source install/setup.bash
ros2 launch my_robot_simulation combined.launch.py gui:=true
```

**终端2 - 启动苹果检测节点：**
```bash
cd /home/wb/ros_ws
source install/setup.bash
ros2 run my_robot_vision apple_detector
```

**终端3 - 如果GUI没有自动启动，手动启动：**
```bash
gzclient
```

**终端4 - 启动RViz可视化（可选）：**
```bash
cd /home/wb/ros_ws
source install/setup.bash
rviz2 -d src/my_robot_description/rviz/robot_view.rviz
```

**终端5 - 控制机器人移动（可选）：**
```bash
cd /home/wb/ros_ws
source install/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```

### 无GUI模式启动（适用于远程服务器）
```bash
# 启动无GUI仿真
cd /home/wb/ros_ws
source install/setup.bash
ros2 launch my_robot_simulation combined.launch.py gui:=false

# 在另一个终端启动苹果检测
cd /home/wb/ros_ws
source install/setup.bash
ros2 run my_robot_vision apple_detector
```

### 机器人控制命令
```bash
# 前进
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}, angular: {z: 0.0}}' --once

# 左转
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.5}}' --once

# 右转
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: -0.5}}' --once

# 停止
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}' --once
```

### 系统验证
```bash
# 检查所有话题
ros2 topic list

# 检查相机话题
ros2 topic echo /camera/image_raw --once

# 检查激光雷达数据
ros2 topic echo /scan --once

# 查看所有节点
ros2 node list

# 验证模型文件
ls -la /home/wb/ros_ws/yolov8_apple_custom.pt
```

### 预期结果
- **Gazebo界面**：显示六边形竞技场和机器人
- **苹果检测窗口**：显示相机图像和YOLOv8检测结果
- **RViz界面**：显示机器人的激光雷达数据和TF树
- **终端输出**：显示模型加载成功和检测结果

### 故障排除
```bash
# 检查Gazebo进程
ps aux | grep gazebo

# 检查ROS2节点
ros2 node list

# 检查话题
ros2 topic list

# 测试模型加载
python3 -c "from ultralytics import YOLO; model = YOLO('/home/wb/ros_ws/yolov8_apple_custom.pt'); print('Model loaded successfully')"
```

### 停止系统
```bash
# 停止所有ROS2节点
ros2 node kill /apple_detector
ros2 node kill /robot_state_publisher

# 停止Gazebo
pkill gzserver
pkill gzclient

# 或者使用Ctrl+C停止各个终端
```

## 🗺️ SLAM建图

### 启动SLAM会话

**推荐方式（集成RViz）：**
在六边形竞技场启动SLAM建图：
```bash
ros2 launch my_robot_navigation slam_navigation.launch.py mode:=slam rviz:=true
```

**传统方式（分步启动）：**
```bash
ros2 launch my_robot_navigation slam.launch.py
```

**自定义配置：**
```bash
ros2 launch my_robot_navigation slam_navigation.launch.py mode:=slam rviz:=true \
    slam_params_file:=/path/to/custom_slam_config.yaml
```

### 建图时控制机器人
使用键盘遥控驾驶机器人建图：
```bash
# 新终端
ros2 run turtlebot3_teleop teleop_keyboard
```

**话题手动控制：**
```bash
# 缓慢前进
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.3}, angular: {z: 0.0}}'

# 原地旋转
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.5}}'
```

### 保存地图
SLAM过程中或结束后保存当前地图：
```bash
# 默认名称保存
python3 src/my_robot_navigation/scripts/save_map.py

# 自定义名称保存
python3 src/my_robot_navigation/scripts/save_map.py my_custom_map

# 带时间戳保存
python3 src/my_robot_navigation/scripts/save_map.py --timestamp arena_map
```

### SLAM可视化
使用专用SLAM RViz配置：
```bash
rviz2 -d src/my_robot_description/rviz/slam_view.rviz
```

## 🧭 自主导航

### 启动导航模式
使用已建地图启动自主导航：
```bash
ros2 launch my_robot_navigation navigation.launch.py
```

**自定义地图：**
```bash
ros2 launch my_robot_navigation navigation.launch.py \
    map:=/path/to/your/map.yaml
```

### 设置导航目标

**推荐：使用RViz**
1. 打开RViz导航视图：
   ```bash
   rviz2 -d src/my_robot_description/rviz/full_navigation.rviz
   ```
2. 使用"2D Nav Goal"工具设置目标
3. 点击并拖动设置目标位姿和朝向

**命令行方式：**
```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
    '{header: {frame_id: "map"}, 
      pose: {position: {x: 2.0, y: 1.0, z: 0.0}, 
             orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
```

### 设置初始位姿（AMCL）
如果机器人初始位置不确定：
```bash
# 推荐：RViz"2D Pose Estimate"工具
# 或命令行：
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    '{header: {frame_id: "map"}, 
      pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, 
                    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'
```

## 🔄 SLAM与导航一体

**一键启动竞技场+SLAM+导航+RViz 推荐命令：**
```bash
cd /home/wb/ros_ws
source install/setup.bash
ros2 launch my_robot_navigation slam_navigation.launch.py mode:=both rviz:=true
```

- 该命令会自动启动仿真竞技场、机器人、SLAM建图、导航和RViz可视化。
- 如只需SLAM或导航，可将mode参数改为slam或navigation。

## 🗂️ 地图管理

### 列出可用地图
```bash
python3 src/my_robot_navigation/scripts/map_manager.py list
```

### 复制地图
```bash
python3 src/my_robot_navigation/scripts/map_manager.py copy source_map target_map
```

### 设置默认地图
```bash
python3 src/my_robot_navigation/scripts/map_manager.py default my_best_map
```

### 删除地图
```bash
# 需确认
python3 src/my_robot_navigation/scripts/map_manager.py delete old_map

# 强制删除
python3 src/my_robot_navigation/scripts/map_manager.py delete old_map --force
```

## ⚙️ 配置管理

### 机器人参数
编辑 `src/my_robot_description/config/robot_params.yaml`：
```yaml
robot_description:
  dimensions:
    base_link_length: 0.3
    wheel_separation: 0.22
  sensors:
    laser:
      max_range: 5.0
      samples: 360
```

### 仿真参数  
编辑 `src/my_robot_simulation/config/simulation_params.yaml`：
```yaml
hex_arena:
  arena_size: 10.0
  wall_height: 2.0
spawn_config:
  default:
    x: 0.0
    y: 0.0
    z: 0.1
```

### SLAM配置
编辑 `src/my_robot_navigation/config/slam_toolbox_config.yaml`：
```yaml
slam_toolbox:
  ros__parameters:
    # SLAM模式与分辨率
    mode: mapping
    resolution: 0.05
    max_laser_range: 5.0
    
    # 闭环参数
    do_loop_closing: true
    loop_search_maximum_distance: 3.0
    
    # 性能调优
    minimum_travel_distance: 0.2
    minimum_travel_heading: 0.2
```

### 导航配置
编辑 `src/my_robot_navigation/config/nav2_params.yaml`：
```yaml
# 关键参数示例
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      max_vel_x: 0.8
      max_vel_theta: 1.0
      
local_costmap:
  local_costmap:
    ros__parameters:
      width: 3
      height: 3
      resolution: 0.05
      
global_costmap:
  global_costmap:
    ros__parameters:
      resolution: 0.05
      robot_radius: 0.15
```

## 🔧 开发指南

### 构建工作区
```bash
# 构建指定包
colcon build --packages-select my_robot_description my_robot_simulation

# 带调试符号构建
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# 清理构建
rm -rf build/ install/ && colcon build
```

### 修改机器人设计
1. **编辑URDF/Xacro**：`src/my_robot_description/urdf/my_robot.urdf.xacro`
2. **重新生成URDF**： 
   ```bash
   xacro src/my_robot_description/urdf/my_robot.urdf.xacro > \
         src/my_robot_description/urdf/model.urdf
   ```
3. **重建包**：`colcon build --packages-select my_robot_description`

### 添加自定义传感器
1. **在URDF中定义传感器**：添加新link和joint
2. **配置Gazebo插件**：添加传感器插件配置
3. **更新RViz配置**：添加新传感器可视化
4. **测试集成**：验证传感器数据发布

### 创建自定义世界
1. **设计世界文件**：在`worlds/`目录创建`.world`文件
2. **添加到CMakeLists.txt**：包含到安装配置
3. **创建启动文件**：为新环境添加启动器
4. **测试验证**：确保物理和光照正常

## 🧪 测试与验证

### 功能测试
```bash
# 测试机器人生成
ros2 launch my_robot_description my_robot_sim.launch.py

# 验证话题
ros2 topic list | grep -E "(cmd_vel|odom|scan)"

# 检查TF
ros2 run tf2_ros tf2_echo odom base_link
```

### 性能测试
```bash
# 监控系统资源
htop

# 检查仿真实时因子
gz stats

# 分析话题频率
ros2 topic hz /scan /odom
```

### 导航测试（可选）
```bash
# 启动导航栈
ros2 launch nav2_bringup tb3_simulation_launch.py \
    use_simulator:=false \
    map:=/path/to/map.yaml

# 测试导航命令
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
    '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 1.0}}}'
```

## 🐛 故障排查

### 常见问题

**🔴 Gazebo 启动失败**
```bash
# 检查Gazebo插件
echo $GAZEBO_PLUGIN_PATH

# 验证Gazebo安装
gazebo --version

# 重置Gazebo配置
rm -rf ~/.gazebo/
```

**🔴 机器人生成问题**
```bash
# 验证URDF语法
check_urdf src/my_robot_description/urdf/model.urdf

# 测试URDF加载
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat src/my_robot_description/urdf/model.urdf)"
```

**🔴 传感器数据问题**
```bash
# 检查激光插件加载
ros2 topic list | grep scan

# 验证激光配置
ros2 topic echo /scan --once
```

**🔴 性能问题**
- 降低世界文件物理更新率
- 降低传感器更新频率
- 关闭不必要的可视化
- 使用无界面模式：`gui:=false`

**🔴 SLAM 问题**
```bash
# SLAM建图异常
# 检查激光数据
ros2 topic echo /scan --once

# 验证SLAM节点
ros2 node list | grep slam

# 检查TF树
ros2 run tf2_tools view_frames

# 重启SLAM
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
```

**🔴 导航问题**
```bash
# 机器人无法到达目标
# 检查代价地图
ros2 topic echo /global_costmap/costmap --once
ros2 topic echo /local_costmap/costmap --once

# 验证导航节点
ros2 node list | grep nav

# 检查目标话题
ros2 topic echo /goal_pose --once

# 导航卡死重置
ros2 service call /bt_navigator/clear_entirely_global_costmap std_srvs/srv/Empty
```

**🔴 定位问题**
```bash
# AMCL定位异常
# 手动设置初始位姿
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    '{header: {frame_id: "map"}}'

# 检查粒子云
ros2 topic echo /particle_cloud --once

# 增加粒子数
# max_particles: 5000
```

**🔴 地图加载问题**
```bash
# 地图加载异常
# 检查地图文件
ls src/my_robot_navigation/maps/

# 验证地图格式
head src/my_robot_navigation/maps/my_robot_map.yaml

# 手动测试地图服务器
ros2 run nav2_map_server map_server \
    --ros-args -p yaml_filename:=path/to/map.yaml
```

### 构建问题
```bash
# 清理工作区
rm -rf build/ install/ log/

# 安装缺失依赖
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 检查包依赖
ros2 pkg xml my_robot_description
```

## 🚀 高级特性

### SLAM集成
```bash
# 安装SLAM Toolbox
sudo apt install ros-${ROS_DISTRO}-slam-toolbox

# 启动SLAM
ros2 launch slam_toolbox online_async_launch.py
```

### 导航集成
```bash
# 安装Navigation2
sudo apt install ros-${ROS_DISTRO}-navigation2

# 启动导航栈
ros2 launch nav2_bringup navigation_launch.py
```

### 多机器人仿真
在启动文件中配置命名空间以支持多机器人。

## 📊 性能指标与优化

### 优化后的性能参数
| 组件 | 更新率 | 资源占用 | 优化说明 |
|------|--------|----------|----------|
| 物理引擎 | 1000 Hz | CPU: ~15% | 保持默认 |
| 差速驱动 | 30 Hz | CPU: ~1.5% | 降低从50→30Hz |
| 激光雷达 | 5 Hz | CPU: ~2.5% | 降低从10→5Hz，180采样 |
| RGB-D相机 | 30 Hz | CPU: ~3% | 高分辨率640×480 |
| 状态发布 | 30 Hz | CPU: ~1% | 保持默认 |
| **传统SLAM** |  |  |  |
| SLAM Toolbox | 可变 | CPU: ~10-20% | 优化参数配置 |
| 地图更新 | 5 Hz | 内存: ~50MB | 保持默认 |
| **视觉SLAM（ORB-SLAM3）** |  |  |  |
| 特征提取 | 30 Hz | CPU: ~15-25% | ORB特征检测 |
| AI参数优化 | 0.5 Hz | CPU: ~5-10% | ResNet-18推理 |
| 轨迹发布 | 30 Hz | 内存: ~100MB | 包含地图点云 |
| **导航组件** |  |  |  |
| 全局规划 | 10 Hz | CPU: ~2% | 降低从20→10Hz |
| 局部规划 | 10 Hz | CPU: ~4% | 降低从20→10Hz |
| AMCL定位 | 2 Hz | CPU: ~3% | 粒子数量1500→300 |
| 代价地图更新 | 3 Hz | CPU: ~2.5% | 降低从5→3Hz |

### 性能优化建议
```bash
# 无界面模式节省资源
ros2 launch my_robot_description my_robot_sim.launch.py gui:=false

# 监控系统资源使用
htop
watch -n 1 "ros2 topic hz /scan /odom"

# 检查Gazebo实时因子
gz stats

# 减少日志输出
export RCUTILS_LOGGING_SEVERITY=WARN
```

## 📚 参考资源

### ROS 2与仿真
- [ROS 2官方文档](https://docs.ros.org/en/humble/)
- [Gazebo教程](http://gazebosim.org/tutorials)
- [URDF教程](http://wiki.ros.org/urdf/Tutorials)

### SLAM与导航
- [Navigation2官方文档](https://navigation.ros.org/)
- [SLAM Toolbox文档](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2教程](https://navigation.ros.org/tutorials/)
- [AMCL文档](https://navigation.ros.org/configuration/packages/configuring-amcl.html)

### 配置与调优
- [Nav2配置指南](https://navigation.ros.org/configuration/index.html)
- [DWB控制器调优](https://navigation.ros.org/configuration/packages/dwb-params/index.html)
- [代价地图配置](https://navigation.ros.org/configuration/packages/costmap-plugins/index.html)

## 🤝 贡献指南

1. Fork本仓库
2. 创建功能分支：`git checkout -b feature/amazing-feature`
3. 提交更改：`git commit -m 'Add amazing feature'`
4. 推送分支：`git push origin feature/amazing-feature`
5. 创建Pull Request

### 代码规范
- 遵循ROS 2风格指南
- 补充完整文档
- 新功能需包含单元测试
- 保持向后兼容

## 📄 许可证

本项目基于 Apache 2.0 许可证，详见 [LICENSE](LICENSE) 文件。

## 👨‍💻 维护者

**wb** - 1878087979@qq.com

如需支持、提问或贡献，请创建issue或联系维护者。

## 🚀 快速参考

### 模型位置速查

| 组件 | 模型文件 | 位置 | 大小 |
|------|----------|------|------|
| **ORB-SLAM3 特征提取** | resnet18-f37072fd.pth | `~/.cache/torch/hub/checkpoints/` | 44.7MB |
| **SLAM参数映射** | 内嵌网络 | `orb_slam3_ai/slam_ai_optimizer.py` | <1MB |
| **苹果检测** | yolov8_apple_custom.pt | `/home/wb/ros_ws/` | 自定义 |
| **自定义训练模型** | best_model.pth | `src/orb_slam3_ai/models/trained/` | 变化 |

### 训练命令速查

```bash
# ORB-SLAM3 AI模型训练
cd /home/wb/ros_ws
python3 src/orb_slam3_ai/scripts/train_ai_model.py

# 数据收集
mkdir -p slam_training_data/{images,depth,ground_truth,slam_results}

# 模型部署
cp trained_models/best_model.pth src/orb_slam3_ai/models/trained/
```

### RViz命令速查

```bash
# 快速启动预配置视图
rviz2 -d src/my_robot_description/rviz/robot_view.rviz      # 机器人视图
rviz2 -d src/my_robot_description/rviz/slam_view.rviz       # SLAM建图视图
rviz2 -d src/my_robot_description/rviz/navigation_view.rviz # 导航视图
rviz2 -d src/my_robot_description/rviz/full_navigation.rviz # 完整导航视图

# 常用操作
2D Pose Estimate  # 设置机器人初始位姿 (AMCL)
2D Nav Goal       # 设置导航目标点
Add -> RobotModel # 显示机器人3D模型
Add -> LaserScan  # 显示激光雷达数据 (/scan)
Add -> Map        # 显示SLAM地图 (/map)
Add -> Path       # 显示轨迹路径
```

### 完整系统启动速查

| 功能 | 启动命令 | RViz配置 |
|------|----------|----------|
| **基础仿真** | `ros2 launch my_robot_simulation combined.launch.py` | `robot_view.rviz` |
| **传统SLAM（推荐）** | `ros2 launch my_robot_navigation slam_navigation.launch.py mode:=slam rviz:=true` | 集成RViz |
| **传统SLAM（分步）** | `ros2 launch my_robot_navigation slam.launch.py` | `slam_view.rviz` |
| **AI增强SLAM** | `ros2 launch orb_slam3_ai orb_slam3_ai.launch.py` | `slam_view.rviz` |
| **自主导航** | `ros2 launch my_robot_navigation navigation.launch.py` | `full_navigation.rviz` |
| **苹果检测** | `ros2 run my_robot_vision apple_detector` | `robot_view.rviz` |

### 性能指标速查

| 系统组件 | 处理频率 | 延迟 | 资源占用 |
|----------|----------|------|----------|
| **RGB-D相机** | 30Hz | <33ms | CPU 3% |
| **激光雷达** | 5Hz | <200ms | CPU 2.5% |
| **ORB-SLAM3 AI** | 0.5Hz | <100ms | CPU 5-10% |
| **苹果检测** | 实时 | 60-80ms | CPU 15% |
| **总体系统** | - | - | CPU <30% |

---

<div align="center">

**基于 ROS 2 和 Gazebo 构建 ❤️**

*面向教育与科研的专业机器人仿真*

**🤖 ORB-SLAM3 AI增强 | 🍎 YOLOv8目标检测 | 📡 多传感器融合**

</div>