# 增强型 ROS 2 机器人仿真工作空间

一个专业级的 ROS 2 差速驱动机器人仿真工作区，具备高级传感器集成、参数化配置和全面可视化工具。

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

## 🚀 概述

本项目基于 ROS 2 和 Gazebo 实现了完整的机器人仿真生态系统，主要特性包括：

- **参数化机器人设计**：可完全配置的差速驱动机器人，带激光雷达
- **多环境支持**：默认世界和自定义六边形竞技场环境
- **高级传感器集成**：带噪声建模和真实物理的增强型激光雷达
- **SLAM 建图**：基于 slam_toolbox 的实时同步定位与建图
- **自主导航**：完整的 Nav2 导航栈，支持路径规划与避障
- **专业配置管理**：基于 YAML 的参数配置系统
- **全面可视化**：预配置的 RViz 开发与导航视图
- **模块化架构**：机器人描述与仿真环境分离

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

## 🛠️ 安装

### 配置rosdep镜像源（推荐）
为了在中国网络环境下更快地安装依赖，建议首先配置清华大学镜像：
```bash
# 设置环境变量
export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml
echo 'export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml' >> ~/.bashrc

# 运行配置脚本
source ~/.ros/rosdep.sh
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
- **传感器**：360°激光雷达，带高斯噪声
- **性能**：50Hz控制循环，10Hz传感器更新

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
启动完整竞技场环境：
```bash
ros2 launch my_robot_simulation combined.launch.py
```

**高级配置：**
```bash
ros2 launch my_robot_simulation combined.launch.py \
    spawn_x:=2.0 \
    spawn_y:=1.0 \
    spawn_z:=0.3
```

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

### 可视化选项

**基本机器人视图：**
```bash
rviz2 -d src/my_robot_description/rviz/robot_view.rviz
```

**导航视图：**
```bash
rviz2 -d src/my_robot_description/rviz/navigation_view.rviz
```

## 🗺️ SLAM建图

### 启动SLAM会话
在六边形竞技场启动SLAM建图：
```bash
ros2 launch my_robot_navigation slam.launch.py
```

**自定义配置：**
```bash
ros2 launch my_robot_navigation slam.launch.py \
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

### 一体化系统启动
同时启动SLAM和导航功能：
```bash
# 仅SLAM模式
ros2 launch my_robot_navigation slam_navigation.launch.py mode:=slam

# 仅导航模式  
ros2 launch my_robot_navigation slam_navigation.launch.py mode:=navigation

# SLAM+导航（高级）
ros2 launch my_robot_navigation slam_navigation.launch.py mode:=both
```

**带RViz的完整系统：**
```bash
ros2 launch my_robot_navigation slam_navigation.launch.py \
    mode:=slam \
    rviz:=true
```

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
| 状态发布 | 30 Hz | CPU: ~1% | 保持默认 |
| **SLAM组件** |  |  |  |
| SLAM Toolbox | 可变 | CPU: ~10-20% | 优化参数配置 |
| 地图更新 | 5 Hz | 内存: ~50MB | 保持默认 |
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

---

<div align="center">

**基于 ROS 2 和 Gazebo 构建 ❤️**

*面向教育与科研的专业机器人仿真*

</div>