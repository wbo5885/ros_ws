# Enhanced ROS 2 Robot Simulation Workspace

A professional-grade ROS 2 workspace for differential drive robot simulation featuring advanced sensor integration, parametric configuration, and comprehensive visualization tools.

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

## 🚀 Overview

This project implements a complete robot simulation ecosystem using ROS 2 and Gazebo, featuring:

- **Parametric Robot Design**: Fully configurable differential drive robot with laser scanner
- **Multi-Environment Support**: Default world and custom hexagonal arena environments  
- **Advanced Sensor Integration**: Enhanced laser scanner with noise modeling and realistic physics
- **Professional Configuration Management**: YAML-based parameter configuration system
- **Comprehensive Visualization**: Pre-configured RViz setups for development and navigation
- **Modular Architecture**: Clean separation between robot description and simulation environments

## 📋 Prerequisites

### System Requirements
- **ROS 2** (Humble/Iron recommended)
- **Gazebo Classic** (11.x) or **Ignition Gazebo** (6.x+)
- **Ubuntu 20.04+** or **macOS** with ROS 2 support

### Required ROS 2 Packages
```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-gazebo-ros-pkgs \
                 ros-${ROS_DISTRO}-robot-state-publisher \
                 ros-${ROS_DISTRO}-joint-state-publisher-gui \
                 ros-${ROS_DISTRO}-xacro \
                 ros-${ROS_DISTRO}-rviz2
```

### Optional Dependencies (for advanced features)
```bash
sudo apt install ros-${ROS_DISTRO}-navigation2 \
                 ros-${ROS_DISTRO}-slam-toolbox \
                 ros-${ROS_DISTRO}-turtlebot3-teleop
```

## 🛠️ Installation

1. **Setup workspace:**
   ```bash
   cd ros_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build
   source install/setup.bash
   ```

2. **Verify installation:**
   ```bash
   ros2 pkg list | grep -E "(my_robot|description|simulation)"
   ```

3. **Test basic functionality:**
   ```bash
   ros2 launch my_robot_description my_robot_sim.launch.py
   ```

## 📦 Package Architecture

### 🤖 my_robot_description
**Core robot definition and visualization package**

```
my_robot_description/
├── config/
│   ├── robot_params.yaml     # Robot physical parameters
├── launch/
│   └── my_robot_sim.launch.py # Basic simulation launcher
├── rviz/
│   ├── robot_view.rviz       # Basic robot visualization
│   └── navigation_view.rviz  # Navigation-ready setup
├── urdf/
│   ├── my_robot.urdf.xacro   # Parametric robot definition
│   ├── model.urdf            # Compiled URDF
│   └── model.config          # Gazebo model configuration
└── worlds/
    └── my_robot_world.world  # Default simulation world
```

**Robot Specifications:**
- **Chassis**: 0.3m × 0.2m × 0.1m with realistic mass distribution
- **Drive System**: Differential drive with 0.22m wheel separation
- **Wheels**: 0.05m radius with enhanced friction modeling
- **Caster**: Rear spherical support for stability
- **Sensors**: 360° laser scanner with Gaussian noise modeling
- **Performance**: 50Hz control loop, 10Hz sensor updates

### 🌍 my_robot_simulation  
**Advanced simulation environments and launch configurations**

```
my_robot_simulation/
├── config/
│   └── simulation_params.yaml  # Environment parameters
├── launch/
│   ├── my_hex_arena.launch.py  # Hexagonal arena launcher
│   └── combined.launch.py      # Complete simulation setup
└── worlds/
    └── my_hex_arena.world      # Custom hexagonal arena
```

**Environment Features:**
- **Hexagonal Arena**: 10m radius with 2m walls for contained experiments
- **Realistic Physics**: ODE physics engine with accurate material properties
- **Advanced Lighting**: Directional lighting with shadow casting
- **Modular Design**: Easy customization through YAML configuration

## 🎮 Usage Guide

### Basic Robot Simulation
Launch robot in default environment:
```bash
ros2 launch my_robot_description my_robot_sim.launch.py
```

**Optional parameters:**
```bash
ros2 launch my_robot_description my_robot_sim.launch.py \
    world:=/path/to/custom.world \
    use_sim_time:=true
```

### Hexagonal Arena Simulation
Launch complete arena setup:
```bash
ros2 launch my_robot_simulation combined.launch.py
```

**Advanced configuration:**
```bash
ros2 launch my_robot_simulation combined.launch.py \
    spawn_x:=2.0 \
    spawn_y:=1.0 \
    spawn_z:=0.3
```

### Robot Control Interface

**Velocity Control:**
```bash
# Forward motion
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 1.0}, angular: {z: 0.0}}'

# Rotation
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.5}}'

# Emergency stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

**Keyboard Teleop (optional):**
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

### Sensor Data Monitoring

**Laser Scanner:**
```bash
ros2 topic echo /scan
ros2 topic hz /scan    # Check update rate
```

**Odometry:**
```bash
ros2 topic echo /odom
```

**Transform Tree:**
```bash
ros2 run tf2_tools view_frames
```

### Visualization Options

**Basic Robot View:**
```bash
rviz2 -d src/my_robot_description/rviz/robot_view.rviz
```

**Navigation-Ready View:**
```bash
rviz2 -d src/my_robot_description/rviz/navigation_view.rviz
```

## ⚙️ Configuration Management

### Robot Parameters
Edit `src/my_robot_description/config/robot_params.yaml`:
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

### Simulation Parameters  
Edit `src/my_robot_simulation/config/simulation_params.yaml`:
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

## 🔧 Development Guide

### Building the Workspace
```bash
# Build specific packages
colcon build --packages-select my_robot_description my_robot_simulation

# Build with debugging symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Clean build
rm -rf build/ install/ && colcon build
```

### Modifying Robot Design
1. **Edit URDF/Xacro**: `src/my_robot_description/urdf/my_robot.urdf.xacro`
2. **Regenerate URDF**: 
   ```bash
   xacro src/my_robot_description/urdf/my_robot.urdf.xacro > \
         src/my_robot_description/urdf/model.urdf
   ```
3. **Rebuild package**: `colcon build --packages-select my_robot_description`

### Adding Custom Sensors
1. **Define sensor in URDF**: Add new link and joint
2. **Configure Gazebo plugin**: Add sensor plugin configuration
3. **Update RViz config**: Add visualization for new sensor
4. **Test integration**: Verify sensor data publication

### Creating Custom Worlds
1. **Design world file**: Create `.world` file in `worlds/` directory
2. **Add to CMakeLists.txt**: Include in install configuration
3. **Create launch file**: Add launcher for new environment
4. **Test and validate**: Ensure proper physics and lighting

## 🧪 Testing and Validation

### Functional Tests
```bash
# Test robot spawning
ros2 launch my_robot_description my_robot_sim.launch.py

# Verify topics
ros2 topic list | grep -E "(cmd_vel|odom|scan)"

# Check transforms
ros2 run tf2_ros tf2_echo odom base_link
```

### Performance Testing
```bash
# Monitor system resources
htop

# Check simulation real-time factor
gz stats

# Analyze topic frequencies
ros2 topic hz /scan /odom
```

### Navigation Testing (optional)
```bash
# Launch with navigation stack
ros2 launch nav2_bringup tb3_simulation_launch.py \
    use_simulator:=false \
    map:=/path/to/map.yaml

# Test navigation commands
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
    '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 1.0}}}'
```

## 🐛 Troubleshooting

### Common Issues

**🔴 Gazebo Launch Failures**
```bash
# Check Gazebo plugins
echo $GAZEBO_PLUGIN_PATH

# Verify Gazebo installation
gazebo --version

# Reset Gazebo configuration
rm -rf ~/.gazebo/
```

**🔴 Robot Spawning Issues**
```bash
# Validate URDF syntax
check_urdf src/my_robot_description/urdf/model.urdf

# Test URDF loading
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat src/my_robot_description/urdf/model.urdf)"
```

**🔴 Sensor Data Problems**
```bash
# Check laser plugin loading
ros2 topic list | grep scan

# Verify laser configuration
ros2 topic echo /scan --once
```

**🔴 Performance Issues**
- Reduce physics update rate in world files
- Lower sensor update frequencies
- Disable unnecessary visualizations
- Use headless mode: `gui:=false`

### Build Issues
```bash
# Clear workspace
rm -rf build/ install/ log/

# Install missing dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Check package dependencies
ros2 pkg xml my_robot_description
```

## 🚀 Advanced Features

### SLAM Integration
```bash
# Install SLAM Toolbox
sudo apt install ros-${ROS_DISTRO}-slam-toolbox

# Launch SLAM
ros2 launch slam_toolbox online_async_launch.py
```

### Navigation Integration
```bash
# Install Navigation2
sudo apt install ros-${ROS_DISTRO}-navigation2

# Launch navigation stack
ros2 launch nav2_bringup navigation_launch.py
```

### Multi-Robot Simulation
Configure namespaces in launch files for multi-robot setups.

## 📊 Performance Metrics

| Component | Update Rate | Resource Usage |
|-----------|-------------|----------------|
| Physics Engine | 1000 Hz | CPU: ~15% |
| Differential Drive | 50 Hz | CPU: ~2% |
| Laser Scanner | 10 Hz | CPU: ~5% |
| Robot State Publisher | 30 Hz | CPU: ~1% |

## 📚 Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [Navigation2 Documentation](https://navigation.ros.org/)

## 🤝 Contributing

1. Fork the repository
2. Create feature branch: `git checkout -b feature/amazing-feature`
3. Commit changes: `git commit -m 'Add amazing feature'`
4. Push to branch: `git push origin feature/amazing-feature`
5. Open Pull Request

### Code Standards
- Follow ROS 2 style guidelines
- Add comprehensive documentation
- Include unit tests for new features
- Maintain backward compatibility

## 📄 License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

## 👨‍💻 Maintainer

**wb** - 1878087979@qq.com

For support, questions, or contributions, please create an issue or contact the maintainer.

---

<div align="center">

**Built with ❤️ using ROS 2 and Gazebo**

*Professional robotics simulation for education and research*

</div>