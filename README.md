# Enhanced ROS 2 Robot Simulation Workspace

A professional-grade ROS 2 workspace for differential drive robot simulation featuring advanced sensor integration, parametric configuration, and comprehensive visualization tools.

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

## 🚀 Overview

This project implements a complete robot simulation ecosystem using ROS 2 and Gazebo, featuring:

- **Parametric Robot Design**: Fully configurable differential drive robot with laser scanner
- **Multi-Environment Support**: Default world and custom hexagonal arena environments  
- **Advanced Sensor Integration**: Enhanced laser scanner with noise modeling and realistic physics
- **SLAM Mapping**: Real-time simultaneous localization and mapping using slam_toolbox
- **Autonomous Navigation**: Complete Nav2 navigation stack with path planning and obstacle avoidance
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

### Navigation Dependencies (required for SLAM and autonomous navigation)
```bash
sudo apt install ros-${ROS_DISTRO}-navigation2 \
                 ros-${ROS_DISTRO}-nav2-bringup \
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

### 🧭 my_robot_navigation
**SLAM mapping and autonomous navigation package**

```
my_robot_navigation/
├── config/
│   ├── slam_toolbox_config.yaml # SLAM configuration
│   └── nav2_params.yaml         # Navigation parameters
├── launch/
│   ├── slam.launch.py           # SLAM mapping mode
│   ├── navigation.launch.py     # Autonomous navigation
│   └── slam_navigation.launch.py # Unified SLAM+Nav system
├── maps/                        # Saved map files
│   └── my_robot_map.yaml        # Default map
└── scripts/
    ├── save_map.py             # Map saving utility
    └── map_manager.py          # Map management tools
```

**Navigation Features:**
- **SLAM Mapping**: Real-time map building using slam_toolbox
- **Localization**: AMCL-based robot localization on known maps
- **Path Planning**: Global and local path planning with Nav2
- **Obstacle Avoidance**: Dynamic obstacle detection and avoidance
- **Map Management**: Tools for saving, loading, and managing maps
- **Multi-Mode Support**: SLAM, navigation, or combined operation

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

## 🗺️ SLAM Mapping

### Starting SLAM Session
Launch SLAM mapping in the hexagonal arena:
```bash
ros2 launch my_robot_navigation slam.launch.py
```

**With custom configuration:**
```bash
ros2 launch my_robot_navigation slam.launch.py \
    slam_params_file:=/path/to/custom_slam_config.yaml
```

### Controlling Robot During Mapping
Use keyboard teleop to drive the robot and build the map:
```bash
# In a new terminal
ros2 run turtlebot3_teleop teleop_keyboard
```

**Manual control via topics:**
```bash
# Drive forward slowly
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.3}, angular: {z: 0.0}}'

# Turn in place for better coverage
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.5}}'
```

### Saving Maps
Save the current map during or after SLAM:
```bash
# Save with default name
python3 src/my_robot_navigation/scripts/save_map.py

# Save with custom name
python3 src/my_robot_navigation/scripts/save_map.py my_custom_map

# Save with timestamp
python3 src/my_robot_navigation/scripts/save_map.py --timestamp arena_map
```

### SLAM Visualization
Use the dedicated SLAM RViz configuration:
```bash
rviz2 -d src/my_robot_description/rviz/slam_view.rviz
```

## 🧭 Autonomous Navigation

### Starting Navigation Mode
Launch autonomous navigation with a pre-built map:
```bash
ros2 launch my_robot_navigation navigation.launch.py
```

**With custom map:**
```bash
ros2 launch my_robot_navigation navigation.launch.py \
    map:=/path/to/your/map.yaml
```

### Setting Navigation Goals

**Using RViz (Recommended):**
1. Open RViz with navigation view:
   ```bash
   rviz2 -d src/my_robot_description/rviz/full_navigation.rviz
   ```
2. Use "2D Nav Goal" tool to set destination
3. Click and drag to set goal pose and orientation

**Using Command Line:**
```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
    '{header: {frame_id: "map"}, 
      pose: {position: {x: 2.0, y: 1.0, z: 0.0}, 
             orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
```

### Setting Initial Pose (for AMCL)
If the robot's initial position is uncertain:
```bash
# Using RViz "2D Pose Estimate" tool (recommended)
# Or via command line:
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    '{header: {frame_id: "map"}, 
      pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, 
                    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'
```

## 🔄 Combined SLAM and Navigation

### Unified System Launch
Launch both SLAM and navigation capabilities:
```bash
# SLAM mode only
ros2 launch my_robot_navigation slam_navigation.launch.py mode:=slam

# Navigation mode only  
ros2 launch my_robot_navigation slam_navigation.launch.py mode:=navigation

# Both SLAM and navigation (advanced)
ros2 launch my_robot_navigation slam_navigation.launch.py mode:=both
```

**Complete system with RViz:**
```bash
ros2 launch my_robot_navigation slam_navigation.launch.py \
    mode:=slam \
    rviz:=true
```

## 🗂️ Map Management

### List Available Maps
```bash
python3 src/my_robot_navigation/scripts/map_manager.py list
```

### Copy Maps
```bash
python3 src/my_robot_navigation/scripts/map_manager.py copy source_map target_map
```

### Set Default Map
```bash
python3 src/my_robot_navigation/scripts/map_manager.py default my_best_map
```

### Delete Maps
```bash
# With confirmation
python3 src/my_robot_navigation/scripts/map_manager.py delete old_map

# Force delete without confirmation
python3 src/my_robot_navigation/scripts/map_manager.py delete old_map --force
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

### SLAM Configuration
Edit `src/my_robot_navigation/config/slam_toolbox_config.yaml`:
```yaml
slam_toolbox:
  ros__parameters:
    # SLAM mode and resolution
    mode: mapping
    resolution: 0.05
    max_laser_range: 5.0
    
    # Loop closure parameters
    do_loop_closing: true
    loop_search_maximum_distance: 3.0
    
    # Performance tuning
    minimum_travel_distance: 0.2
    minimum_travel_heading: 0.2
```

### Navigation Configuration
Edit `src/my_robot_navigation/config/nav2_params.yaml`:
```yaml
# Example key parameters
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

**🔴 SLAM Issues**
```bash
# SLAM not building map properly
# Check laser scan data
ros2 topic echo /scan --once

# Verify SLAM node is running
ros2 node list | grep slam

# Check transform tree
ros2 run tf2_tools view_frames

# Restart SLAM if needed
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
```

**🔴 Navigation Issues**
```bash
# Robot not reaching goals
# Check costmaps
ros2 topic echo /global_costmap/costmap --once
ros2 topic echo /local_costmap/costmap --once

# Verify navigation nodes
ros2 node list | grep nav

# Check goal topic
ros2 topic echo /goal_pose --once

# Reset navigation if stuck
ros2 service call /bt_navigator/clear_entirely_global_costmap std_srvs/srv/Empty
```

**🔴 Localization Issues**
```bash
# AMCL not localizing properly
# Set initial pose manually
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    '{header: {frame_id: "map"}}'

# Check particle cloud
ros2 topic echo /particle_cloud --once

# Increase particle count in nav2_params.yaml:
# max_particles: 5000
```

**🔴 Map Loading Issues**
```bash
# Map not loading correctly
# Check map file exists
ls src/my_robot_navigation/maps/

# Verify map format
head src/my_robot_navigation/maps/my_robot_map.yaml

# Test map server manually
ros2 run nav2_map_server map_server \
    --ros-args -p yaml_filename:=path/to/map.yaml
```

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
| **SLAM Components** |  |  |
| SLAM Toolbox | Variable | CPU: ~10-20% |
| Map Updates | 5 Hz | Memory: ~50MB |
| **Navigation Components** |  |  |
| Global Planner | 1 Hz | CPU: ~3% |
| Local Planner | 20 Hz | CPU: ~8% |
| AMCL Localization | 2 Hz | CPU: ~5% |
| Costmap Updates | 5 Hz | CPU: ~4% |

## 📚 Additional Resources

### Core ROS 2 and Simulation
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)

### SLAM and Navigation
- [Navigation2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox Documentation](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 Tutorials](https://navigation.ros.org/tutorials/)
- [AMCL Documentation](https://navigation.ros.org/configuration/packages/configuring-amcl.html)

### Configuration and Tuning
- [Nav2 Configuration Guide](https://navigation.ros.org/configuration/index.html)
- [DWB Controller Tuning](https://navigation.ros.org/configuration/packages/dwb-params/index.html)
- [Costmap Configuration](https://navigation.ros.org/configuration/packages/costmap-plugins/index.html)

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