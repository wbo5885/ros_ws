# ROS 2 Robot Simulation Workspace

A comprehensive ROS 2 workspace for robot simulation featuring a differential drive robot with laser sensing capabilities in Gazebo environments.

## Overview

This project implements a complete robot simulation system using ROS 2 and Gazebo. The workspace contains a differential drive robot with integrated laser scanning, custom simulation environments, and extensible Python frameworks for robot behavior development.

## Prerequisites

- **ROS 2** (Humble/Foxy or later)
- **Gazebo** (Classic or Ignition)
- **Python 3.8+**
- **Required ROS 2 packages:**
  - `gazebo_ros`
  - `robot_state_publisher`
  - `joint_state_publisher_gui`
  - `xacro`
  - `ros_gz_sim` (for Ignition Gazebo)
  - `ros_gz_bridge`

## Installation

1. **Clone and setup workspace:**
   ```bash
   cd ros_ws
   colcon build
   source install/setup.bash
   ```

2. **Verify installation:**
   ```bash
   ros2 pkg list | grep -E "(my_robot|simple_robot)"
   ```

## Package Structure

### 📦 my_robot_description
Robot model definition and visualization package.

**Key Components:**
- `urdf/my_robot.urdf.xacro` - Parametric robot model definition
- `urdf/model.urdf` - Compiled URDF file
- `launch/my_robot_sim.launch.py` - Basic simulation launcher
- `worlds/my_robot_world.world` - Default simulation environment

**Robot Specifications:**
- **Base:** 0.3m × 0.2m × 0.1m rectangular chassis
- **Wheels:** Two 0.05m radius drive wheels with 0.22m separation
- **Caster:** Rear spherical caster wheel (0.02m radius)
- **Sensors:** 360° laser scanner (5m range, 360 samples)
- **Drive:** Differential drive with odometry

### 📦 my_robot_simulation
Simulation environments and advanced launch configurations.

**Key Components:**
- `launch/combined.launch.py` - Integrated arena + robot launcher
- `launch/my_hex_arena.launch.py` - Hexagonal arena environment
- `worlds/my_hex_arena.world` - Custom hexagonal arena with walls

**Arena Features:**
- Hexagonal boundary walls
- Realistic lighting and physics
- Ground plane with friction modeling

### 📦 simple_robot
Python package framework for robot behavior development.

**Structure:**
- Standard ROS 2 Python package layout
- Test framework integration
- Ready for custom node development

## Usage

### Basic Robot Simulation
Launch the robot in the default world:
```bash
ros2 launch my_robot_description my_robot_sim.launch.py
```

### Hexagonal Arena Simulation
Launch the robot in the custom hexagonal arena:
```bash
ros2 launch my_robot_simulation combined.launch.py
```

### Robot Control
Control the robot using velocity commands:
```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Rotate
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Sensor Data
Monitor laser scan data:
```bash
ros2 topic echo /scan
```

Monitor odometry:
```bash
ros2 topic echo /odom
```

### Visualization
Launch RViz for robot state visualization:
```bash
ros2 run rviz2 rviz2
```

## Development

### Building the Workspace
```bash
colcon build --packages-select my_robot_description my_robot_simulation simple_robot
source install/setup.bash
```

### Modifying the Robot Model
1. Edit `src/my_robot_description/urdf/my_robot.urdf.xacro`
2. Rebuild the package:
   ```bash
   colcon build --packages-select my_robot_description
   ```

### Creating Custom Worlds
1. Add new `.world` files to `src/my_robot_simulation/worlds/`
2. Create corresponding launch files in `src/my_robot_simulation/launch/`
3. Update `CMakeLists.txt` to install new files

### Adding Robot Behaviors
Implement custom nodes in the `simple_robot` package:
```bash
cd src/simple_robot/simple_robot/
# Add your Python node files here
```

## Technical Details

### Robot Kinematic Model
- **Type:** Differential drive
- **Wheel base:** 0.22m
- **Wheel radius:** 0.05m
- **Maximum speed:** Configurable via cmd_vel topics

### Sensor Configuration
- **Laser Scanner:**
  - Range: 0.1m - 5.0m
  - Resolution: 0.01m
  - Samples: 360 (1° resolution)
  - Update rate: 10 Hz

### Physics Parameters
- **Mass:** Base link 1.0kg, wheels 0.1kg each
- **Inertia:** Realistic values for stable simulation
- **Friction:** Configured for realistic wheel-ground interaction

## Troubleshooting

### Common Issues

**Gazebo fails to launch:**
- Ensure Gazebo plugins are properly installed
- Check `GAZEBO_PLUGIN_PATH` environment variable

**Robot doesn't spawn:**
- Verify URDF syntax: `check_urdf model.urdf`
- Check TimerAction delay in launch files

**No laser data:**
- Confirm laser plugin is loaded
- Check topic names: `ros2 topic list | grep scan`

**Build failures:**
- Install missing dependencies: `rosdep install --from-paths src --ignore-src -r -y`
- Clear build cache: `rm -rf build/ install/`

### Performance Optimization
- Adjust physics parameters in world files
- Modify sensor update rates for better performance
- Use appropriate real-time factors

## File Organization

```
ros_ws/
├── src/
│   ├── my_robot_description/     # Robot model definition
│   │   ├── urdf/                 # Robot description files
│   │   ├── launch/               # Basic launch files
│   │   ├── worlds/               # Simple world files
│   │   └── package.xml
│   ├── my_robot_simulation/      # Simulation environments
│   │   ├── launch/               # Advanced launch configurations
│   │   ├── worlds/               # Complex world files
│   │   └── package.xml
│   └── simple_robot/             # Python development package
│       ├── simple_robot/         # Python modules
│       ├── test/                 # Unit tests
│       └── setup.py
├── build/                        # Build artifacts
├── install/                      # Installed packages
└── log/                          # Build and runtime logs
```

## License

This project is licensed under the Apache 2.0 License (simple_robot package) and other open-source licenses as specified in individual package.xml files.

## Maintainer

**wb** - 1878087979@qq.com

For questions, issues, or contributions, please contact the maintainer or create an issue in the project repository.

---

*Built with ROS 2 and Gazebo for educational and research purposes.*