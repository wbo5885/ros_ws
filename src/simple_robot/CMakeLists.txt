cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(simple_robot)

# Find ROS 2 development tools for Python packages
find_package(ament_cmake_auto REQUIRED)
find_package(ament_lint_auto REQUIRED)

# Find other ROS 2 packages needed for your robot
find_package(ros_gz_sim REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(gazebo_ros REQUIRED)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files # <-- 添加这一段
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
# Install world files # <-- 添加这一段
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)
# Standard ament_python package setup (确保这些行存在)
ament_package()

ament_auto_find_test_dependencies()
ament_lint_auto_find_test_dependencies()
