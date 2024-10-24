cmake_minimum_required(VERSION 3.0.2)
project(locobot_sorting)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  moveit_commander
  locobot_simulation
  message_runtime
  tf2_ros
  tf2_geometry_msgs
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    moveit_commander
    locobot_simulation
    message_runtime
    tf2_ros
    tf2_geometry_msgs
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/object_detection_utils.py
  scripts/camera_object_detection.py
  scripts/object_sorter_grasp.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install the __init__.py file
install(FILES
  scripts/__init__.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  config/locobot.srdf
  config/controllers.yaml
  config/joint_limits.yaml
  config/kinematics.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
