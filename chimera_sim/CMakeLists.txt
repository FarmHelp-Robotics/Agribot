cmake_minimum_required(VERSION 3.0.2)
project(chimera_sim)

find_package(catkin REQUIRED COMPONENTS
  chimera_description
  controller_manager
  diff_drive_controller
  gazebo_ros
  gazebo_ros_control
  joint_state_controller
  position_controllers
  robot_state_publisher
  rqt_robot_steering
  rviz
  xacro
)
catkin_package()

install(DIRECTORY launch worlds models urdf
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
