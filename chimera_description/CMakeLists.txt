cmake_minimum_required(VERSION 3.0.2)
project(chimera_description)

find_package(catkin REQUIRED COMPONENTS
  joint_state_publisher
  joint_state_publisher_gui
  robot_state_publisher
  rviz
  xacro
)

catkin_package()

install(DIRECTORY launch meshes rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
