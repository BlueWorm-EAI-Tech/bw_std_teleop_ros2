cmake_minimum_required(VERSION 3.8)
project(bw_std_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  FILES
    urdf/standard.ros2_control.xacro
    urdf/standard.urdf
    urdf/standard.xacro
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
