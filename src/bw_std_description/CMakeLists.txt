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

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  add_test(
    NAME description_contract
    COMMAND Python3::Interpreter ${CMAKE_CURRENT_SOURCE_DIR}/test/test_description_contract.py
  )
  set_tests_properties(description_contract PROPERTIES TIMEOUT 60)
endif()

ament_package()
