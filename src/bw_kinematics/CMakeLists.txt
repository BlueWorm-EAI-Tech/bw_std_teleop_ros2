cmake_minimum_required(VERSION 3.16)
project(bw_kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)

add_library(${PROJECT_NAME}_algorithm SHARED
  src/algorithm/dls_ik_solver.cpp
)
target_include_directories(${PROJECT_NAME}_algorithm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_algorithm Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}_algorithm
  kdl_parser
  orocos_kdl_vendor
  urdf
)

add_library(${PROJECT_NAME}_manager SHARED
  src/manager/kinematics_manager.cpp
)
target_include_directories(${PROJECT_NAME}_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_manager PUBLIC ${PROJECT_NAME}_algorithm)

add_library(${PROJECT_NAME}_component SHARED
  src/node/kinematics_node.cpp
)
target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_component ${PROJECT_NAME}_manager)
ament_target_dependencies(${PROJECT_NAME}_component
  geometry_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  trajectory_msgs
)
rclcpp_components_register_nodes(${PROJECT_NAME}_component
  "bw_kinematics::KinematicsNode"
)

add_executable(kinematics_node src/node/kinematics_node_main.cpp)
target_link_libraries(kinematics_node ${PROJECT_NAME}_component)
ament_target_dependencies(kinematics_node rclcpp)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(
  TARGETS
    ${PROJECT_NAME}_algorithm
    ${PROJECT_NAME}_manager
    ${PROJECT_NAME}_component
    kinematics_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(
  ${PROJECT_NAME}_algorithm
  ${PROJECT_NAME}_manager
  ${PROJECT_NAME}_component
)
ament_export_dependencies(
  Eigen3
  geometry_msgs
  kdl_parser
  orocos_kdl_vendor
  rclcpp
  rclcpp_components
  sensor_msgs
  trajectory_msgs
  urdf
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_kinematics_manager test/test_kinematics_manager.cpp)
  target_link_libraries(test_kinematics_manager ${PROJECT_NAME}_manager)

  ament_add_gtest(test_kinematics_node test/test_kinematics_node.cpp)
  target_link_libraries(test_kinematics_node ${PROJECT_NAME}_component)
  ament_target_dependencies(test_kinematics_node
    geometry_msgs
    rclcpp
    sensor_msgs
  )

  ament_add_pytest_test(test_standard_defaults test/test_standard_defaults.py)
endif()

ament_package()
