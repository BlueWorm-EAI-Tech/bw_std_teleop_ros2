cmake_minimum_required(VERSION 3.17)
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
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(BW_KINEMATICS_ALGORITHM_MODE "PREBUILT" CACHE STRING
  "Algorithm implementation mode: PREBUILT or SOURCE")
set_property(CACHE BW_KINEMATICS_ALGORITHM_MODE PROPERTY STRINGS PREBUILT SOURCE)

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/bw_kinematics_implementation_provider.cmake")
bw_kinematics_configure_implementation_provider(
  NAME algorithm
  MODE "${BW_KINEMATICS_ALGORITHM_MODE}"
  PUBLIC_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include"
  PREBUILT_LIBRARY
    "${CMAKE_CURRENT_SOURCE_DIR}/algorithm/lib/libbw_kinematics_algorithm.so"
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/algorithm/source"
  EXPORT_SET export_${PROJECT_NAME})

# 平滑器只交付预编译实现, 没有源码模式。
bw_kinematics_configure_implementation_provider(
  NAME smoother
  MODE "PREBUILT"
  PUBLIC_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include"
  PREBUILT_LIBRARY
    "${CMAKE_CURRENT_SOURCE_DIR}/algorithm/lib/libbw_kinematics_smoother.so"
  EXPORT_SET export_${PROJECT_NAME})

add_library(${PROJECT_NAME}_manager SHARED
  src/manager/kinematics_manager.cpp
)
target_include_directories(${PROJECT_NAME}_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_manager PUBLIC
  ${BW_KINEMATICS_ALGORITHM_TARGET})
if(BW_KINEMATICS_ALGORITHM_BUILD_TARGET)
  add_dependencies(${PROJECT_NAME}_manager ${BW_KINEMATICS_ALGORITHM_BUILD_TARGET})
endif()

add_library(${PROJECT_NAME}_component SHARED
  src/node/kinematics_node.cpp
)
target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_component
  ${PROJECT_NAME}_manager
  ${BW_KINEMATICS_SMOOTHER_TARGET})
ament_target_dependencies(${PROJECT_NAME}_component
  ament_index_cpp
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

if(BW_KINEMATICS_ALGORITHM_BUILD_RPATH)
  set_target_properties(${PROJECT_NAME}_manager
    PROPERTIES BUILD_RPATH "${BW_KINEMATICS_ALGORITHM_BUILD_RPATH}")
endif()
if(BW_KINEMATICS_SMOOTHER_BUILD_RPATH)
  set_target_properties(${PROJECT_NAME}_component
    PROPERTIES BUILD_RPATH "${BW_KINEMATICS_SMOOTHER_BUILD_RPATH}")
endif()
set_target_properties(${PROJECT_NAME}_manager ${PROJECT_NAME}_component
  PROPERTIES INSTALL_RPATH "$ORIGIN")
set_target_properties(kinematics_node
  PROPERTIES INSTALL_RPATH "$ORIGIN/..")

install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY config launch assets
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(
  TARGETS
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
  ${PROJECT_NAME}_manager
  ${PROJECT_NAME}_component
)
ament_export_dependencies(
  ament_index_cpp
  geometry_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  trajectory_msgs
)

# 本包只交付预编译实现与编排逻辑, 不携带测试; 测试源码由算法实现方维护。

ament_package()
