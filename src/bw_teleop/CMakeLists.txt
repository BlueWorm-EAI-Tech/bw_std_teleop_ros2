cmake_minimum_required(VERSION 3.8)
project(bw_teleop)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(bw_teleop_core
  src/client_session_manager.cpp
  src/pose_mapper.cpp
  src/teleop_manager.cpp
  src/vr_input_mapper.cpp
  src/vr_packet_parser.cpp
  src/vr_stream_manager.cpp
)
target_include_directories(bw_teleop_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(bw_teleop
  src/bw_teleop.cpp
  src/main.cpp
)
target_include_directories(bw_teleop PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(bw_teleop bw_teleop_core)
ament_target_dependencies(bw_teleop
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_vr_pipeline test/test_vr_pipeline.cpp)
  target_include_directories(test_vr_pipeline PRIVATE include)
  target_link_libraries(test_vr_pipeline bw_teleop_core)
endif()

install(TARGETS bw_teleop bw_teleop_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(bw_teleop_core)
ament_package()
