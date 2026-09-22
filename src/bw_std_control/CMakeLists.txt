cmake_minimum_required(VERSION 3.16)
project(bw_std_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(asio_cmake_module REQUIRED)
find_package(ASIO REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

add_library(${PROJECT_NAME}_core STATIC
  src/arm_power_on_sequence.cpp
  src/base_command_limiter.cpp
  src/command_safety.cpp
  src/hardware_contract.cpp
  src/standard_mapping.cpp
  src/command_queue.cpp
  src/frame_codec.cpp
)
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME} SHARED
  src/async_serial_transport.cpp
  src/feedback_handoff.cpp
  src/standard_system_hardware.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${ASIO_INCLUDE_DIRS}
)
target_compile_definitions(${PROJECT_NAME} PRIVATE ${ASIO_DEFINITIONS})
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(${PROJECT_NAME}
  "$<LINK_ONLY:${PROJECT_NAME}_core>" "$<LINK_ONLY:Threads::Threads>")
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface pluginlib rclcpp rclcpp_lifecycle realtime_tools)

add_library(standard_base_controller SHARED src/standard_base_controller.cpp)
target_include_directories(standard_base_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(standard_base_controller PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(standard_base_controller "$<LINK_ONLY:${PROJECT_NAME}_core>")
ament_target_dependencies(standard_base_controller
  controller_interface geometry_msgs hardware_interface pluginlib
  rclcpp rclcpp_lifecycle realtime_tools)

add_library(standard_head_controller SHARED src/standard_head_controller.cpp)
target_include_directories(standard_head_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(standard_head_controller PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(standard_head_controller
  controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle
  realtime_tools sensor_msgs)

pluginlib_export_plugin_description_file(
  hardware_interface bw_std_control_hardware_plugin.xml)
pluginlib_export_plugin_description_file(
  controller_interface bw_std_control_controller_plugin.xml)

install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_core
    standard_base_controller
    standard_head_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(
  FILES
    bw_std_control_hardware_plugin.xml
    bw_std_control_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface geometry_msgs hardware_interface pluginlib
  rclcpp rclcpp_lifecycle realtime_tools sensor_msgs Threads)

ament_package()
