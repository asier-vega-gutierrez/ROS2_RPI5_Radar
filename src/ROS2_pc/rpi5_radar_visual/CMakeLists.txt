cmake_minimum_required(VERSION 3.8)
project(rpi5_radar_visual)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED) # for action client
find_package(rclcpp_components REQUIRED) # for action client
find_package(rpi5_radar_action_interface REQUIRED) # incude the pakage that contains the action interface
find_package(OpenCV REQUIRED) # opencv to work with images

# for action client
add_library(action_client SHARED
  src/rpi5_radar_action_client.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_client
  "rpi5_radar_action_interface"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "OpenCV")
rclcpp_components_register_node(action_client PLUGIN "rpi5_radar_visual::Rpi5RadarActionClient" EXECUTABLE rpi5_radar_action_client)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
