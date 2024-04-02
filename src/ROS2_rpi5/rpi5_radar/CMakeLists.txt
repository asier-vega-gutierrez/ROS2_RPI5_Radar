cmake_minimum_required(VERSION 3.8)
project(rpi5_radar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED) # for action server
find_package(rclcpp_components REQUIRED) # for action server
find_package(rpi5_radar_action_interface REQUIRED) # incude the pakage that contains the action interface
include_directories(/usr/include)

# for action server
add_library(action_server SHARED
  src/rpi5_radar_action_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "ACTION_CPP_BUILDING_DLL")
ament_target_dependencies(action_server
  "rpi5_radar_action_interface"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
target_link_libraries(action_server "gpiod")
rclcpp_components_register_node(action_server PLUGIN "rpi5_radar::Rpi5RadarActionServer" EXECUTABLE rpi5_radar_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)



ament_package()
