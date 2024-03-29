cmake_minimum_required(VERSION 3.8)
project(rpi5_radar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
include_directories(/usr/include)

# ejecutables
add_executable(rpi5_ultrasonic_pub src/rpi5_ultrasonic_pub.cpp)
ament_target_dependencies(rpi5_ultrasonic_pub rclcpp)
target_link_libraries(rpi5_ultrasonic_pub gpiod)


# para autocompletar ros2 run
install(TARGETS
rpi5_ultrasonic_pub
  DESTINATION lib/${PROJECT_NAME})

# a�adimos el mensaje custom
set(msg_files
  "msg/Ultrasonic.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
# esto hace falta no se para que vale
ament_export_dependencies(rosidl_default_runtime)

# esto linka el codigo de los nodos de este paquete a las interfaces presente en el (asi no hace falta generar un pqeute con los mensajes aparte)
rosidl_target_interfaces(rpi5_ultrasonic_pub
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


ament_package()
