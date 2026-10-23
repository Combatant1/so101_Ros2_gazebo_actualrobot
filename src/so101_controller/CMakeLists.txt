cmake_minimum_required(VERSION 3.8)
project(so101_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL REQUIRED libserial)

add_library(so101_controller SHARED src/so101_interface.cpp)
target_include_directories(so101_controller PRIVATE include ${SERIAL_INCLUDE_DIRS})
target_link_libraries(so101_controller ${SERIAL_LIBRARIES})
target_link_directories(so101_controller PRIVATE ${SERIAL_LIBRARY_DIRS})
ament_target_dependencies(so101_controller
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface so101_controller.xml)

install(TARGETS so101_controller DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(so101_controller)
ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface pluginlib)

ament_package()
