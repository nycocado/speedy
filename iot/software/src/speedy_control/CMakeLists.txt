cmake_minimum_required(VERSION 3.8)
project(speedy_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(gpiodcxx REQUIRED libgpiodcxx)

include_directories(include)

add_library(speedy_hardware_interface SHARED
  src/SpeedyHardwareInterface.cpp
)
target_include_directories(speedy_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${gpiodcxx_INCLUDE_DIRS}
)
target_link_libraries(speedy_hardware_interface
  ${gpiodcxx_LIBRARIES}
)
ament_target_dependencies(speedy_hardware_interface
  rclcpp
  hardware_interface
  pluginlib
  control_toolbox
)

pluginlib_export_plugin_description_file(hardware_interface speedy_hardware.xml)

install(TARGETS speedy_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES speedy_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()