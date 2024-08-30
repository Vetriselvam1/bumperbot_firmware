cmake_minimum_required(VERSION 3.8)
project(bumperbot_firmware)

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

pkg_check_modules(SERIAL libserial)

# Include directories
include_directories(${SERIAL_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
  src/bumperbot_interface.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include 
  ${SERIAL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${SERIAL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Executable for simple_serial_transmitter
add_executable(simple_serial_transmitter src/simple_serial_transmitter.cpp)
ament_target_dependencies(simple_serial_transmitter rclcpp std_msgs)
target_include_directories(simple_serial_transmitter PUBLIC ${SERIAL_INCLUDE_DIRS})
target_link_libraries(simple_serial_transmitter ${SERIAL_LIBRARIES})

add_executable(simple_serial_receiver src/simple_serial_receiver.cpp)
ament_target_dependencies(simple_serial_receiver rclcpp std_msgs)
target_include_directories(simple_serial_receiver PUBLIC ${SERIAL_INCLUDE_DIRS})
target_link_libraries(simple_serial_receiver ${SERIAL_LIBRARIES})

pluginlib_export_plugin_description_file(hardware_interface bumperbot_interface.xml)

# Install executable
install(TARGETS
  simple_serial_transmitter
  simple_serial_receiver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include
  DESTINATION include
)

# Install Python scripts
install(PROGRAMS
  ${PROJECT_NAME}/simple_serial_transmitter.py
  ${PROJECT_NAME}/mpu6050_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies (if needed)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
