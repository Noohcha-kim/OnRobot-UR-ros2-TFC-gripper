cmake_minimum_required(VERSION 3.8)
project(onrobot_gripper_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find libmodbus
find_library(MODBUS_LIBRARY NAMES modbus REQUIRED)
find_path(MODBUS_INCLUDE_DIR modbus/modbus.h REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GripperState.msg"
  "srv/Grip.srv"
  "srv/Release.srv"
  "srv/SetParameter.srv"
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(
  include
  ${MODBUS_INCLUDE_DIR}
)

# Modbus client library
add_library(modbus_client
  src/modbus_client.cpp
)
ament_target_dependencies(modbus_client rclcpp)
target_link_libraries(modbus_client ${MODBUS_LIBRARY})

# Gripper driver base
add_library(gripper_driver_base
  src/gripper_driver_base.cpp
)
ament_target_dependencies(gripper_driver_base rclcpp)
target_link_libraries(gripper_driver_base modbus_client)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(gripper_driver_base "${cpp_typesupport_target}")

# Individual gripper drivers
add_library(gripper_drivers
  src/grippers/fg2_driver.cpp
  src/grippers/vg_driver.cpp
  src/grippers/vgp20_driver.cpp
  src/grippers/fg3_driver.cpp
  src/grippers/mg10_driver.cpp
  src/grippers/rg_driver.cpp
)
ament_target_dependencies(gripper_drivers rclcpp)
target_link_libraries(gripper_drivers gripper_driver_base modbus_client)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(gripper_drivers "${cpp_typesupport_target}")

# Gripper factory
add_library(gripper_factory
  src/gripper_factory.cpp
)
ament_target_dependencies(gripper_factory rclcpp)
target_link_libraries(gripper_factory gripper_drivers gripper_driver_base)

# Main executable
add_executable(onrobot_gripper_node
  src/main.cpp
)
ament_target_dependencies(onrobot_gripper_node rclcpp)
target_link_libraries(onrobot_gripper_node
  modbus_client
  gripper_driver_base
  gripper_drivers
  gripper_factory
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(onrobot_gripper_node "${cpp_typesupport_target}")

# Install targets
install(TARGETS
  modbus_client
  gripper_driver_base
  gripper_drivers
  gripper_factory
  onrobot_gripper_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs sensor_msgs control_msgs)
ament_export_include_directories(include)

ament_package()
