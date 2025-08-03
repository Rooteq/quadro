cmake_minimum_required(VERSION 3.22)
project(quadro)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ros2_socketcan
  ament_index_cpp
  sensor_msgs
  std_msgs
)

#-----------------DELETE LATER ------------------
list(APPEND CMAKE_PREFIX_PATH "/usr/local")
list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots")

find_package(pinocchio REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

#--------------------CUSTOM NODE--------------------------#
add_executable(trajectory_publisher src/simple_trajectory_publisher.cpp)
ament_target_dependencies(trajectory_publisher
  rclcpp
  trajectory_msgs
)

#--------------------CUSTOM NODE--------------------------#
add_executable(leg_ik src/back_legs_trajectory.cpp)
ament_target_dependencies(leg_ik
  rclcpp
  trajectory_msgs
)

#-------------------TORQUE WITH PINOCCHIO--------------------
add_executable(torque_controller src/torque_controller.cpp)
ament_target_dependencies(torque_controller
  rclcpp
  ament_index_cpp
  sensor_msgs
  std_msgs
)
target_link_libraries(torque_controller pinocchio::pinocchio)



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  hardware/hw_actuators.cpp
  # hardware/cybergear_driver.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware>
$<INSTALL_INTERFACE:include/roboto>
)

ament_target_dependencies(
  ${PROJECT_NAME} PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface roboto.xml)

install(
  DIRECTORY hardware/include/
  DESTINATION include/roboto
)

install(
  DIRECTORY hardware/cybergear_driver_core/
  DESTINATION include/roboto
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_roboto
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#--------------------CUSTOM NODE--------------------------#
install(TARGETS
  leg_ik
  trajectory_publisher
  torque_controller
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    description
    launch
    #rviz
    hardware
    config
  DESTINATION "share/${PROJECT_NAME}"
)

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
# the following line skips the linter which checks for copyrights
# comment the line when a copyright and license is added to all source files
set(ament_cmake_copyright_FOUND TRUE)
# the following line skips cpplint (only works in a git repo)
# comment the line when this package is in a git repo and when
# a copyright and license is added to all source files
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_roboto HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
