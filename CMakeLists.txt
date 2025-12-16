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

#find_package(pinocchio REQUIRED)

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
  sensor_msgs
)

add_library(pos_controller 
  src/inverse_kinematics.cpp 
  src/position_controller.cpp
  include/inverse_kinematics.hpp
  include/position_controller.hpp
)
target_include_directories(pos_controller PUBLIC include)
target_include_directories(trajectory_publisher PUBLIC include)
target_link_libraries(trajectory_publisher pos_controller)

#--------------------CUSTOM NODE--------------------------#
install(TARGETS
  trajectory_publisher
  pos_controller
  #torque_controller
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    description
    launch
    #rviz
    #hardware
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

# Uncomment when hardware plugin is implemented
#ament_export_targets(export_roboto HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
