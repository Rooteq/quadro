#include "include/hw_actuators.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace roboto
{

} // namespace roboto


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  roboto::CybergearActuator, hardware_interface::ActuatorInterface)
