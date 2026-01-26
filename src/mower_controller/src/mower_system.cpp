
#include "mower_controller/mower_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


hardware_interface::CallbackReturn MowerSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set up the serial communication with STM32
  // Read parameters from URDF
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
  // Set up the wheels with their names

  // stm_comms_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);

  RCLCPP_INFO(get_logger(), "Finished Configuration - Connected to %s", cfg_.device.c_str());
  
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn MowerSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  // for (const auto & [name, descr] : joint_command_interfaces_)
  // {
  //   set_command(name, get_state(name));
  // }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MowerSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
   // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
   std::stringstream ss;
   ss << "Reading states:";
   ss << std::fixed << std::setprecision(2);
   for (const auto & [name, descr] : joint_state_interfaces_)
   {
     if (descr.get_interface_name() == hardware_interface::HW_IF_POSITION)
     {
       // Simulate DiffBot wheels's movement as a first-order system
       // Update the joint status: this is a revolute joint without any limit.
       // Simply integrates
       auto velo = get_command(descr.get_prefix_name() + "/" + hardware_interface::HW_IF_VELOCITY);
       set_state(name, get_state(name) + period.seconds() * velo);
 
       ss << std::endl
          << "\t position " << get_state(name) << " and velocity " << velo << " for '" << name
          << "'!";
     }
   }
   RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
   // END: This part here is for exemplary purposes - Please do not copy to your production code
 
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MowerSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::stringstream ss;
//  if(!stm_comms_.is_connected())
//  {
//   return hardware_interface::return_type::ERROR;
//  }
 // Set the motor values
 for (const auto & [name, descr] : joint_command_interfaces_)
 {
   // Simulate sending commands to the hardware
  //  set_state(name, get_command(name));

   ss << std::fixed << std::setprecision(2) << std::endl
      << "\t" << "command " << get_command(name) << " for '" << name << "'!";
 }
//  stm_comms_.setMotorValues(get_command(wheel_left_.name) * RADPS_TO_PWM, get_command(wheel_right_.name) * RADPS_TO_PWM);
 RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
 return hardware_interface::return_type::OK;
}


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  MowerSystemHardware, 
  hardware_interface::SystemInterface
)