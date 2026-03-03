
#include "mower_controller/mower_system.hpp"

#include <algorithm>
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
  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  // Set up the wheels with their names
  wheel_left_.setup(cfg_.left_wheel_name);
  wheel_right_.setup(cfg_.right_wheel_name);
  RCLCPP_INFO(get_logger(), "Finished Configuration - Connected to %s", cfg_.device.c_str());
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MowerSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  stm_comms_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);
  stm_comms_.setLedOK();
  if(!stm_comms_.is_connected())
  {
    RCLCPP_ERROR(get_logger(), "Failed to connect to the STM32");
    return hardware_interface::CallbackReturn::ERROR;
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  // for (const auto & [name, descr] : joint_command_interfaces_)
  // {
  //   set_command(name, get_state(name));
  // }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MowerSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  stm_comms_.setLedError();
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MowerSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Open-loop feedback: echo the command back as state so that
  // diff_drive_controller receives non-zero velocity readings.
  wheel_left_.vel  = wheel_left_.cmd;
  wheel_right_.vel = wheel_right_.cmd;

  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    const std::string joint = descr.get_prefix_name();

    if (descr.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
    {
      if (joint == wheel_left_.name)
        set_state(name, wheel_left_.vel);
      else if (joint == wheel_right_.name)
        set_state(name, wheel_right_.vel);
    }
    else if (descr.get_interface_name() == hardware_interface::HW_IF_POSITION)
    {
      if (joint == wheel_left_.name)
        set_state(name, get_state(name) + period.seconds() * wheel_left_.vel);
      else if (joint == wheel_right_.name)
        set_state(name, get_state(name) + period.seconds() * wheel_right_.vel);
    }
    else if (descr.get_interface_name() == hardware_interface::HW_IF_EFFORT)
    {
      if (joint == "mower_joint")
        set_state(name, mower_blade_cmd_);
    }
  }

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 500,
    "Read states — left vel: %.3f rad/s, right vel: %.3f rad/s, blade effort: %.1f",
    wheel_left_.vel, wheel_right_.vel, mower_blade_cmd_);

  return hardware_interface::return_type::OK;
}



  hardware_interface::return_type MowerSystemHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if(!stm_comms_.is_connected())
    {
      return hardware_interface::return_type::ERROR;
    }
    // Set the motor values
    for (const auto & [name, descr] : joint_command_interfaces_)
    {
      // Collect commands for sending to hardware
      if (name == wheel_left_.name)
      {
        wheel_left_.cmd = get_command(name);
      }
      else if (name == wheel_right_.name)
      {
        wheel_right_.cmd = get_command(name);
      }
      else if (name == "mower_joint/effort")
      {
        mower_blade_cmd_ = get_command(name);
      }
    }
    stm_comms_.setMotorValues(wheel_left_.cmd * RADPS_TO_PWM, wheel_right_.cmd * RADPS_TO_PWM);
    int blade_val = static_cast<int>(std::clamp(mower_blade_cmd_, -100.0, 100.0));
    stm_comms_.setMowerBladeValue(blade_val);


    
    RCLCPP_INFO(get_logger(), "Left wheel command: %f, Right wheel command: %f", wheel_left_.cmd * RADPS_TO_PWM, wheel_right_.cmd * RADPS_TO_PWM);
    RCLCPP_INFO(get_logger(), "Mower blade command: %f", mower_blade_cmd_);
    //  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
    return hardware_interface::return_type::OK;
  }


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  MowerSystemHardware, 
  hardware_interface::SystemInterface
)