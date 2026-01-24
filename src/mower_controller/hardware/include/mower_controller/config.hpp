#ifndef MOWER_CONTROLLER_CONFIG_H
#define MOWER_CONTROLLER_CONFIG_H

#include <string>


struct Config
{
  std::string left_wheel_name = "left_wheel";
  std::string right_wheel_name = "right_wheel";
  float loop_rate = 30;
  std::string device = "/dev/ttyUSB0";
  int baud_rate = 115200;
  int timeout = 1000;
};

#endif // MOWER_CONTROLLER_CONFIG_H