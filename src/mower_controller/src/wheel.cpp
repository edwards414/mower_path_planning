#include "mower_controller/wheel.hpp"

void Wheel::setup(const std::string &wheel_name)
{
        name = wheel_name + "/" + command_interface_name;
}

