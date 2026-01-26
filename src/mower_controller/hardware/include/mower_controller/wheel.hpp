#ifndef MOWER_CONTROLLER_HARDWARE_WHEEL_HPP
#define MOWER_CONTROLLER_HARDWARE_WHEEL_HPP

#include <string>
class Wheel
{
    public:

    std::string name = "";
    std::string command_interface_name = "velocity";
    double cmd = 0;
    double vel = 0;

    Wheel() = default;

    Wheel(const std::string &wheel_name);
    
    void setup(const std::string &wheel_name);


};


#endif // DIFFDRIVE_ARDUINO_WHEEL_H