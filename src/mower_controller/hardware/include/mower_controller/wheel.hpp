#ifndef MOWER_CONTROLLER_HARDWARE_WHEEL_HPP
#define MOWER_CONTROLLER_HARDWARE_WHEEL_HPP

#include <string>
class Wheel
{
    public:

    std::string name = "";
    double cmd = 0;
    double vel = 0;

    Wheel() = default;

    Wheel(const std::string &wheel_name, int counts_per_rev);
    
    void setup(const std::string &wheel_name, int counts_per_rev);


};


#endif // DIFFDRIVE_ARDUINO_WHEEL_H