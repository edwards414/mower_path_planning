#ifndef MOWER_CONTROLLER_STM_COMMS_HPP
#define MOWER_CONTROLLER_STM_COMMS_HPP

#include <serial/serial.h>

#define MAX_RADPS 59.76f
#define PWM_MAX   200.0f
#define RADPS_TO_PWM (PWM_MAX / MAX_RADPS)

class StmComms
{
public:
    StmComms()
    {  }

    StmComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
        : serial_conn_(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms))
    {  }
    void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
    void setMotorValues(int val_1, int val_2);
    std::string sendMsg(const std::string &msg, bool print_output = false);

private:
    serial::Serial serial_conn_;
};