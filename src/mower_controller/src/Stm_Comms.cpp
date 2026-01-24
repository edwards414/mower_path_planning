#include "mower_controller/Stm_Comms.hpp"

#include "rclcpp/rclcpp.hpp"
void StmComms::setup(const std::string &serial_device , int32_t baud_rate , int32_t timeout_ms)
{
    // Initialize the STM32 communication
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
    serial_conn_.open();

}

void StmComms::setMotorValues(int val_1, int val_2)
{
    // Read the data from the STM32
    std::stringstream ss;
    ss << "@m " << val_1 << " " << val_2 << "\r";
    sendMsg(ss.str(), false);
}

std::string StmComms::sendMsg(const std::string &msg, bool print_output = false)
{
    // Write the data to the STM32
    serial_conn_.write(msg);
    std::string response = serial_conn_.readline();
    if (print_output)
    {
        // std::cout << "Sent: " << msg << std::endl;
        RCLCPP_INFO(rclcpp::get_logger("StmComms"), "Sent: %s", msg.c_str());
    }
    return response;
}