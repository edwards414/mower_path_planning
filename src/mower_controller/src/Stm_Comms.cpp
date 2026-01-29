#include "mower_controller/Stm_Comms.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sstream>

using namespace LibSerial;

StmComms::StmComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
    : timeout_ms_(timeout_ms)
{
    setup(serial_device, baud_rate, timeout_ms);
}

void StmComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{
    timeout_ms_ = timeout_ms;
    
    // Open the serial port
    serial_conn_.Open(serial_device);
    
    // Set baud rate
    BaudRate baud;
    switch(baud_rate) {
        case 9600: baud = BaudRate::BAUD_9600; break;
        case 19200: baud = BaudRate::BAUD_19200; break;
        case 38400: baud = BaudRate::BAUD_38400; break;
        case 57600: baud = BaudRate::BAUD_57600; break;
        case 115200: baud = BaudRate::BAUD_115200; break;
        case 230400: baud = BaudRate::BAUD_230400; break;
        default: baud = BaudRate::BAUD_115200; break;
    }
    serial_conn_.SetBaudRate(baud);
    
    // Set serial port parameters
    serial_conn_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    serial_conn_.SetParity(Parity::PARITY_NONE);
    serial_conn_.SetStopBits(StopBits::STOP_BITS_1);
    serial_conn_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
}

void StmComms::setMotorValues(int val_1, int val_2)
{
    // Send motor command to the STM32
    std::stringstream ss;
    ss << "@M " << val_1 << " " << val_2 << "\r";
    sendMsg(ss.str(), false);
}

void StmComms::setLedOK()
{
    std::stringstream ss;
    ss << "@L 1 0 255 0\r";
    sendMsg(ss.str(), false);
}

void StmComms::setLedError()
{
    std::stringstream ss;
    ss << "@L 2 255 0 0\r";
    sendMsg(ss.str(), false);
}


std::string StmComms::sendMsg(const std::string &msg, bool print_output)
{
    // Write the data to the STM32
    serial_conn_.Write(msg);
    serial_conn_.DrainWriteBuffer();
    
    // Read response
    std::string response;
    try {
        serial_conn_.ReadLine(response, '\n', timeout_ms_);
    } catch (const std::exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("StmComms"), "Error reading from serial: %s", e.what());
        response = "";
    }
    
    if (print_output)
    {
        RCLCPP_INFO(rclcpp::get_logger("StmComms"), "Sent: %s | Received: %s", msg.c_str(), response.c_str());
    }
    
    return response;
}

bool StmComms::is_connected() const
{
    return serial_conn_.IsOpen();
}