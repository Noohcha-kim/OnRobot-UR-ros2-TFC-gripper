#ifndef ONROBOT_GRIPPER_DRIVER__MODBUS_CLIENT_HPP_
#define ONROBOT_GRIPPER_DRIVER__MODBUS_CLIENT_HPP_

#include <modbus/modbus.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <mutex>

namespace onrobot_gripper_driver
{

class ModbusClient
{
public:
    // Modbus RTU constructor
    ModbusClient(const std::string& serial_port, int baud_rate, char parity,
                 int data_bits, int stop_bits, int slave_id, rclcpp::Logger logger);
    ~ModbusClient();

    bool connect();
    void disconnect();
    bool reconnect(int max_attempts = 3, int delay_ms = 1000);
    bool is_connected() const;

    bool read_holding_registers(int addr, int nb, uint16_t* dest);
    bool write_register(int addr, uint16_t value);
    bool write_registers(int addr, int nb, const uint16_t* data);

private:
    modbus_t* ctx_;
    std::string serial_port_;
    int baud_rate_;
    char parity_;
    int data_bits_;
    int stop_bits_;
    int slave_id_;
    bool connected_;
    mutable std::mutex mutex_;
    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;
};

} // namespace onrobot_gripper_driver

#endif
