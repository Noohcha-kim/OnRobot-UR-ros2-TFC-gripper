#include "onrobot_gripper_driver/modbus_client.hpp"
#include <thread>
#include <chrono>

namespace onrobot_gripper_driver
{

ModbusClient::ModbusClient(const std::string& serial_port, int baud_rate, char parity,
                           int data_bits, int stop_bits, int slave_id, rclcpp::Logger logger)
: ctx_(nullptr), serial_port_(serial_port), baud_rate_(baud_rate), parity_(parity),
  data_bits_(data_bits), stop_bits_(stop_bits), slave_id_(slave_id), 
  connected_(false), logger_(logger)
{
    clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
}

ModbusClient::~ModbusClient()
{
    disconnect();
}

bool ModbusClient::connect()
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (connected_) {
        return true;
    }
    
    // Create Modbus RTU context
    ctx_ = modbus_new_rtu(serial_port_.c_str(), baud_rate_, parity_, data_bits_, stop_bits_);
    if (ctx_ == nullptr) {
        RCLCPP_ERROR(logger_, "Failed to create Modbus RTU context: %s", modbus_strerror(errno));
        return false;
    }
    
    // Set slave ID
    if (modbus_set_slave(ctx_, slave_id_) != 0) {
        RCLCPP_ERROR(logger_, "Failed to set slave ID: %s", modbus_strerror(errno));
        modbus_free(ctx_);
        ctx_ = nullptr;
        return false;
    }
    
    // Set timeouts
    modbus_set_response_timeout(ctx_, 0, 500000);  // 500ms
    modbus_set_byte_timeout(ctx_, 0, 50000);       // 50ms
    
    // Set RS485 mode
    modbus_rtu_set_serial_mode(ctx_, MODBUS_RTU_RS485);
    
    // Connect to serial port
    if (modbus_connect(ctx_) == -1) {
        RCLCPP_ERROR(logger_, "Modbus RTU connection failed: %s", modbus_strerror(errno));
        modbus_free(ctx_);
        ctx_ = nullptr;
        return false;
    }
    
    connected_ = true;
    RCLCPP_INFO(logger_, "Modbus RTU connected to %s (slave %d, %d baud, parity %c)", 
                serial_port_.c_str(), slave_id_, baud_rate_, parity_);
    return true;
}

void ModbusClient::disconnect()
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (ctx_) {
        modbus_close(ctx_);
        modbus_free(ctx_);
        ctx_ = nullptr;
    }
    connected_ = false;
}

bool ModbusClient::reconnect(int max_attempts, int delay_ms)
{
    disconnect();
    
    for (int i = 0; i < max_attempts; i++) {
        RCLCPP_INFO(logger_, "Reconnect attempt %d/%d", i+1, max_attempts);
        
        if (connect()) {
            return true;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
    
    return false;
}

bool ModbusClient::is_connected() const
{
    return connected_;
}

bool ModbusClient::read_holding_registers(int addr, int nb, uint16_t* dest)
{
    const int max_retries = 3;
    
    for (int attempt = 0; attempt < max_retries; attempt++) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            
            if (!connected_ || !ctx_) {
                if (attempt < max_retries - 1) {
                    RCLCPP_WARN(logger_, "Not connected, will attempt reconnect (%d/%d)", 
                               attempt + 1, max_retries);
                } else {
                    return false;
                }
            } else {
                int rc = modbus_read_registers(ctx_, addr, nb, dest);
                if (rc == -1) {
                    if (attempt < max_retries - 1) {
                        RCLCPP_WARN(logger_, "Read failed at 0x%04X (attempt %d/%d): %s", 
                                   addr, attempt + 1, max_retries, modbus_strerror(errno));
                        connected_ = false;
                    } else {
                        RCLCPP_ERROR(logger_, "Read failed at 0x%04X after %d attempts: %s", 
                                    addr, max_retries, modbus_strerror(errno));
                        connected_ = false;
                        return false;
                    }
                } else {
                    return true;  // Success
                }
            }
        }
        
        // Reconnect outside of lock
        if (attempt < max_retries - 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            reconnect(1, 500);
        }
    }
    
    return false;
}

bool ModbusClient::write_register(int addr, uint16_t value)
{
    const int max_retries = 3;
    
    for (int attempt = 0; attempt < max_retries; attempt++) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            
            if (!connected_ || !ctx_) {
                if (attempt < max_retries - 1) {
                    RCLCPP_WARN(logger_, "Not connected, will attempt reconnect (%d/%d)", 
                               attempt + 1, max_retries);
                } else {
                    return false;
                }
            } else {
                int rc = modbus_write_register(ctx_, addr, value);
                if (rc == -1) {
                    if (attempt < max_retries - 1) {
                        RCLCPP_WARN(logger_, "Write failed at 0x%04X (attempt %d/%d): %s", 
                                   addr, attempt + 1, max_retries, modbus_strerror(errno));
                        connected_ = false;
                    } else {
                        RCLCPP_ERROR(logger_, "Write failed at 0x%04X after %d attempts: %s", 
                                    addr, max_retries, modbus_strerror(errno));
                        connected_ = false;
                        return false;
                    }
                } else {
                    return true;  // Success
                }
            }
        }
        
        // Reconnect outside of lock
        if (attempt < max_retries - 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            reconnect(1, 500);
        }
    }
    
    return false;
}

bool ModbusClient::write_registers(int addr, int nb, const uint16_t* data)
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!connected_ || !ctx_) {
        return false;
    }
    
    int rc = modbus_write_registers(ctx_, addr, nb, data);
    if (rc == -1) {
        RCLCPP_WARN(logger_, "Write multiple failed at 0x%04X: %s", addr, modbus_strerror(errno));
        connected_ = false;
        return false;
    }
    
    return true;
}

} // namespace onrobot_gripper_driver
