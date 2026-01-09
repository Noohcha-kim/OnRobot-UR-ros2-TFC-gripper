#include "onrobot_gripper_driver/grippers/rg_driver.hpp"
#include <thread>
#include <chrono>

namespace onrobot_gripper_driver
{

RGDriver::RGDriver(std::shared_ptr<ModbusClient> modbus_client, rclcpp::Logger logger)
: GripperDriverBase(modbus_client, logger)
{
}

bool RGDriver::initialize()
{
    RCLCPP_INFO(logger_, "Initializing RG gripper...");
    
    // Read product code (register 1536 = 0x600)
    uint16_t product_code;
    if (!modbus_->read_holding_registers(1536, 1, &product_code)) {
        RCLCPP_ERROR(logger_, "Failed to read product code");
        return false;
    }
    
    product_code_ = product_code;
    
    // Set specifications based on product code
    if (product_code == 0x00A0) {  // RG2
        specs_ = {
            .min_width = 0.0,
            .max_width = 110.0,
            .min_force = 3.0,
            .max_force = 40.0,
            .min_speed = 10.0,
            .max_speed = 100.0,
            .model_name = "RG2"
        };
        RCLCPP_INFO(logger_, "Detected RG2: width 0-110mm, force 3-40N, speed 10-100%%");
    } else if (product_code == 0x00A1) {  // RG6
        specs_ = {
            .min_width = 0.0,
            .max_width = 160.0,
            .min_force = 20.0,
            .max_force = 120.0,
            .min_speed = 10.0,
            .max_speed = 100.0,
            .model_name = "RG6"
        };
        RCLCPP_INFO(logger_, "Detected RG6: width 0-160mm, force 20-120N, speed 10-100%%");
    } else {
        // Unknown model, use RG2 defaults
        specs_ = {
            .min_width = 0.0,
            .max_width = 110.0,
            .min_force = 3.0,
            .max_force = 40.0,
            .min_speed = 10.0,
            .max_speed = 100.0,
            .model_name = "RG_UNKNOWN"
        };
        RCLCPP_WARN(logger_, "Unknown product code 0x%04X, using RG2 defaults", product_code);
    }
    
    uint16_t status;
    if (!modbus_->read_holding_registers(REG_STATUS, 1, &status)) {
        RCLCPP_ERROR(logger_, "Failed to read status");
        return false;
    }
    
    RCLCPP_INFO(logger_, "RG gripper initialized (status: 0x%04X)", status);
    return true;
}

bool RGDriver::grip(
    const std::shared_ptr<srv::Grip::Request> request,
    std::shared_ptr<srv::Grip::Response> response)
{
    // Clamp values to valid range
    double clamped_width = clamp(request->target_position, specs_.min_width, specs_.max_width);
    double clamped_force = clamp(request->force, specs_.min_force, specs_.max_force);
    
    // Warn if values were clamped
    if (clamped_width != request->target_position || clamped_force != request->force) {
        RCLCPP_WARN(logger_, "Values clamped - width: %.1f->%.1f mm, force: %.1f->%.1f N",
                    request->target_position, clamped_width,
                    request->force, clamped_force);
    }
    
    RCLCPP_INFO(logger_, "RG Grip: width=%.1fmm, force=%.1fN",
                clamped_width, clamped_force);
    
    uint16_t target_width = mm_to_registers(clamped_width);
    uint16_t target_force = static_cast<uint16_t>(clamped_force * 10.0);
    
    if (!modbus_->write_register(REG_TARGET_FORCE, target_force) ||
        !modbus_->write_register(REG_TARGET_WIDTH, target_width)) {
        response->success = false;
        response->message = "Failed to write parameters";
        return false;
    }
    
    if (!modbus_->write_register(REG_CONTROL, CTRL_GRIP)) {
        response->success = false;
        response->message = "Failed to send grip command";
        return false;
    }
    
    if (request->wait_for_completion) {
        wait_for_motion_complete(5.0);
    } else {
        // Wait for gripper to stabilize before reading actual position
        // Check BUSY bit for up to 500ms
        auto start = std::chrono::steady_clock::now();
        while (true) {
            uint16_t status;
            if (modbus_->read_holding_registers(REG_STATUS, 1, &status)) {
                if ((status & STATUS_BUSY) == 0) {
                    break;  // Motion complete
                }
            }
            
            auto elapsed = std::chrono::steady_clock::now() - start;
            if (std::chrono::duration<double>(elapsed).count() > 0.5) {
                break;  // 500ms timeout
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
    
    uint16_t final_width;
    if (modbus_->read_holding_registers(REG_ACTUAL_WIDTH, 1, &final_width)) {
        response->actual_position = registers_to_mm(final_width);
    }
    
    response->success = true;
    response->message = "RG grip completed";
    return true;
}

bool RGDriver::release(
    const std::shared_ptr<srv::Release::Request> request,
    std::shared_ptr<srv::Release::Response> response)
{
    RCLCPP_INFO(logger_, "RG Release: opening to max width %.1fmm", specs_.max_width);
    
    uint16_t max_width = mm_to_registers(specs_.max_width);
    
    modbus_->write_register(REG_TARGET_WIDTH, max_width);
    modbus_->write_register(REG_CONTROL, CTRL_GRIP);
    
    if (request->wait_for_completion) {
        wait_for_motion_complete(5.0);
    }
    
    response->success = true;
    response->message = "RG release completed";
    return true;
}

bool RGDriver::update_state(msg::GripperState& state)
{
    uint16_t width, status;
    
    if (modbus_->read_holding_registers(REG_ACTUAL_WIDTH, 1, &width) &&
        modbus_->read_holding_registers(REG_STATUS, 1, &status)) {
        
        state.position_mm = registers_to_mm(width);
        state.status = static_cast<uint8_t>(status & 0xFF);
        state.is_busy = (status & STATUS_BUSY) != 0;
        state.is_gripping = (status & STATUS_GRIP_DETECTED) != 0;
        
        return true;
    }
    
    return false;
}

std::string RGDriver::get_gripper_name() const
{
    return specs_.model_name;
}

bool RGDriver::set_parameter(const std::string& name, double value)
{
    if (name == "payload") {
        uint16_t payload_g = static_cast<uint16_t>(value * 1000.0);
        return modbus_->write_register(0x0402, payload_g);
    }
    
    return GripperDriverBase::set_parameter(name, value);
}

double RGDriver::clamp(double value, double min, double max) const
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

double RGDriver::registers_to_mm(uint16_t reg_value) const
{
    return static_cast<double>(reg_value) / 10.0;
}

uint16_t RGDriver::mm_to_registers(double mm) const
{
    return static_cast<uint16_t>(mm * 10.0);
}

bool RGDriver::wait_for_motion_complete(double timeout_sec)
{
    auto start = std::chrono::steady_clock::now();
    
    while (true) {
        uint16_t status;
        if (modbus_->read_holding_registers(REG_STATUS, 1, &status)) {
            if ((status & STATUS_BUSY) == 0) {
                return true;
            }
        }
        
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (std::chrono::duration<double>(elapsed).count() > timeout_sec) {
            return false;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

} // namespace onrobot_gripper_driver
