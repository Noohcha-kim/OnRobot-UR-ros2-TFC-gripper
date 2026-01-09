#include "onrobot_gripper_driver/grippers/fg3_driver.hpp"
#include <thread>
#include <chrono>

namespace onrobot_gripper_driver
{

FG3Driver::FG3Driver(std::shared_ptr<ModbusClient> modbus_client, rclcpp::Logger logger)
: GripperDriverBase(modbus_client, logger)
{
}

bool FG3Driver::initialize()
{
    RCLCPP_INFO(logger_, "Initializing 3FG gripper...");
    
    // Read product code
    uint16_t product_code;
    if (!modbus_->read_holding_registers(1536, 1, &product_code)) {
        RCLCPP_ERROR(logger_, "Failed to read product code");
        return false;
    }
    
    product_code_ = product_code;
    
    // Set specifications
    if (product_code == 0x00D0) {  // 3FG15
        specs_ = {
            .min_diameter = 0.0,
            .max_diameter = 150.0,
            .min_force = 50.0,
            .max_force = 500.0,
            .model_name = "3FG15"
        };
        RCLCPP_INFO(logger_, "Detected 3FG15: diameter 0-150mm, force 50-500N");
    } else if (product_code == 0x00D1) {  // 3FG25
        specs_ = {
            .min_diameter = 0.0,
            .max_diameter = 250.0,
            .min_force = 50.0,
            .max_force = 500.0,
            .model_name = "3FG25"
        };
        RCLCPP_INFO(logger_, "Detected 3FG25: diameter 0-250mm, force 50-500N");
    } else {
        specs_ = {
            .min_diameter = 0.0,
            .max_diameter = 150.0,
            .min_force = 50.0,
            .max_force = 500.0,
            .model_name = "3FG_UNKNOWN"
        };
        RCLCPP_WARN(logger_, "Unknown product code 0x%04X, using 3FG15 defaults", product_code);
    }
    
    uint16_t status;
    if (!modbus_->read_holding_registers(REG_STATUS, 1, &status)) {
        RCLCPP_ERROR(logger_, "Failed to read status");
        return false;
    }
    
    RCLCPP_INFO(logger_, "3FG gripper initialized (status: 0x%04X)", status);
    return true;
}

bool FG3Driver::grip(
    const std::shared_ptr<srv::Grip::Request> request,
    std::shared_ptr<srv::Grip::Response> response)
{
    // Clamp values
    double clamped_diameter = clamp(request->target_position, specs_.min_diameter, specs_.max_diameter);
    double clamped_force = clamp(request->force, specs_.min_force, specs_.max_force);
    
    if (clamped_diameter != request->target_position || clamped_force != request->force) {
        RCLCPP_WARN(logger_, "Values clamped - diameter: %.1f->%.1f mm, force: %.1f->%.1f N",
                    request->target_position, clamped_diameter,
                    request->force, clamped_force);
    }
    
    RCLCPP_INFO(logger_, "3FG Grip: diameter=%.1fmm, force=%.1fN",
                clamped_diameter, clamped_force);
    
    uint16_t target_diameter = mm_to_registers(clamped_diameter);
    uint16_t target_force = static_cast<uint16_t>(clamped_force * 10.0);
    
    if (!modbus_->write_register(REG_TARGET_DIAMETER, target_diameter) ||
        !modbus_->write_register(REG_TARGET_FORCE, target_force)) {
        response->success = false;
        response->message = "Failed to write parameters";
        return false;
    }
    
    uint16_t control = (request->grip_mode == 0) ? CTRL_GRIP_EXTERNAL : CTRL_GRIP_INTERNAL;
    if (!modbus_->write_register(REG_CONTROL, control)) {
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
    
    uint16_t final_diameter;
    if (modbus_->read_holding_registers(REG_DIAMETER, 1, &final_diameter)) {
        response->actual_position = registers_to_mm(final_diameter);
    }
    
    response->success = true;
    response->message = "3FG grip completed";
    return true;
}

bool FG3Driver::release(
    const std::shared_ptr<srv::Release::Request> request,
    std::shared_ptr<srv::Release::Response> response)
{
    RCLCPP_INFO(logger_, "3FG Release: opening to max diameter %.1fmm", specs_.max_diameter);
    
    uint16_t max_diameter = mm_to_registers(specs_.max_diameter);
    
    modbus_->write_register(REG_TARGET_DIAMETER, max_diameter);
    modbus_->write_register(REG_CONTROL, CTRL_GRIP_EXTERNAL);
    
    if (request->wait_for_completion) {
        wait_for_motion_complete(5.0);
    }
    
    response->success = true;
    response->message = "3FG release completed";
    return true;
}

bool FG3Driver::update_state(msg::GripperState& state)
{
    uint16_t status, diameter, force;
    
    if (modbus_->read_holding_registers(REG_STATUS, 1, &status) &&
        modbus_->read_holding_registers(REG_DIAMETER, 1, &diameter) &&
        modbus_->read_holding_registers(REG_FORCE, 1, &force)) {
        
        state.position_mm = registers_to_mm(diameter);
        state.force_n = static_cast<double>(force) / 10.0;
        state.status = static_cast<uint8_t>(status & 0xFF);
        state.is_busy = (status & STATUS_BUSY) != 0;
        state.is_gripping = (status & STATUS_GRIP_DETECTED) != 0;
        
        return true;
    }
    
    return false;
}

std::string FG3Driver::get_gripper_name() const
{
    return specs_.model_name;
}

bool FG3Driver::set_parameter(const std::string& name, double value)
{
    return GripperDriverBase::set_parameter(name, value);
}

double FG3Driver::registers_to_mm(uint16_t reg_value) const
{
    return static_cast<double>(reg_value) / 10.0;
}

uint16_t FG3Driver::mm_to_registers(double mm) const
{
    return static_cast<uint16_t>(mm * 10.0);
}

bool FG3Driver::wait_for_motion_complete(double timeout_sec)
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

double FG3Driver::clamp(double value, double min, double max) const
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

} // namespace onrobot_gripper_driver
