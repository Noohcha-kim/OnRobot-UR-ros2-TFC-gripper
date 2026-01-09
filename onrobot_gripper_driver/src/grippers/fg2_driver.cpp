#include "onrobot_gripper_driver/grippers/fg2_driver.hpp"
#include <thread>
#include <chrono>

namespace onrobot_gripper_driver
{

FG2Driver::FG2Driver(std::shared_ptr<ModbusClient> modbus_client, rclcpp::Logger logger)
: GripperDriverBase(modbus_client, logger)
{
}

bool FG2Driver::initialize()
{
    RCLCPP_INFO(logger_, "Initializing 2FG gripper...");
    
    // Read product code (register 1536 = 0x600)
    uint16_t product_code;
    if (!modbus_->read_holding_registers(1536, 1, &product_code)) {
        RCLCPP_ERROR(logger_, "Failed to read product code");
        return false;
    }
    
    product_code_ = product_code;
    
    // Set specifications based on product code
    if (product_code == 0x00C0) {  // 2FG7
        specs_ = {
            .min_width = 0.0,
            .max_width = 110.0,
            .min_force = 20.0,
            .max_force = 120.0,
            .min_speed = 10.0,
            .max_speed = 100.0,
            .model_name = "2FG7"
        };
        RCLCPP_INFO(logger_, "Detected 2FG7: width 0-110mm, force 20-120N, speed 10-100%%");
    } else if (product_code == 0x00C1) {  // 2FG14
        specs_ = {
            .min_width = 0.0,
            .max_width = 140.0,
            .min_force = 40.0,
            .max_force = 250.0,
            .min_speed = 10.0,
            .max_speed = 100.0,
            .model_name = "2FG14"
        };
        RCLCPP_INFO(logger_, "Detected 2FG14: width 0-140mm, force 40-250N, speed 10-100%%");
    } else {
        // Unknown model, use 2FG7 defaults
        specs_ = {
            .min_width = 0.0,
            .max_width = 110.0,
            .min_force = 20.0,
            .max_force = 120.0,
            .min_speed = 10.0,
            .max_speed = 100.0,
            .model_name = "2FG_UNKNOWN"
        };
        RCLCPP_WARN(logger_, "Unknown product code 0x%04X, using 2FG7 defaults", product_code);
    }
    
    uint16_t status;
    if (!modbus_->read_holding_registers(REG_STATUS, 1, &status)) {
        RCLCPP_ERROR(logger_, "Failed to read gripper status");
        return false;
    }
    
    RCLCPP_INFO(logger_, "2FG gripper initialized (status: 0x%04X)", status);
    return true;
}

bool FG2Driver::grip(
    const std::shared_ptr<srv::Grip::Request> request,
    std::shared_ptr<srv::Grip::Response> response)
{
    // Clamp values to valid range
    double clamped_width = clamp(request->target_position, specs_.min_width, specs_.max_width);
    double clamped_force = clamp(request->force, specs_.min_force, specs_.max_force);
    double clamped_speed = clamp(request->speed, specs_.min_speed, specs_.max_speed);
    
    // Warn if values were clamped
    if (clamped_width != request->target_position ||
        clamped_force != request->force ||
        clamped_speed != request->speed) {
        RCLCPP_WARN(logger_, "Values clamped - width: %.1f->%.1f mm, force: %.1f->%.1f N, speed: %.1f->%.1f %%",
                    request->target_position, clamped_width,
                    request->force, clamped_force,
                    request->speed, clamped_speed);
    }
    
    RCLCPP_INFO(logger_, "2FG Grip: width=%.1fmm, force=%.1fN, speed=%.1f%%",
                clamped_width, clamped_force, clamped_speed);
    
    uint16_t target_width = mm_to_registers(clamped_width);
    uint16_t target_force = static_cast<uint16_t>(clamped_force);
    uint16_t target_speed = static_cast<uint16_t>(clamped_speed);
    
    if (!modbus_->write_register(REG_TARGET_WIDTH, target_width) ||
        !modbus_->write_register(REG_TARGET_FORCE, target_force) ||
        !modbus_->write_register(REG_TARGET_SPEED, target_speed)) {
        response->success = false;
        response->message = "Failed to write parameters";
        return false;
    }
    
    uint16_t command = (request->grip_mode == 0) ? CMD_GRIP_EXTERNAL : CMD_GRIP_INTERNAL;
    if (!modbus_->write_register(REG_COMMAND, command)) {
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
    if (modbus_->read_holding_registers(
        request->grip_mode == 0 ? REG_EXTERNAL_WIDTH : REG_INTERNAL_WIDTH, 
        1, &final_width)) {
        response->actual_position = registers_to_mm(final_width);
    }
    
    response->success = true;
    response->message = "Grip completed";
    return true;
}

bool FG2Driver::release(
    const std::shared_ptr<srv::Release::Request> request,
    std::shared_ptr<srv::Release::Response> response)
{
    RCLCPP_INFO(logger_, "2FG Release: opening to max width %.1fmm", specs_.max_width);
    
    uint16_t max_width = mm_to_registers(specs_.max_width);
    
    modbus_->write_register(REG_TARGET_WIDTH, max_width);
    modbus_->write_register(REG_TARGET_SPEED, 100);
    modbus_->write_register(REG_COMMAND, CMD_GRIP_EXTERNAL);
    
    if (request->wait_for_completion) {
        wait_for_motion_complete(5.0);
    }
    
    response->success = true;
    response->message = "Release completed";
    return true;
}

bool FG2Driver::update_state(msg::GripperState& state)
{
    uint16_t regs[8];
    
    if (!modbus_->read_holding_registers(REG_STATUS, 8, regs)) {
        return false;
    }
    
    state.status = static_cast<uint8_t>(regs[0] & 0xFF);
    state.is_busy = (regs[0] & STATUS_BUSY) != 0;
    state.is_gripping = (regs[0] & STATUS_GRIP_DETECTED) != 0;
    state.position_mm = registers_to_mm(regs[1]);
    
    uint16_t force_reg;
    if (modbus_->read_holding_registers(REG_FORCE, 1, &force_reg)) {
        state.force_n = static_cast<double>(force_reg);
    }
    
    return true;
}

std::string FG2Driver::get_gripper_name() const
{
    return specs_.model_name;
}

bool FG2Driver::set_parameter(const std::string& name, double value)
{
    if (name == "fingertip_offset") {
        uint16_t offset_reg = static_cast<uint16_t>(value * 100.0);
        return modbus_->write_register(REG_FINGERTIP_OFFSET, offset_reg);
    }
    
    return GripperDriverBase::set_parameter(name, value);
}

double FG2Driver::clamp(double value, double min, double max) const
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

double FG2Driver::registers_to_mm(uint16_t reg_value) const
{
    int16_t signed_value = static_cast<int16_t>(reg_value);
    return static_cast<double>(signed_value) / 10.0;
}

uint16_t FG2Driver::mm_to_registers(double mm) const
{
    return static_cast<uint16_t>(mm * 10.0);
}

bool FG2Driver::wait_for_motion_complete(double timeout_sec)
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
