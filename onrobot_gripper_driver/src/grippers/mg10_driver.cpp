#include "onrobot_gripper_driver/grippers/mg10_driver.hpp"

namespace onrobot_gripper_driver
{

MG10Driver::MG10Driver(std::shared_ptr<ModbusClient> modbus_client, rclcpp::Logger logger)
: GripperDriverBase(modbus_client, logger)
{
}

bool MG10Driver::initialize()
{
    RCLCPP_INFO(logger_, "Initializing MG10 gripper...");
    
    // Read product code
    uint16_t product_code;
    if (!modbus_->read_holding_registers(1536, 1, &product_code)) {
        RCLCPP_ERROR(logger_, "Failed to read product code");
        return false;
    }
    
    product_code_ = product_code;
    
    // Set specifications
    specs_ = {
        .min_strength = 0.0,
        .max_strength = 100.0,
        .model_name = "MG10"
    };
    
    RCLCPP_INFO(logger_, "Detected MG10: strength 0-100%%");
    
    // Release magnet
    modbus_->write_register(REG_CONTROL, CTRL_RELEASE);
    
    RCLCPP_INFO(logger_, "MG10 gripper initialized");
    return true;
}

bool MG10Driver::grip(
    const std::shared_ptr<srv::Grip::Request> request,
    std::shared_ptr<srv::Grip::Response> response)
{
    // Clamp strength
    double clamped_strength = clamp(request->target_position, specs_.min_strength, specs_.max_strength);
    
    if (clamped_strength != request->target_position) {
        RCLCPP_WARN(logger_, "Strength clamped: %.1f->%.1f %%",
                    request->target_position, clamped_strength);
    }
    
    RCLCPP_INFO(logger_, "MG10 Grip: strength=%.1f%%", clamped_strength);
    
    uint16_t control = strength_to_control(clamped_strength);
    
    if (!modbus_->write_register(REG_CONTROL, control)) {
        response->success = false;
        response->message = "Failed to activate magnet";
        return false;
    }
    
    response->success = true;
    response->message = "Magnet activated";
    return true;
}

bool MG10Driver::release(
    const std::shared_ptr<srv::Release::Request> request,
    std::shared_ptr<srv::Release::Response> response)
{
    RCLCPP_INFO(logger_, "MG10 Release");
    
    if (!modbus_->write_register(REG_CONTROL, CTRL_RELEASE)) {
        response->success = false;
        response->message = "Failed to release magnet";
        return false;
    }
    
    response->success = true;
    response->message = "Magnet released";
    return true;
}

bool MG10Driver::update_state(msg::GripperState& state)
{
    uint16_t status;
    
    if (modbus_->read_holding_registers(REG_STATUS, 1, &status)) {
        state.status = static_cast<uint8_t>(status & 0xFF);
        state.is_gripping = (status & 0x01) != 0;  // part_gripped bit
        return true;
    }
    
    return false;
}

std::string MG10Driver::get_gripper_name() const
{
    return specs_.model_name;
}

uint16_t MG10Driver::strength_to_control(double strength_pct)
{
    // Map 0-100% to control values
    if (strength_pct >= 90.0) return CTRL_GRIP_100;
    if (strength_pct >= 70.0) return CTRL_GRIP_80;
    if (strength_pct >= 50.0) return CTRL_GRIP_60;
    if (strength_pct >= 30.0) return CTRL_GRIP_40;
    if (strength_pct >= 10.0) return CTRL_GRIP_20;
    return CTRL_RELEASE;
}

double MG10Driver::clamp(double value, double min, double max) const
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

} // namespace onrobot_gripper_driver
