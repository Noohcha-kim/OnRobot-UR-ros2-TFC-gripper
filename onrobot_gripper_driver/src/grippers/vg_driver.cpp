#include "onrobot_gripper_driver/grippers/vg_driver.hpp"
#include <thread>
#include <chrono>

namespace onrobot_gripper_driver
{

VGDriver::VGDriver(std::shared_ptr<ModbusClient> modbus_client, rclcpp::Logger logger)
: GripperDriverBase(modbus_client, logger)
{
}

bool VGDriver::initialize()
{
    RCLCPP_INFO(logger_, "Initializing VG gripper...");
    
    // Read product code (register 1536 = 0x600)
    uint16_t product_code;
    if (!modbus_->read_holding_registers(1536, 1, &product_code)) {
        RCLCPP_ERROR(logger_, "Failed to read product code");
        return false;
    }
    
    product_code_ = product_code;
    
    // Set specifications based on product code
    if (product_code == 0x00E0) {  // VG10
        specs_ = {
            .min_vacuum = 0.0,
            .max_vacuum = 100.0,
            .model_name = "VG10"
        };
        RCLCPP_INFO(logger_, "Detected VG10: vacuum 0-100%%");
    } else if (product_code == 0x00E1) {  // VGC10
        specs_ = {
            .min_vacuum = 0.0,
            .max_vacuum = 100.0,
            .model_name = "VGC10"
        };
        RCLCPP_INFO(logger_, "Detected VGC10: vacuum 0-100%%");
    } else {
        // Unknown model, use VG10 defaults
        specs_ = {
            .min_vacuum = 0.0,
            .max_vacuum = 100.0,
            .model_name = "VG_UNKNOWN"
        };
        RCLCPP_WARN(logger_, "Unknown product code 0x%04X, using VG10 defaults", product_code);
    }
    
    // Release both channels
    uint16_t release_cmd = build_control_register(MODE_RELEASE, 0);
    modbus_->write_register(REG_CHANNEL_A_CONTROL, release_cmd);
    modbus_->write_register(REG_CHANNEL_B_CONTROL, release_cmd);
    
    RCLCPP_INFO(logger_, "VG gripper initialized");
    return true;
}

bool VGDriver::grip(
    const std::shared_ptr<srv::Grip::Request> request,
    std::shared_ptr<srv::Grip::Response> response)
{
    // Clamp vacuum percentage to valid range
    double clamped_vacuum = clamp(request->target_position, specs_.min_vacuum, specs_.max_vacuum);
    
    if (clamped_vacuum != request->target_position) {
        RCLCPP_WARN(logger_, "Vacuum clamped: %.1f->%.1f %%",
                    request->target_position, clamped_vacuum);
    }
    
    uint8_t vacuum_pct = static_cast<uint8_t>(clamped_vacuum);
    
    RCLCPP_INFO(logger_, "VG Grip: vacuum=%d%%", vacuum_pct);
    
    uint16_t grip_cmd = build_control_register(MODE_GRIP, vacuum_pct);
    
    std::vector<int> channels = request->channel_mask.empty() ? 
                                std::vector<int>{0, 1} : request->channel_mask;
    
    for (int ch : channels) {
        int reg = (ch == 0) ? REG_CHANNEL_A_CONTROL : REG_CHANNEL_B_CONTROL;
        if (!modbus_->write_register(reg, grip_cmd)) {
            response->success = false;
            response->message = "Failed to write vacuum command";
            return false;
        }
    }
    
    if (request->wait_for_completion) {
        wait_for_vacuum(3.0);
    }
    
    response->success = true;
    response->message = "Vacuum grip activated";
    return true;
}

bool VGDriver::release(
    const std::shared_ptr<srv::Release::Request> request,
    std::shared_ptr<srv::Release::Response> response)
{
    RCLCPP_INFO(logger_, "VG Release");
    
    uint16_t release_cmd = build_control_register(MODE_RELEASE, 0);
    
    std::vector<int> channels = request->channel_mask.empty() ? 
                                std::vector<int>{0, 1} : request->channel_mask;
    
    for (int ch : channels) {
        int reg = (ch == 0) ? REG_CHANNEL_A_CONTROL : REG_CHANNEL_B_CONTROL;
        if (!modbus_->write_register(reg, release_cmd)) {
            response->success = false;
            response->message = "Failed to release vacuum";
            return false;
        }
    }
    
    response->success = true;
    response->message = "Vacuum released";
    return true;
}

bool VGDriver::update_state(msg::GripperState& state)
{
    uint16_t vacuum_a, vacuum_b;
    
    if (modbus_->read_holding_registers(REG_CHANNEL_A_VACUUM, 1, &vacuum_a) &&
        modbus_->read_holding_registers(REG_CHANNEL_B_VACUUM, 1, &vacuum_b)) {
        
        double vac_a_pct = static_cast<double>(vacuum_a) / 10.0;
        double vac_b_pct = static_cast<double>(vacuum_b) / 10.0;
        
        state.channel_vacuum_pct = {vac_a_pct, vac_b_pct};
        state.force_n = (vac_a_pct + vac_b_pct) / 2.0;
        state.is_gripping = (vac_a_pct > 20.0) || (vac_b_pct > 20.0);
        
        return true;
    }
    
    return false;
}

std::string VGDriver::get_gripper_name() const
{
    return specs_.model_name;
}

double VGDriver::clamp(double value, double min, double max) const
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

uint16_t VGDriver::build_control_register(ControlMode mode, uint8_t vacuum_pct)
{
    return (static_cast<uint16_t>(mode) << 8) | static_cast<uint16_t>(vacuum_pct);
}

bool VGDriver::wait_for_vacuum(double timeout_sec)
{
    auto start = std::chrono::steady_clock::now();
    
    while (true) {
        uint16_t vacuum_a, vacuum_b;
        if (modbus_->read_holding_registers(REG_CHANNEL_A_VACUUM, 1, &vacuum_a) &&
            modbus_->read_holding_registers(REG_CHANNEL_B_VACUUM, 1, &vacuum_b)) {
            
            double vac_a = static_cast<double>(vacuum_a) / 10.0;
            double vac_b = static_cast<double>(vacuum_b) / 10.0;
            
            if (vac_a > 30.0 || vac_b > 30.0) {
                return true;
            }
        }
        
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (std::chrono::duration<double>(elapsed).count() > timeout_sec) {
            return false;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

} // namespace onrobot_gripper_driver
