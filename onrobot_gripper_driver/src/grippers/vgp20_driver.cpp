#include "onrobot_gripper_driver/grippers/vgp20_driver.hpp"

namespace onrobot_gripper_driver
{

VGP20Driver::VGP20Driver(std::shared_ptr<ModbusClient> modbus_client, rclcpp::Logger logger)
: GripperDriverBase(modbus_client, logger)
{
}

bool VGP20Driver::initialize()
{
    RCLCPP_INFO(logger_, "Initializing VGP20 gripper...");
    
    uint16_t release_cmd = build_control_register(MODE_RELEASE, 0);
    for (int i = 0; i < NUM_CHANNELS; i++) {
        modbus_->write_register(REG_CHANNEL_A_CONTROL + i, release_cmd);
    }
    
    RCLCPP_INFO(logger_, "VGP20 gripper initialized");
    return true;
}

bool VGP20Driver::grip(
    const std::shared_ptr<srv::Grip::Request> request,
    std::shared_ptr<srv::Grip::Response> response)
{
    uint8_t vacuum_pct = static_cast<uint8_t>(std::min(80.0, std::max(0.0, request->target_position)));
    
    RCLCPP_INFO(logger_, "VGP20 Grip: vacuum=%d%%", vacuum_pct);
    
    uint16_t grip_cmd = build_control_register(MODE_GRIP, vacuum_pct);
    
    std::vector<int> channels = request->channel_mask.empty() ? 
                                std::vector<int>{0, 1, 2, 3} : request->channel_mask;
    
    for (int ch : channels) {
        if (ch >= 0 && ch < NUM_CHANNELS) {
            modbus_->write_register(REG_CHANNEL_A_CONTROL + ch, grip_cmd);
        }
    }
    
    response->success = true;
    response->message = "VGP20 vacuum activated";
    return true;
}

bool VGP20Driver::release(
    const std::shared_ptr<srv::Release::Request> request,
    std::shared_ptr<srv::Release::Response> response)
{
    uint16_t release_cmd = build_control_register(MODE_RELEASE, 0);
    
    std::vector<int> channels = request->channel_mask.empty() ? 
                                std::vector<int>{0, 1, 2, 3} : request->channel_mask;
    
    for (int ch : channels) {
        if (ch >= 0 && ch < NUM_CHANNELS) {
            modbus_->write_register(REG_CHANNEL_A_CONTROL + ch, release_cmd);
        }
    }
    
    response->success = true;
    response->message = "VGP20 vacuum released";
    return true;
}

bool VGP20Driver::update_state(msg::GripperState& state)
{
    uint16_t vacuum_regs[NUM_CHANNELS];
    
    if (modbus_->read_holding_registers(REG_CHANNEL_A_VACUUM, NUM_CHANNELS, vacuum_regs)) {
        state.channel_vacuum_pct.clear();
        state.channel_gripping.clear();
        
        double total_vacuum = 0.0;
        for (int i = 0; i < NUM_CHANNELS; i++) {
            double vac_pct = static_cast<double>(vacuum_regs[i]) / 10.0;
            state.channel_vacuum_pct.push_back(vac_pct);
            state.channel_gripping.push_back(vac_pct > 20.0);
            total_vacuum += vac_pct;
        }
        
        state.force_n = total_vacuum / NUM_CHANNELS;
        state.is_gripping = total_vacuum > 40.0;
        
        return true;
    }
    
    return false;
}

std::string VGP20Driver::get_gripper_name() const
{
    return "VGP20";
}

uint16_t VGP20Driver::build_control_register(ControlMode mode, uint8_t vacuum_pct)
{
    return (static_cast<uint16_t>(mode) << 8) | static_cast<uint16_t>(vacuum_pct);
}

} // namespace onrobot_gripper_driver
