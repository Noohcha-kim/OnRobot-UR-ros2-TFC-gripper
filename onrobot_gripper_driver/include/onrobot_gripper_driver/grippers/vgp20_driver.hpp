#ifndef ONROBOT_GRIPPER_DRIVER__GRIPPERS__VGP20_DRIVER_HPP_
#define ONROBOT_GRIPPER_DRIVER__GRIPPERS__VGP20_DRIVER_HPP_

#include "../gripper_driver_base.hpp"

namespace onrobot_gripper_driver
{

// VGP20/VGP30 driver (4-channel vacuum gripper)
class VGP20Driver : public GripperDriverBase
{
public:
    VGP20Driver(std::shared_ptr<ModbusClient> modbus_client, rclcpp::Logger logger);
    
    bool initialize() override;
    bool grip(const std::shared_ptr<srv::Grip::Request> request,
              std::shared_ptr<srv::Grip::Response> response) override;
    bool release(const std::shared_ptr<srv::Release::Request> request,
                 std::shared_ptr<srv::Release::Response> response) override;
    bool update_state(msg::GripperState& state) override;
    std::string get_gripper_name() const override;

private:
    static constexpr int REG_CHANNEL_A_CONTROL = 0x0000;
    static constexpr int REG_CHANNEL_A_VACUUM = 0x0102;
    
    enum ControlMode {
        MODE_RELEASE = 0x00,
        MODE_GRIP = 0x01,
        MODE_IDLE = 0x02
    };
    
    static constexpr int NUM_CHANNELS = 4;
    
    uint16_t build_control_register(ControlMode mode, uint8_t vacuum_pct);
};

} // namespace onrobot_gripper_driver

#endif
