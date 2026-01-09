#ifndef ONROBOT_GRIPPER_DRIVER__GRIPPERS__MG10_DRIVER_HPP_
#define ONROBOT_GRIPPER_DRIVER__GRIPPERS__MG10_DRIVER_HPP_

#include "../gripper_driver_base.hpp"

namespace onrobot_gripper_driver
{

// MG10 driver (Magnetic gripper)
class MG10Driver : public GripperDriverBase
{
public:
    MG10Driver(std::shared_ptr<ModbusClient> modbus_client, rclcpp::Logger logger);
    
    bool initialize() override;
    bool grip(const std::shared_ptr<srv::Grip::Request> request,
              std::shared_ptr<srv::Grip::Response> response) override;
    bool release(const std::shared_ptr<srv::Release::Request> request,
                 std::shared_ptr<srv::Release::Response> response) override;
    bool update_state(msg::GripperState& state) override;
    std::string get_gripper_name() const override;

private:
    static constexpr int REG_CONTROL = 0x0000;
    static constexpr int REG_STATUS = 0x0100;
    
    enum Control {
        CTRL_RELEASE = 0x0000,
        CTRL_GRIP_20 = 0x0114,
        CTRL_GRIP_40 = 0x0228,
        CTRL_GRIP_60 = 0x033C,
        CTRL_GRIP_80 = 0x0450,
        CTRL_GRIP_100 = 0x0564
    };
    
    struct GripperSpecs {
        double min_strength;
        double max_strength;
        std::string model_name;
    };
    
    GripperSpecs specs_;
    uint16_t product_code_;
    
    uint16_t strength_to_control(double strength_pct);
    double clamp(double value, double min, double max) const;
};

} // namespace onrobot_gripper_driver

#endif
