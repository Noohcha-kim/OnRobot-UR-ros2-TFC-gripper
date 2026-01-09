#ifndef ONROBOT_GRIPPER_DRIVER__GRIPPERS__VG_DRIVER_HPP_
#define ONROBOT_GRIPPER_DRIVER__GRIPPERS__VG_DRIVER_HPP_

#include "../gripper_driver_base.hpp"

namespace onrobot_gripper_driver
{

// VG10/VGC10 driver (2-channel vacuum gripper)
class VGDriver : public GripperDriverBase
{
public:
    VGDriver(std::shared_ptr<ModbusClient> modbus_client, rclcpp::Logger logger);
    
    bool initialize() override;
    bool grip(const std::shared_ptr<srv::Grip::Request> request,
              std::shared_ptr<srv::Grip::Response> response) override;
    bool release(const std::shared_ptr<srv::Release::Request> request,
                 std::shared_ptr<srv::Release::Response> response) override;
    bool update_state(msg::GripperState& state) override;
    std::string get_gripper_name() const override;

private:
    static constexpr int REG_CHANNEL_A_CONTROL = 0x0000;
    static constexpr int REG_CHANNEL_B_CONTROL = 0x0001;
    static constexpr int REG_CHANNEL_A_VACUUM = 0x0102;
    static constexpr int REG_CHANNEL_B_VACUUM = 0x0103;
    
    enum ControlMode {
        MODE_RELEASE = 0x00,
        MODE_GRIP = 0x01,
        MODE_IDLE = 0x02
    };
    
    static constexpr int NUM_CHANNELS = 2;
    
    // Gripper specifications
    struct GripperSpecs {
        double min_vacuum;
        double max_vacuum;
        std::string model_name;
    };
    
    GripperSpecs specs_;
    uint16_t product_code_;
    
    uint16_t build_control_register(ControlMode mode, uint8_t vacuum_pct);
    bool wait_for_vacuum(double timeout_sec = 3.0);
    double clamp(double value, double min, double max) const;
};

} // namespace onrobot_gripper_driver

#endif
