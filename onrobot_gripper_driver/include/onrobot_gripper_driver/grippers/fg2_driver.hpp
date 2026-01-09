#ifndef ONROBOT_GRIPPER_DRIVER__GRIPPERS__FG2_DRIVER_HPP_
#define ONROBOT_GRIPPER_DRIVER__GRIPPERS__FG2_DRIVER_HPP_

#include "../gripper_driver_base.hpp"

namespace onrobot_gripper_driver
{

// 2FG7 and 2FG14 driver (same register map)
class FG2Driver : public GripperDriverBase
{
public:
    FG2Driver(std::shared_ptr<ModbusClient> modbus_client, rclcpp::Logger logger);
    
    bool initialize() override;
    bool grip(const std::shared_ptr<srv::Grip::Request> request,
              std::shared_ptr<srv::Grip::Response> response) override;
    bool release(const std::shared_ptr<srv::Release::Request> request,
                 std::shared_ptr<srv::Release::Response> response) override;
    bool update_state(msg::GripperState& state) override;
    std::string get_gripper_name() const override;
    bool set_parameter(const std::string& name, double value) override;

private:
    // Register addresses
    static constexpr int REG_TARGET_WIDTH = 0x0000;
    static constexpr int REG_TARGET_FORCE = 0x0001;
    static constexpr int REG_TARGET_SPEED = 0x0002;
    static constexpr int REG_COMMAND = 0x0003;
    
    static constexpr int REG_STATUS = 0x0100;
    static constexpr int REG_EXTERNAL_WIDTH = 0x0101;
    static constexpr int REG_INTERNAL_WIDTH = 0x0102;
    static constexpr int REG_FORCE = 0x0107;
    
    static constexpr int REG_FINGERTIP_OFFSET = 0x0403;
    
    enum Command {
        CMD_GRIP_EXTERNAL = 1,
        CMD_GRIP_INTERNAL = 2,
        CMD_STOP = 3
    };
    
    enum StatusBits {
        STATUS_BUSY = 0x01,
        STATUS_GRIP_DETECTED = 0x02
    };
    
    // Gripper specifications
    struct GripperSpecs {
        double min_width;
        double max_width;
        double min_force;
        double max_force;
        double min_speed;
        double max_speed;
        std::string model_name;
    };
    
    GripperSpecs specs_;
    uint16_t product_code_;
    
    double registers_to_mm(uint16_t reg_value) const;
    uint16_t mm_to_registers(double mm) const;
    bool wait_for_motion_complete(double timeout_sec = 5.0);
    double clamp(double value, double min, double max) const;
};

} // namespace onrobot_gripper_driver

#endif
