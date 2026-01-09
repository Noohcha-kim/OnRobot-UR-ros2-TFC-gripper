#ifndef ONROBOT_GRIPPER_DRIVER__GRIPPERS__FG3_DRIVER_HPP_
#define ONROBOT_GRIPPER_DRIVER__GRIPPERS__FG3_DRIVER_HPP_

#include "../gripper_driver_base.hpp"

namespace onrobot_gripper_driver
{

// 3FG15/3FG25 driver (3-finger adaptive gripper)
class FG3Driver : public GripperDriverBase
{
public:
    FG3Driver(std::shared_ptr<ModbusClient> modbus_client, rclcpp::Logger logger);
    
    bool initialize() override;
    bool grip(const std::shared_ptr<srv::Grip::Request> request,
              std::shared_ptr<srv::Grip::Response> response) override;
    bool release(const std::shared_ptr<srv::Release::Request> request,
                 std::shared_ptr<srv::Release::Response> response) override;
    bool update_state(msg::GripperState& state) override;
    std::string get_gripper_name() const override;
    bool set_parameter(const std::string& name, double value) override;

private:
    static constexpr int REG_TARGET_DIAMETER = 0x0000;
    static constexpr int REG_TARGET_FORCE = 0x0001;
    static constexpr int REG_CONTROL = 0x0002;
    
    static constexpr int REG_STATUS = 0x0100;
    static constexpr int REG_DIAMETER_RAW = 0x0101;
    static constexpr int REG_DIAMETER = 0x0102;
    static constexpr int REG_FORCE = 0x0103;
    
    enum Control {
        CTRL_GRIP_EXTERNAL = 0x0001,
        CTRL_GRIP_INTERNAL = 0x0002,
        CTRL_STOP = 0x0003
    };
    
    enum StatusBits {
        STATUS_BUSY = 0x01,
        STATUS_GRIP_DETECTED = 0x02
    };
    
    // Gripper specifications
    struct GripperSpecs {
        double min_diameter;
        double max_diameter;
        double min_force;
        double max_force;
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
