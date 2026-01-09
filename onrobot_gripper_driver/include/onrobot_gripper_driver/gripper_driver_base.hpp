#ifndef ONROBOT_GRIPPER_DRIVER__GRIPPER_DRIVER_BASE_HPP_
#define ONROBOT_GRIPPER_DRIVER__GRIPPER_DRIVER_BASE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "modbus_client.hpp"
#include "onrobot_gripper_driver/msg/gripper_state.hpp"
#include "onrobot_gripper_driver/srv/grip.hpp"
#include "onrobot_gripper_driver/srv/release.hpp"
#include "onrobot_gripper_driver/srv/set_parameter.hpp"

namespace onrobot_gripper_driver
{

// Product codes from OnRobot Connectivity Guide (Register 0x600)
enum class GripperType : uint16_t {
    FG2_7 = 0xC0,
    FG2_14 = 0xC1,
    FG2P20 = 0xF0,
    FG3_15 = 0x70,
    FG3_25 = 0x71,
    VG10 = 0x10,
    VGC10 = 0x11,
    VGP20 = 0x18,
    VGP30 = 0x19,
    RG2 = 0x20,
    RG6 = 0x21,
    MG10 = 0xA0,
    UNKNOWN = 0xFFFF
};

class GripperDriverBase
{
public:
    GripperDriverBase(
        std::shared_ptr<ModbusClient> modbus_client,
        rclcpp::Logger logger);
    
    virtual ~GripperDriverBase() = default;

    // Pure virtual methods - must be implemented by derived classes
    virtual bool initialize() = 0;
    virtual bool grip(const std::shared_ptr<srv::Grip::Request> request,
                      std::shared_ptr<srv::Grip::Response> response) = 0;
    virtual bool release(const std::shared_ptr<srv::Release::Request> request,
                         std::shared_ptr<srv::Release::Response> response) = 0;
    virtual bool update_state(msg::GripperState& state) = 0;
    virtual std::string get_gripper_name() const = 0;
    
    // Optional - can be overridden
    virtual bool set_parameter(const std::string& name, double value);
    
protected:
    std::shared_ptr<ModbusClient> modbus_;
    rclcpp::Logger logger_;
    
    // Common register addresses (OnRobot standard)
    static constexpr int REG_PRODUCT_CODE = 0x600;
    static constexpr int REG_FW_VERSION_MAJOR_MINOR = 0x604;
    static constexpr int REG_FW_VERSION_BUILD = 0x605;
    
    // Helper functions
    bool read_product_code(uint16_t& product_code);
    bool read_firmware_version(uint16_t& major_minor, uint16_t& build);
};

} // namespace onrobot_gripper_driver

#endif
