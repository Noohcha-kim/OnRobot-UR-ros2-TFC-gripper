#include "onrobot_gripper_driver/gripper_driver_base.hpp"

namespace onrobot_gripper_driver
{

GripperDriverBase::GripperDriverBase(
    std::shared_ptr<ModbusClient> modbus_client,
    rclcpp::Logger logger)
: modbus_(modbus_client), logger_(logger)
{
}

bool GripperDriverBase::read_product_code(uint16_t& product_code)
{
    return modbus_->read_holding_registers(REG_PRODUCT_CODE, 1, &product_code);
}

bool GripperDriverBase::read_firmware_version(uint16_t& major_minor, uint16_t& build)
{
    uint16_t regs[2];
    if (modbus_->read_holding_registers(REG_FW_VERSION_MAJOR_MINOR, 2, regs)) {
        major_minor = regs[0];
        build = regs[1];
        return true;
    }
    return false;
}

bool GripperDriverBase::set_parameter(const std::string& name, double value)
{
    RCLCPP_WARN(logger_, "Parameter '%s' not supported by this gripper type", name.c_str());
    return false;
}

} // namespace onrobot_gripper_driver
