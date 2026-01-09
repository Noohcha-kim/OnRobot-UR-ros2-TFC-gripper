#ifndef ONROBOT_GRIPPER_DRIVER__GRIPPER_FACTORY_HPP_
#define ONROBOT_GRIPPER_DRIVER__GRIPPER_FACTORY_HPP_

#include "gripper_driver_base.hpp"
#include "modbus_client.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

namespace onrobot_gripper_driver
{

class GripperFactory
{
public:
    static std::unique_ptr<GripperDriverBase> create_gripper(
        std::shared_ptr<ModbusClient> modbus_client,
        rclcpp::Logger logger);
    
    static std::string gripper_type_to_string(GripperType type);
    static GripperType product_code_to_type(uint16_t product_code);
};

} // namespace onrobot_gripper_driver

#endif
