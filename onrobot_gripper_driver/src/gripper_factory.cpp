#include "onrobot_gripper_driver/gripper_factory.hpp"
#include "onrobot_gripper_driver/grippers/fg2_driver.hpp"
#include "onrobot_gripper_driver/grippers/vg_driver.hpp"
#include "onrobot_gripper_driver/grippers/vgp20_driver.hpp"
#include "onrobot_gripper_driver/grippers/fg3_driver.hpp"
#include "onrobot_gripper_driver/grippers/mg10_driver.hpp"
#include "onrobot_gripper_driver/grippers/rg_driver.hpp"

namespace onrobot_gripper_driver
{

std::unique_ptr<GripperDriverBase> GripperFactory::create_gripper(
    std::shared_ptr<ModbusClient> modbus_client,
    rclcpp::Logger logger)
{
    uint16_t product_code = 0;
    if (!modbus_client->read_holding_registers(0x600, 1, &product_code)) {
        RCLCPP_ERROR(logger, "Failed to read gripper product code");
        return nullptr;
    }
    
    GripperType type = product_code_to_type(product_code);
    RCLCPP_INFO(logger, "Detected gripper: %s (product code: 0x%04X)",
                gripper_type_to_string(type).c_str(), product_code);
    
    switch (type) {
        case GripperType::FG2_7:
        case GripperType::FG2_14:
            return std::make_unique<FG2Driver>(modbus_client, logger);
            
        case GripperType::VG10:
        case GripperType::VGC10:
            return std::make_unique<VGDriver>(modbus_client, logger);
            
        case GripperType::VGP20:
        case GripperType::VGP30:
            return std::make_unique<VGP20Driver>(modbus_client, logger);
            
        case GripperType::FG3_15:
        case GripperType::FG3_25:
            return std::make_unique<FG3Driver>(modbus_client, logger);
            
        case GripperType::MG10:
            return std::make_unique<MG10Driver>(modbus_client, logger);
            
        case GripperType::RG2:
        case GripperType::RG6:
            return std::make_unique<RGDriver>(modbus_client, logger);
            
        default:
            RCLCPP_ERROR(logger, "Unknown or unsupported gripper type: 0x%04X", product_code);
            return nullptr;
    }
}

std::string GripperFactory::gripper_type_to_string(GripperType type)
{
    switch (type) {
        case GripperType::FG2_7: return "2FG7";
        case GripperType::FG2_14: return "2FG14";
        case GripperType::FG2P20: return "2FGP20";
        case GripperType::FG3_15: return "3FG15";
        case GripperType::FG3_25: return "3FG25";
        case GripperType::VG10: return "VG10";
        case GripperType::VGC10: return "VGC10";
        case GripperType::VGP20: return "VGP20";
        case GripperType::VGP30: return "VGP30";
        case GripperType::RG2: return "RG2";
        case GripperType::RG6: return "RG6";
        case GripperType::MG10: return "MG10";
        default: return "UNKNOWN";
    }
}

GripperType GripperFactory::product_code_to_type(uint16_t product_code)
{
    return static_cast<GripperType>(product_code);
}

} // namespace onrobot_gripper_driver
