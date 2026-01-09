#include <rclcpp/rclcpp.hpp>
#include "onrobot_gripper_driver/modbus_client.hpp"
#include "onrobot_gripper_driver/gripper_factory.hpp"
#include "onrobot_gripper_driver/msg/gripper_state.hpp"
#include "onrobot_gripper_driver/srv/grip.hpp"
#include "onrobot_gripper_driver/srv/release.hpp"
#include "onrobot_gripper_driver/srv/set_parameter.hpp"

using namespace onrobot_gripper_driver;

class OnRobotGripperNode : public rclcpp::Node
{
public:
    OnRobotGripperNode() : Node("onrobot_gripper_node")
    {
        // Modbus RTU parameters
        this->declare_parameter("serial_port", "/tmp/ttyUR");
        this->declare_parameter("baud_rate", 1000000);
        this->declare_parameter("parity", "E");  // E=Even, O=Odd, N=None
        this->declare_parameter("data_bits", 8);
        this->declare_parameter("stop_bits", 1);
        this->declare_parameter("slave_id", 65);
        this->declare_parameter("polling_rate_hz", 50.0);
        
        std::string serial_port = this->get_parameter("serial_port").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();
        std::string parity_str = this->get_parameter("parity").as_string();
        int data_bits = this->get_parameter("data_bits").as_int();
        int stop_bits = this->get_parameter("stop_bits").as_int();
        int slave_id = this->get_parameter("slave_id").as_int();
        double polling_rate = this->get_parameter("polling_rate_hz").as_double();
        
        // Convert parity string to char
        char parity = 'E';  // Default Even
        if (parity_str == "E" || parity_str == "Even") {
            parity = 'E';
        } else if (parity_str == "O" || parity_str == "Odd") {
            parity = 'O';
        } else if (parity_str == "N" || parity_str == "None") {
            parity = 'N';
        }
        
        RCLCPP_INFO(this->get_logger(), "Connecting to gripper at %s (slave %d, %d baud, parity %c)",
                    serial_port.c_str(), slave_id, baud_rate, parity);
        
        modbus_client_ = std::make_shared<ModbusClient>(
            serial_port, baud_rate, parity, data_bits, stop_bits, slave_id, this->get_logger());
        
        if (!modbus_client_->connect()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to gripper");
            return;
        }
        
        gripper_ = GripperFactory::create_gripper(modbus_client_, this->get_logger());
        if (!gripper_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create gripper driver");
            return;
        }
        
        if (!gripper_->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize gripper");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Gripper '%s' initialized successfully",
                    gripper_->get_gripper_name().c_str());
        
        state_pub_ = this->create_publisher<msg::GripperState>("~/state", 10);
        
        grip_srv_ = this->create_service<srv::Grip>(
            "~/grip",
            std::bind(&OnRobotGripperNode::handle_grip, this, std::placeholders::_1, std::placeholders::_2));
        
        release_srv_ = this->create_service<srv::Release>(
            "~/release",
            std::bind(&OnRobotGripperNode::handle_release, this, std::placeholders::_1, std::placeholders::_2));
        
        set_param_srv_ = this->create_service<srv::SetParameter>(
            "~/set_parameter",
            std::bind(&OnRobotGripperNode::handle_set_parameter, this, std::placeholders::_1, std::placeholders::_2));
        
        auto period = std::chrono::duration<double>(1.0 / polling_rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&OnRobotGripperNode::poll_state, this));
        
        RCLCPP_INFO(this->get_logger(), "OnRobot gripper node started");
    }

private:
    void poll_state()
    {
        if (!modbus_client_->is_connected()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Modbus disconnected, attempting reconnect...");
            if (modbus_client_->reconnect(3, 1000)) {
                gripper_->initialize();
            }
            return;
        }
        
        msg::GripperState state_msg;
        state_msg.header.stamp = this->now();
        state_msg.header.frame_id = "gripper";
        state_msg.gripper_type = gripper_->get_gripper_name();
        
        if (gripper_->update_state(state_msg)) {
            state_pub_->publish(state_msg);
        }
    }
    
    void handle_grip(
        const std::shared_ptr<srv::Grip::Request> request,
        std::shared_ptr<srv::Grip::Response> response)
    {
        gripper_->grip(request, response);
    }
    
    void handle_release(
        const std::shared_ptr<srv::Release::Request> request,
        std::shared_ptr<srv::Release::Response> response)
    {
        gripper_->release(request, response);
    }
    
    void handle_set_parameter(
        const std::shared_ptr<srv::SetParameter::Request> request,
        std::shared_ptr<srv::SetParameter::Response> response)
    {
        response->success = gripper_->set_parameter(request->parameter_name, request->value);
        response->message = response->success ? "Parameter set" : "Parameter not supported";
    }
    
    std::shared_ptr<ModbusClient> modbus_client_;
    std::unique_ptr<GripperDriverBase> gripper_;
    
    rclcpp::Publisher<msg::GripperState>::SharedPtr state_pub_;
    rclcpp::Service<srv::Grip>::SharedPtr grip_srv_;
    rclcpp::Service<srv::Release>::SharedPtr release_srv_;
    rclcpp::Service<srv::SetParameter>::SharedPtr set_param_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OnRobotGripperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
