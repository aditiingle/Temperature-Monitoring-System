#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class CppNode : public rclcpp::Node 
{
    public:
    CppNode() : Node("cpp_node"), robot_name_("CppPub")
    {
        publisher_ = this->create_publisher<example_interfaces::msg::String>("cpp_topic",10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), 
                                         std::bind(&CppNode::publishNews, this));
        subscriber_ = this->create_subscription<example_interfaces::msg::String>("py_topic", 10, 
        std::bind(&CppNode::callbackRobotNews, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Cpp Node has been started.");
    }

    private:
        void publishNews()
        {
            auto msg = example_interfaces::msg::String();
            msg.data = std::string("Hello from Cpp Publisher, ") + robot_name_ + std::string(".");
            publisher_->publish(msg);
        }

        void callbackRobotNews(const example_interfaces::msg::String::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
        }

        std::string robot_name_;
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
        rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;
};  

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CppNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
