#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"


class TemperatureMonitor : public rclcpp::Node 
{
    public:
    TemperatureMonitor() : Node("monitor_node"), temperature_threshold_(30.0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("alert_topic",10);
        subscriber_ = this->create_subscription<std_msgs::msg::Float32>("temperature_topic", 10, 
        std::bind(&TemperatureMonitor::callbackTemperature, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Temperature Monitoring System has been started.");
    }

    private:
        void callbackTemperature(const std_msgs::msg::Float32::SharedPtr msg)
        {
            float temperature = msg->data;
            RCLCPP_INFO(this->get_logger(), "Recieved temperature: %.2f degrees Celsius", temperature);

            if (temperature > temperature_threshold_)
            {
                auto alert_msg = std_msgs::msg::Float32();
                alert_msg.data = 1.0;
                publisher_->publish(alert_msg);
                RCLCPP_WARN(this->get_logger(), "High temperature detected: %.2f degrees Celsius", temperature);
            }
        }


        float temperature_threshold_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_; 
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;
        
};  

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TemperatureMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
