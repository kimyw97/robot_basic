#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SerialSubscriber : public rclcpp::Node
{
public:
    SerialSubscriber() : Node("serial_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "serial_data", 10,
            std::bind(&SerialSubscriber::serial_callback, this, std::placeholders::_1));
    }

private:
    void serial_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
