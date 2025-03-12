#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

class IntSubscriber : public rclcpp::Node
{
public:
    IntSubscriber() : Node("int_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Int64>(
            "random_int", 10, std::bind(&IntSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::Int64::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received: %ld", msg->data);
    }

    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IntSubscriber>());
    rclcpp::shutdown();
    return 0;
}
