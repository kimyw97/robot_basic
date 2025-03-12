#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include <random>

class RandomIntPublisher : public rclcpp::Node
{
public:
    RandomIntPublisher() : Node("random_int_publisher"), gen_(rd_()), dist_(0, 100)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int64>("random_int", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&RandomIntPublisher::publish_random_int, this));
    }

private:
    void publish_random_int()
    {
        auto message = std_msgs::msg::Int64();
        message.data = dist_(gen_);
        RCLCPP_INFO(this->get_logger(), "Publishing: %ld", message.data);
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_int_distribution<int64_t> dist_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RandomIntPublisher>());
    rclcpp::shutdown();
    return 0;
}
