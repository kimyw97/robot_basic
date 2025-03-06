#include "rclcpp/rclcpp.hpp"
#include "three_ints_interfaces/srv/three_ints.hpp"

#include <memory>

void add(const std::shared_ptr<three_ints_interfaces::srv::ThreeInts::Request> request,
         std::shared_ptr<three_ints_interfaces::srv::ThreeInts::Response> response)
{
    response->sum = request->a + request->b + request->c;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld"
                                              " b: %ld"
                                              " c: %ld",
                request->a, request->b, request->c);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_server");
    // 클래스 생성대신 rclcpp의 기본 모듈에서 가져와서 사용
    // 인자로 콜백함수와 이름을 넣어줌
    rclcpp::Service<three_ints_interfaces::srv::ThreeInts>::SharedPtr service =
        node->create_service<three_ints_interfaces::srv::ThreeInts>("add_three_ints", &add);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three ints.");

    rclcpp::spin(node);
    rclcpp::shutdown();
}