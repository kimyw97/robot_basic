// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ros에서 일반적으로 사용할수있는 include
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp" //데이터 게시할때 사용하는 메시지 유형

using namespace std::chrono_literals; //

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
// 노드 클래스, rclcpp의 노드를 상속 받음
class MinimalPublisher : public rclcpp::Node
{
public:
  // rclcpp의 node는 이름과 횟수를 인자로 받음
  MinimalPublisher()
      : Node("minimal_publisher"), count_(0)
  {
    // 생성자에서 퍼브리셔와 타이머 생성
    // 제너릭으로 메시지의 타입을 넣어주고 인자로 토픽의 이름과 큐의 크기를 넣어줌(buffer size)
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    // 타이머의 주기와 콜백 함수을 넣어줌
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  // 타이머 콜백에 대한 내용
  // 주기가 반복될떄마다 Heloo, world와 count의 값을 증가시켜서 pub
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    // RCLCPP_INFO는 모든 게시된 메시지를 콘솔에 출력하게 하는 매서드
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  // 타이머, 게시자, 카운터에 대한 선언
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  // 스핀락 : 락을 획득 할때 까지 계속 루프를 돌면서 기달리는 방식
  //  스핀 => 계속 대기상태로 넣어줌
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

/*코드 작업이 끝나고 종속성을 추가해야함
1. package.xml
<description>Examples of minimal publisher/subscriber using rclcpp</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
<depend>rclcpp</depend>
<depend>std_msgs</depend> => 이후에 custom한 메시지 타입도 이렇게 넣어야함

2. CMakeList.txt
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/publisher_member_function.cpp) => talker(?)와 소스코드 매핑
ament_target_dependencies(talker rclcpp std_msgs) => 의존성 매핑
*/