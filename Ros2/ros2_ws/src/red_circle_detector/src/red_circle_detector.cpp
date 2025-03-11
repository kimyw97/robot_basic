#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

class RedCircleDetector : public rclcpp::Node
{
public:
    RedCircleDetector() : Node("red_circle_detector")
    {
        // 퍼블리셔 생성
        point_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("red_circle_center", 10);
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("red_circle_image", 10);

        // 웹캠 열기 (V4L2 백엔드 지정)
        cap_.open(0, cv::CAP_V4L2);
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open webcam with V4L2");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Webcam opened successfully with V4L2");
        }

        // 타이머 설정 (100ms마다 실행)
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&RedCircleDetector::detect_red_circle, this));
    }

private:
    void detect_red_circle()
    {
        cv::Mat frame, hsv, mask1, mask2, mask, gray;
        cap_ >> frame;
        if (frame.empty())
            return;

        // BGR -> HSV 변환
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // 적색 범위 설정
        cv::inRange(hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);
        mask = mask1 | mask2;

        // 노이즈 제거
        cv::GaussianBlur(mask, mask, cv::Size(9, 9), 2);

        // 원 검출
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(mask, circles, cv::HOUGH_GRADIENT, 1, mask.rows / 8, 100, 20, 10, 100);

        if (!circles.empty())
        {
            float x = circles[0][0], y = circles[0][1];

            // 원 중심 퍼블리싱
            geometry_msgs::msg::Point point_msg;
            point_msg.x = x;
            point_msg.y = y;
            point_msg.z = 0.0;
            point_publisher_->publish(point_msg);

            // OpenCV 창에 원 표시
            cv::circle(frame, cv::Point(x, y), circles[0][2], cv::Scalar(0, 255, 0), 2);
            cv::circle(frame, cv::Point(x, y), 2, cv::Scalar(0, 255, 255), 3);
        }

        // OpenCV 창 출력 (직접 확인용)
        cv::imshow("Red Circle Detection", frame);
        cv::imshow("Mask", mask);
        cv::waitKey(1);

        // 이미지를 ROS 메시지로 변환 후 퍼블리싱
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        image_publisher_->publish(*msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr point_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RedCircleDetector>());
    rclcpp::shutdown();
    return 0;
}
