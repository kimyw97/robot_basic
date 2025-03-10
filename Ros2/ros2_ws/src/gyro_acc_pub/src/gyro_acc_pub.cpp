#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <tf2/LinearMath/Quaternion.h>

#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUD_RATE B115200

class GyroAccPublisher : public rclcpp::Node
{
public:
    GyroAccPublisher() : Node("gyro_pub")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        serial_fd_ = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_SYNC);

        if (serial_fd_ == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to open serial port: %s", SERIAL_PORT);
            return;
        }

        struct termios tty;
        tcgetattr(serial_fd_, &tty);
        cfsetospeed(&tty, BAUD_RATE);
        cfsetispeed(&tty, BAUD_RATE);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // XON/XOFF 비활성화
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;  // 최소 0 바이트를 읽도록 설정 (Non-blocking)
        tty.c_cc[VTIME] = 1; // 100ms 대기
        tcsetattr(serial_fd_, TCSANOW, &tty);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GyroAccPublisher::read_serial_data, this));
    }

    ~GyroAccPublisher()
    {
        if (serial_fd_ != -1)
        {
            close(serial_fd_);
        }
    }

private:
    void read_serial_data()
    {
        char buffer[256];
        int n = read(serial_fd_, buffer, sizeof(buffer) - 1);
        if (n > 0)
        {
            buffer[n] = '\0'; // 문자열 종료
            auto message = sensor_msgs::msg::Imu();
            message.header.stamp = this->get_clock()->now();
            message.header.frame_id = "imu_link";
            float gx, gy, gz;
            int16_t ax, ay, az;
            sscanf(buffer, "Gyro: %f %f %f | Acc: %hd %hd %hd", &gx, &gy, &gz, &ax, &ay, &az);
            message.angular_velocity.x = gx;
            message.angular_velocity.y = gy;
            message.angular_velocity.z = gz;
            message.linear_acceleration.x = ax;
            message.linear_acceleration.y = ay;
            message.linear_acceleration.z = az;
            tf2::Quaternion q;
            q.setRPY(0, 0, 0); // 기본값 (회전 없음)
            message.orientation.x = q.x();
            message.orientation.y = q.y();
            message.orientation.z = q.z();
            message.orientation.w = q.w();

            // 🌟 공분산 행렬 설정 (필수!)
            for (int i = 0; i < 9; i++)
            {
                message.orientation_covariance[i] = (i % 4 == 0) ? 0.1 : 0.0; // 단위 행렬 (기본 공분산)
                message.angular_velocity_covariance[i] = (i % 4 == 0) ? 0.1 : 0.0;
                message.linear_acceleration_covariance[i] = (i % 4 == 0) ? 0.1 : 0.0;
            }
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Published: Gyro: %f %f %f | Acc: %hd %hd %hd", gx, gy, gz, ax, ay, az);
        }
    }

    int serial_fd_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GyroAccPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
