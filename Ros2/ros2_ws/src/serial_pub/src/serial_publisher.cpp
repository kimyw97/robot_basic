#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUD_RATE B115200

class SerialPublisher : public rclcpp::Node
{
public:
    SerialPublisher() : Node("serial_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("serial_data", 10);
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
            std::bind(&SerialPublisher::read_serial_data, this));
    }

    ~SerialPublisher()
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
            auto message = std_msgs::msg::String();
            message.data = std::string(buffer);
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Published: '%s'", message.data.c_str());
        }
    }

    int serial_fd_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
