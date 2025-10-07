#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>

class SignalGenerator : public rclcpp::Node {
public:
    SignalGenerator() : Node("signal_generator") {
        sine_pub_ = this->create_publisher<std_msgs::msg::Float32>("sine_signal", 10);
        square_pub_ = this->create_publisher<std_msgs::msg::Float32>("square_signal", 10);
        sine_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SignalGenerator::sine_callback, this));
        square_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&SignalGenerator::square_callback, this));
        sine_count_ = 0;
        square_state_ = false;
    }
private:
    void sine_callback() {
        auto msg = std::make_unique<std_msgs::msg::Float32>();
        msg->data = sin(2 * M_PI * sine_count_ / 100.0);
        sine_pub_->publish(std::move(msg));
        sine_count_++;
    }
    void square_callback() {
        auto msg = std::make_unique<std_msgs::msg::Float32>();
        msg->data = square_state_ ? 1.0 : -1.0;
        square_pub_->publish(std::move(msg));
        square_state_ = !square_state_;
    }
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sine_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr square_pub_;
    rclcpp::TimerBase::SharedPtr sine_timer_;
    rclcpp::TimerBase::SharedPtr square_timer_;
    int sine_count_;
    bool square_state_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalGenerator>());
    rclcpp::shutdown();
    return 0;
}