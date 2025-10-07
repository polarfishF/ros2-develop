#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class SignalProcessor : public rclcpp::Node {
public:
    SignalProcessor() : Node("signal_processor") {
        sine_sub_ = this->create_subscription<std_msgs::msg::Float32>("sine_signal", 10, std::bind(&SignalProcessor::sine_callback, this, std::placeholders::_1));
        square_sub_ = this->create_subscription<std_msgs::msg::Float32>("square_signal", 10, std::bind(&SignalProcessor::square_callback, this, std::placeholders::_1));
        output_pub_ = this->create_publisher<std_msgs::msg::Float32>("output_signal", 10);
    }
private:
    void sine_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        sine_data_ = msg->data;
        process_signal();
    }
    void square_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        square_data_ = msg->data;
        process_signal();
    }
    void process_signal() {
        if ((sine_data_ > 0 && square_data_ > 0) || (sine_data_ < 0 && square_data_ < 0)) {
            auto msg = std::make_unique<std_msgs::msg::Float32>();
            msg->data = sine_data_;
            output_pub_->publish(std::move(msg));
        } else {
            auto msg = std::make_unique<std_msgs::msg::Float32>();
            msg->data = 0.0;
            output_pub_->publish(std::move(msg));
        }
    }
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sine_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr square_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr output_pub_;
    float sine_data_ = 0.0;
    float square_data_ = 0.0;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalProcessor>());
    rclcpp::shutdown();
    return 0;
}