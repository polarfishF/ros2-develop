#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <vector>
#include <algorithm>
#include <cmath>
#include <chrono>

class FilterNode : public rclcpp::Node
{
public:
    FilterNode() : Node("filter_node")
    {
        raw_pub_ = this->create_publisher<std_msgs::msg::Float32>("raw_noisy_data", 10);
        median_pub_ = this->create_publisher<std_msgs::msg::Float32>("median_filtered_data", 10);
        lowpass_pub_ = this->create_publisher<std_msgs::msg::Float32>("lowpass_filtered_data", 10);

        raw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "raw_noisy_data", 
            10, 
            std::bind(&FilterNode::raw_data_callback, this, std::placeholders::_1)
        );

        signal_freq_ = 1.0;
        noise_amp_ = 3.0;
        pub_period_ms_ = 10;  
        median_window_ = 5;
        lowpass_alpha_ = 0.1;
        last_lowpass_val_ = 0.0;

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(pub_period_ms_),
            std::bind(&FilterNode::generate_noisy_data, this)
        );

        RCLCPP_INFO(this->get_logger(), "Filter Node (Pub/Sub) Started.");
    }

private:
    void generate_noisy_data()
    {
        auto now = std::chrono::system_clock::now();
        auto now_s = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
        
        float raw_signal = 50.0f * sin(2 * M_PI * signal_freq_ * now_s) + 100.0f;
        float noisy_data = raw_signal + (rand() % (int)(2 * noise_amp_ + 1)) - noise_amp_;

        auto raw_msg = std_msgs::msg::Float32();
        raw_msg.data = noisy_data;
        raw_pub_->publish(raw_msg);
    }

    void raw_data_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        float noisy_data = msg->data;

        median_buffer_.push_back(noisy_data);

        if (median_buffer_.size() > median_window_)
        {
            median_buffer_.erase(median_buffer_.begin());
        }

        if (median_buffer_.size() == median_window_)
        {
            std::vector<float> temp_buffer = median_buffer_;
            std::sort(temp_buffer.begin(), temp_buffer.end());
            float median_result = temp_buffer[median_window_ / 2];  

            auto median_msg = std_msgs::msg::Float32();
            median_msg.data = median_result;
            median_pub_->publish(median_msg);
        }

        float lowpass_result = lowpass_alpha_ * noisy_data + (1 - lowpass_alpha_) * last_lowpass_val_;
        last_lowpass_val_ = lowpass_result;
        
        auto lowpass_msg = std_msgs::msg::Float32();
        lowpass_msg.data = lowpass_result;
        lowpass_pub_->publish(lowpass_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr raw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr median_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lowpass_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr raw_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    float signal_freq_;
    float noise_amp_;
    int pub_period_ms_;
    int median_window_;
    float lowpass_alpha_;
    float last_lowpass_val_;
    std::vector<float> median_buffer_; 
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FilterNode>());
    rclcpp::shutdown();
    return 0;
}