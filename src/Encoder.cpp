#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class Encoder : public rclcpp::Node
{
public:
    explicit Encoder(const std::string& name) : Node("encoder_" + name), name_(name)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("encoder_" + name + "_data", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&Encoder::publish_encoder_data, this));
    }

private:
    void publish_encoder_data()
    {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.header.frame_id = "encoder_link";
        msg.name = {name_};

        // Example values (replace with real sensor readings)
        msg.position = {0.0};
        msg.velocity = {0.0};
        msg.effort = {0.0};

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published Encoder data.");
    }

    std::string name_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Encoder>("front_left");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}