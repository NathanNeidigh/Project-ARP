#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class DriveTrain : public rclcpp::Node
{
public:
    explicit DriveTrain() : Node("DriveTrain")
    {
        subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&DriveTrain::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received Twist message: linear.x: '%f', angular.z: '%f'", msg->linear.x, msg->angular.z);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveTrain>());
    rclcpp::shutdown();
    return 0;
}