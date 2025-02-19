#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

class PathPlanner : public rclcpp::Node
{
public:
	explicit PathPlanner() : Node("PathPlanner")
	{
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
		subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
			"state", 10, std::bind(&PathPlanner::topic_callback, this, std::placeholders::_1));
		timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
										 std::bind(&PathPlanner::publish_cmds, this));
	}
private:
	void publish_cmds()
	{
		auto msg = geometry_msgs::msg::Twist();
		msg.linear.x = 0.1;
		msg.angular.z = 0.1;
		publisher_->publish(msg);
		RCLCPP_INFO(this->get_logger(), "Published Twist message.");
	}

	void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
	{
		RCLCPP_INFO(this->get_logger(), "Received Odometry message: position.x: '%f', position.y: '%f', position.z: '%f'",
					msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	}

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<PathPlanner>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}