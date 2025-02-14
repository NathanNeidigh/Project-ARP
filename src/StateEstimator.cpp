#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>

class StateEstimator : public rclcpp::Node
{
public:
	explicit StateEstimator() : Node("StateEstimator")
	{
		imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
			"IMU", 10, std::bind(&StateEstimator::imu_topic_callback, this, std::placeholders::_1));
		encoder_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
			"encoder", 10, std::bind(&StateEstimator::encoder_topic_callback, this, std::placeholders::_1));
		state_ = this->create_publisher<geometry_msgs::msg::Pose>("state", 10);
	}
private:
	void imu_topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
	{
		RCLCPP_INFO(this->get_logger(), "Received IMU message: linear_acceleration.y: '%f'", msg->linear_acceleration.y);
	}

	void encoder_topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const
	{
		RCLCPP_INFO(this->get_logger(), "Received Encoder message: position: '%f', velocity: '%f', effort: '%f'",
					msg->position[0], msg->velocity[0], msg->effort[0]);
	}

	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr encoder_subscriber_;
	rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr state_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<StateEstimator>());
	rclcpp::shutdown();
	return 0;
}