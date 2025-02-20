#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class IMU : public rclcpp::Node
{
public:
	explicit IMU() : Node("IMU")
	{
		publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("IMU", 10);
		airsim_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
			"/airsim_node/RaceCar/imu/Imu", 10, std::bind(&IMU::topic_callback, this, std::placeholders::_1));
	}

private:
	void publish_imu_data()
	{
		auto msg = sensor_msgs::msg::Imu();
		msg.header.stamp = this->now();
		msg.header.frame_id = "imu_link";

		// Example values (replace with real sensor readings)
		msg.orientation.x = 0.0;
		msg.orientation.y = 0.0;
		msg.orientation.z = 0.0;
		msg.orientation.w = 1.0;

		msg.angular_velocity.x = 0.1;
		msg.angular_velocity.y = 0.2;
		msg.angular_velocity.z = 0.3;

		msg.linear_acceleration.x = 0.0;
		msg.linear_acceleration.y = 9.81; // Simulating gravity
		msg.linear_acceleration.z = 0.0;

		publisher_->publish(msg);
		RCLCPP_INFO(this->get_logger(), "Published IMU data.");
	}

	void airsim_publish_imu_data(sensor_msgs::msg::Imu::SharedPtr msg) const
	{
		publisher_->publish(*msg);
		RCLCPP_INFO(this->get_logger(), "Published IMU data.");
	}

	void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
	{
		RCLCPP_INFO(this->get_logger(), "Received IMU message: linear_acceleration.y: '%f'", msg->linear_acceleration.y);
		airsim_publish_imu_data(msg);
	}
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr airsim_subscriber_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<IMU>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
