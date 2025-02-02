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
		timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
										 std::bind(&IMU::publish_imu_data, this));
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
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<IMU>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
