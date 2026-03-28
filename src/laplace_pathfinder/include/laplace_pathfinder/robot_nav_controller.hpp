#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

namespace laplace_pathfinder
{

struct Waypoint
{
	double x;
	double y;
};

class RobotNavController : public rclcpp::Node
{
public:
	explicit RobotNavController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
	// Publishers
	std::string cmd_vel_topic_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

	// Subscribers
	std::string odom_topic_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

	std::string waypoint_topic_;
	rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr waypoint_subscription_;

	// Callbacks
	void waypoint_callback(const geometry_msgs::msg::Point::SharedPtr msg);
	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

	// Control helpers
	void publish_stop();
	double compute_yaw_radians(const nav_msgs::msg::Odometry & odometry) const;
	double compute_goal_heading_degrees() const;
	double normalize_angle_degrees(double angle_degrees) const;

	// Parameters
	double waypoint_tolerance_;
	double linear_kp_;
	double angular_kp_;

	// Robot state
	Waypoint current_goal_{0.0, 0.0};
	double x_start_{0.0};
	double y_start_{0.0};
	double last_x_{0.0};
	double last_y_{0.0};
	bool has_active_goal_{false};
	bool has_pose_{false};
};

}  // namespace laplace_pathfinder
