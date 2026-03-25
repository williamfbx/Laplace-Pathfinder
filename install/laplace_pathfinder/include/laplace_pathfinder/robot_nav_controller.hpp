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
	void waypoint_callback(const geometry_msgs::msg::Point::SharedPtr msg);
	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
	void publish_stop();

	double compute_yaw_radians(const nav_msgs::msg::Odometry & odometry) const;
	double compute_goal_heading_degrees() const;
	double normalize_angle_degrees(double angle_degrees) const;

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
	rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr waypoint_subscription_;

	Waypoint current_goal_;
	double x_start_;
	double y_start_;
	double last_x_;
	double last_y_;

	bool has_active_goal_;
	bool has_pose_;

	std::string cmd_vel_topic_;
	std::string odom_topic_;
	std::string waypoint_topic_;

	double waypoint_tolerance_;

	// Proportional control gains
	double linear_kp_;
	double angular_kp_;
};

}  // namespace laplace_pathfinder
