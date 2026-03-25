#include "laplace_pathfinder/robot_nav_controller.hpp"

#include <cmath>
#include <functional>

namespace
{

constexpr double kPi = 3.14159265358979323846;

}  // namespace

namespace laplace_pathfinder
{

RobotNavController::RobotNavController(const rclcpp::NodeOptions & options)
: Node("robot_nav_controller", options),
	current_goal_{0.0, 0.0},
	x_start_(0.0),
	y_start_(0.0),
	last_x_(0.0),
	last_y_(0.0),
	has_active_goal_(false),
	has_pose_(false)
{
	cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
	odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
	waypoint_topic_ = declare_parameter<std::string>("waypoint_topic", "/waypoint");
	waypoint_tolerance_ = declare_parameter<double>("waypoint_tolerance", 0.15);

	linear_kp_ = declare_parameter<double>("linear_kp", 0.2);
	angular_kp_ = declare_parameter<double>("angular_kp", 0.005);

	cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
	odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
		odom_topic_,
		10,
		std::bind(&RobotNavController::odom_callback, this, std::placeholders::_1));
	waypoint_subscription_ = create_subscription<geometry_msgs::msg::Point>(
		waypoint_topic_,
		10,
		std::bind(&RobotNavController::waypoint_callback, this, std::placeholders::_1));

	RCLCPP_INFO(
		get_logger(),
		"Waiting for waypoint messages on %s. Odometry topic: %s. Command topic: %s.",
		waypoint_topic_.c_str(),
		odom_topic_.c_str(),
		cmd_vel_topic_.c_str());
}

void RobotNavController::waypoint_callback(const geometry_msgs::msg::Point::SharedPtr msg)
{
	current_goal_.x = msg->x;
	current_goal_.y = msg->y;
	x_start_ = last_x_;
	y_start_ = last_y_;
	has_active_goal_ = true;

	RCLCPP_INFO(
		get_logger(),
		"Received waypoint (%.3f, %.3f). Starting from (%.3f, %.3f).",
		current_goal_.x,
		current_goal_.y,
		x_start_,
		y_start_);

	if (!has_pose_) {
		RCLCPP_WARN(
			get_logger(),
			"Waypoint received before odometry. Controller will move once /odom messages arrive.");
	}
}

void RobotNavController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	last_x_ = msg->pose.pose.position.x;
	last_y_ = msg->pose.pose.position.y;
	has_pose_ = true;

	if (!has_active_goal_) {
		publish_stop();
		return;
	}

	const double x_curr = last_x_;
	const double y_curr = last_y_;
	const double dist = std::hypot(current_goal_.x - x_curr, current_goal_.y - y_curr);

	const double current_yaw_deg = compute_yaw_radians(*msg) * 180.0 / kPi;
	const double goal_heading_deg = compute_goal_heading_degrees();
	const double heading_error_deg = normalize_angle_degrees(goal_heading_deg - current_yaw_deg);

	// Unified differential drive controller
	// 1. Rotation based on heading error
	geometry_msgs::msg::Twist command_msg;
	command_msg.angular.z = (angular_kp_ * heading_error_deg);

	// 2. Forward motion based on distance, but reduced if heading is misaligned
	// The closer to the correct heading, the faster we move forward
	const double heading_alignment = std::cos(heading_error_deg * kPi / 180.0);
	const double forward_speed = (linear_kp_ * dist) * std::max(0.0, heading_alignment);
	command_msg.linear.x = forward_speed;

	RCLCPP_INFO_THROTTLE(
		get_logger(),
		*get_clock(),
		1000,
		"Pose=(%.3f, %.3f) Goal=(%.3f, %.3f) Dist=%.3f Yaw=%.1f HeadErr=%.1f Align=%.2f",
		x_curr,
		y_curr,
		current_goal_.x,
		current_goal_.y,
		dist,
		current_yaw_deg,
		heading_error_deg,
		heading_alignment);

	if (dist < waypoint_tolerance_) {
		has_active_goal_ = false;
		x_start_ = x_curr;
		y_start_ = y_curr;
		RCLCPP_INFO(
			get_logger(),
			"Reached waypoint (%.3f, %.3f). Waiting for the next waypoint message.",
			current_goal_.x,
			current_goal_.y);
		publish_stop();
		return;
	}

	cmd_vel_publisher_->publish(command_msg);
}

void RobotNavController::publish_stop()
{
	geometry_msgs::msg::Twist stop_msg;
	cmd_vel_publisher_->publish(stop_msg);
}

double RobotNavController::compute_yaw_radians(const nav_msgs::msg::Odometry & odometry) const
{
	const auto & orientation = odometry.pose.pose.orientation;
	const double siny_cosp = 2.0 * ((orientation.w * orientation.z) + (orientation.x * orientation.y));
	const double cosy_cosp =
		1.0 - (2.0 * ((orientation.y * orientation.y) + (orientation.z * orientation.z)));
	return std::atan2(siny_cosp, cosy_cosp);
}

double RobotNavController::compute_goal_heading_degrees() const
{
	return std::atan2(current_goal_.y - y_start_, current_goal_.x - x_start_) * 180.0 / kPi;
}

double RobotNavController::normalize_angle_degrees(double angle_degrees) const
{
	while (angle_degrees > 180.0) {
		angle_degrees -= 360.0;
	}
	while (angle_degrees < -180.0) {
		angle_degrees += 360.0;
	}
	return angle_degrees;
}

}  // namespace laplace_pathfinder
