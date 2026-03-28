#include "laplace_pathfinder/robot_nav_planner.hpp"
#include "laplace_pathfinder/utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace laplace_pathfinder
{

RobotNavPlanner::RobotNavPlanner(const rclcpp::NodeOptions & options) : Node("robot_nav_planner", options)
{
	// Parameters
	odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
	waypoint_topic_ = declare_parameter<std::string>("waypoint_topic", "/waypoint");

	map_origin_x_ = declare_parameter<double>("map_origin_x", 0.0);
	map_origin_y_ = declare_parameter<double>("map_origin_y", 0.0);
	map_resolution_ = declare_parameter<double>("map_resolution", 0.05);

	timer_period_s_ = declare_parameter<double>("timer_period_s", 0.5);
	step_lookahead_ = declare_parameter<int>("step_lookahead", 6);
	goal_phi_threshold_ = declare_parameter<double>("goal_phi_threshold", 100.0);

	phi_path_ = declare_parameter<std::string>("phi_file_path", "");
	if (phi_path_.empty()) {
		throw std::runtime_error("Potential field file path is not set.");
	}

	map_file_path_ = declare_parameter<std::string>("map_file_path", "");
	if (map_file_path_.empty()) {
		throw std::runtime_error("Map file path is not set.");
	}

	// Publishers, subscribers, and timers
	waypoint_publisher_ = create_publisher<geometry_msgs::msg::Point>(waypoint_topic_, 10);

	global_costmap_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
		"/global_costmap", rclcpp::QoS(1).transient_local());

	odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
		odom_topic_, 10,
		std::bind(&RobotNavPlanner::odom_callback, this, std::placeholders::_1));

	timer_ = create_wall_timer(
		std::chrono::duration_cast<std::chrono::nanoseconds>(
			std::chrono::duration<double>(timer_period_s_)),
		std::bind(&RobotNavPlanner::timer_callback, this));

	// Load potential field
	phi_ = laplace_pathfinder::load_npy_float64(phi_path_, phi_rows_, phi_cols_);
	RCLCPP_INFO(
		get_logger(),
		"Loaded phi field: %d rows x %d cols from %s",
		phi_rows_, phi_cols_, phi_path_.c_str());

	// Publish global costmap once at startup
	publish_global_costmap();

	RCLCPP_INFO(
		get_logger(),
		"Planner ready. Timer: %.2f s, lookahead: %d cells (%.2f m)",
		timer_period_s_,
		step_lookahead_,
		step_lookahead_ * map_resolution_);
}


void RobotNavPlanner::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	current_x_ = msg->pose.pose.position.x;
	current_y_ = msg->pose.pose.position.y;
	has_pose_  = true;
}


void RobotNavPlanner::timer_callback()
{
	if (!has_pose_) {
		RCLCPP_WARN_THROTTLE(
			get_logger(), *get_clock(), 5000,
			"Waiting for odometry on %s...", odom_topic_.c_str());
		return;
	}

	// Convert current pose to grid
	auto [row, col] = loc2grid(current_x_, current_y_);
	row = std::clamp(row, 0, phi_rows_ - 1);
	col = std::clamp(col, 0, phi_cols_ - 1);

	// Check if already at goal
	if (phi_[row][col] >= goal_phi_threshold_) {
		RCLCPP_INFO_THROTTLE(
			get_logger(), *get_clock(), 5000,
			"Goal reached (phi=%.2f >= threshold=%.2f). No waypoint issued.",
			phi_[row][col], goal_phi_threshold_);
		return;
	}

	// Follow the gradient
	int cur_row = row, cur_col = col;
	for (int i = 0; i < step_lookahead_; ++i) {
		auto [nr, nc] = find_next_step(cur_row, cur_col);

		if (nr == cur_row && nc == cur_col) {
			RCLCPP_WARN_THROTTLE(
				get_logger(), *get_clock(), 5000,
				"No valid gradient step from grid (%d, %d) at step %d.",
				cur_row, cur_col, i);
			break;
		}
		cur_row = nr;
		cur_col = nc;
	}

	// Convert target grid cell back to world coordinates and publish
	auto [wp_x, wp_y] = grid2loc(cur_row, cur_col);

	geometry_msgs::msg::Point waypoint;
	waypoint.x = wp_x;
	waypoint.y = wp_y;
	waypoint.z = 0.0;
	waypoint_publisher_->publish(waypoint);

	RCLCPP_INFO_THROTTLE(
		get_logger(), *get_clock(), 1000,
		"Pose=(%.3f, %.3f) grid=(%d,%d) to waypoint=(%.3f, %.3f) grid=(%d,%d) phi=%.2f",
		current_x_, current_y_, row, col,
		wp_x, wp_y, cur_row, cur_col, phi_[cur_row][cur_col]);
}


std::pair<int, int> RobotNavPlanner::loc2grid(double x, double y) const
{
	const int col = static_cast<int>(
		std::round((x - map_origin_x_) / map_resolution_));
	const int row = static_cast<int>(
		std::round(static_cast<double>(phi_rows_ - 1) - (y - map_origin_y_) / map_resolution_));
	return {row, col};
}


std::pair<double, double> RobotNavPlanner::grid2loc(int row, int col) const
{
	const double x = map_origin_x_ + col * map_resolution_;
	const double y = map_origin_y_ + (static_cast<double>(phi_rows_ - 1) - row) * map_resolution_;
	return {x, y};
}


std::pair<int, int> RobotNavPlanner::find_next_step(int row, int col) const
{
	constexpr int dr[] = {-1, 1,  0, 0};
	constexpr int dc[] = { 0, 0, -1, 1};

	int best_row = -1;
	int best_col = -1;
	double best_phi = -std::numeric_limits<double>::infinity();

	for (int i = 0; i < 4; ++i) {
		const int nr = row + dr[i];
		const int nc = col + dc[i];
		if (nr < 0 || nr >= phi_rows_ || nc < 0 || nc >= phi_cols_) {
			continue;
		}
		const double v = phi_[nr][nc];
		if (v == 0.0) {
			continue;
		}
		if (v > best_phi) {
			best_phi = v;
			best_row = nr;
			best_col = nc;
		}
	}

	// If no valid step found, return current cell
	if (best_row < 0) {
		return {row, col};
	}
	
	return {best_row, best_col};
}


void RobotNavPlanner::publish_global_costmap()
{
	int map_rows = 0, map_cols = 0;
	const auto map_data = laplace_pathfinder::load_npy_int64(map_file_path_, map_rows, map_cols);

	RCLCPP_INFO(
		get_logger(),
		"Loaded map: %d rows x %d cols from %s",
		map_rows, map_cols, map_file_path_.c_str());

	nav_msgs::msg::OccupancyGrid msg;
	msg.header.stamp = now();
	msg.header.frame_id = "odom";

	msg.info.resolution = map_resolution_;
	msg.info.width = static_cast<uint32_t>(map_cols);
	msg.info.height = static_cast<uint32_t>(map_rows);
	msg.info.origin.position.x = map_origin_x_;
	msg.info.origin.position.y = map_origin_y_;
	msg.info.origin.position.z = 0.0;
	msg.info.origin.orientation.w = 1.0;

	msg.data.resize(static_cast<std::size_t>(map_rows) * map_cols);
	for (int r = 0; r < map_rows; ++r) {
		for (int c = 0; c < map_cols; ++c) {
			const int grid_row = (map_rows - 1) - r;
			const std::size_t idx = static_cast<std::size_t>(r) * map_cols + c;
			msg.data[idx] = (map_data[grid_row][c] != 0) ? 100 : 0;
		}
	}

	global_costmap_publisher_->publish(msg);
	RCLCPP_INFO(get_logger(), "Published global costmap");
}

}  // namespace laplace_pathfinder
