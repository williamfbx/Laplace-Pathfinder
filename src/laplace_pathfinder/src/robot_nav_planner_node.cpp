/**
 * @file robot_nav_planner_node.cpp
 * @brief Gradient ascent waypoint planner over a precomputed Laplace potential field
 *        with online perturbation for dynamic obstacles.
 *
 * This ROS 2 node loads a static potential field (phi) from disk and, on a fixed timer,
 * follows the steepest-ascent gradient to issue short-horizon waypoints to the controller.
 * It also:
 *  1) Maintains a perturbation field (delta_phi) updated from the perturbation_node.
 *  2) Evaluates phi + delta_phi at each step so dynamic obstacles deflect the path.
 *  3) Performs a multi-step lookahead to smooth waypoint chatter.
 *  4) Stops publishing once the robot reaches a cell whose phi value exceeds goal_phi_threshold.
 *  5) Optionally saves phi + delta_phi to disk on demand for offline debugging.
 *
 * @version 1.0.0
 * @date 2026-03-31
 *
 * Maintainer: Boxiang (William) Fu
 * Project: CMU 16832 Integrated Planning and Learning
 *
 * Subscribers:
 * - /odom               : [nav_msgs::msg::Odometry] Current robot pose.
 * - /perturbation_field : [std_msgs::msg::Float32MultiArray] Dynamic obstacle perturbation patch.
 * - /debug_trigger      : [std_msgs::msg::Empty] Triggers a save of phi+delta_phi to disk.
 *
 * Publishers:
 * - /waypoint           : [geometry_msgs::msg::Point] Next short-horizon waypoint for the controller.
 *
 * Timers:
 * - Planning timer      : Fires at timer_period_s and runs the gradient-ascent waypoint selection.
 *
 * Parameters:
 * - odom_topic          : [string]  Topic for odometry.
 * - waypoint_topic      : [string]  Topic on which waypoints are published.
 * - map_origin_x        : [double]  World x-coordinate of the map origin.
 * - map_origin_y        : [double]  World y-coordinate of the map origin.
 * - map_resolution      : [double]  Meters per grid cell.
 * - timer_period_s      : [double]  Planning timer period in seconds.
 * - step_lookahead      : [int]     Number of gradient steps per timer tick.
 * - goal_phi_threshold  : [double]  phi value above which the robot is considered at the goal.
 * - phi_file_path       : [string]  Path to the precomputed potential field (.npy).
 * - debug_path          : [string]  Path for the debug phi+delta_phi snapshot (.npy).
 */
 
#include "laplace_pathfinder/robot_nav_planner.hpp"
#include "laplace_pathfinder/utils.hpp"

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

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

	debug_path_ = declare_parameter<std::string>("debug_path", "");
	if (debug_path_.empty()) {
		throw std::runtime_error("Debug output file path is not set.");
	}

	// Publishers, subscribers, and timers
	waypoint_publisher_ = create_publisher<geometry_msgs::msg::Point>(waypoint_topic_, 10);

	odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
		odom_topic_, 10,
		std::bind(&RobotNavPlanner::odom_callback, this, std::placeholders::_1));

	perturbation_field_subscription_ = create_subscription<std_msgs::msg::Float32MultiArray>(
		"/perturbation_field", rclcpp::QoS(1),
		std::bind(&RobotNavPlanner::perturbation_field_callback, this, std::placeholders::_1));

	debug_trigger_subscription_ = create_subscription<std_msgs::msg::Empty>(
		"/debug_trigger", 10,
		std::bind(&RobotNavPlanner::debug_trigger_callback, this, std::placeholders::_1));

	timer_ = create_wall_timer(
		std::chrono::duration_cast<std::chrono::nanoseconds>(
			std::chrono::duration<double>(timer_period_s_)),
		std::bind(&RobotNavPlanner::timer_callback, this));

	// Load potential field
	phi_ = laplace_pathfinder::load_npy_float64(phi_path_, phi_rows_, phi_cols_);
	delta_phi_.assign(phi_rows_, std::vector<double>(phi_cols_, 0.0));
	RCLCPP_INFO(
		get_logger(),
		"Loaded phi field: %d rows x %d cols from %s",
		phi_rows_, phi_cols_, phi_path_.c_str());

	RCLCPP_INFO(
		get_logger(),
		"Planner ready. Timer: %.2f s, lookahead: %d cells (%.2f m)",
		timer_period_s_,
		step_lookahead_,
		step_lookahead_ * map_resolution_);
}


void RobotNavPlanner::perturbation_field_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
	// Reset delta_phi_
	for (auto & row : delta_phi_) {
		std::fill(row.begin(), row.end(), 0.0);
	}

	// Message layout: [origin_x, origin_y, resolution, width, height, delta_0, delta_1, ...]
	if (msg->data.size() < 5) {
		return;
	}
	const double pf_ox = static_cast<double>(msg->data[0]);
	const double pf_oy = static_cast<double>(msg->data[1]);
	const double pf_res = static_cast<double>(msg->data[2]);
	const int pf_w = static_cast<int>(msg->data[3]);
	const int pf_h = static_cast<int>(msg->data[4]);

	if (static_cast<std::size_t>(pf_w) * pf_h + 5 != msg->data.size()) {
		return;
	}

	// Map each perturbation_field cell onto the corresponding phi_ cell.
	for (int pr = 0; pr < pf_h; ++pr) {
		for (int pc = 0; pc < pf_w; ++pc) {
			const std::size_t data_idx = 5 + static_cast<std::size_t>(pr) * pf_w + pc;
			const float delta_val = msg->data[data_idx];
			if (delta_val == 0.0f) {
				continue;
			}

			const double wx = pf_ox + pc * pf_res;
			const double wy = pf_oy + pr * pf_res;

			// Convert world coords to phi grid indices
			const int phi_c = static_cast<int>(std::round((wx - map_origin_x_) / map_resolution_));
			const int phi_r = static_cast<int>(std::round(static_cast<double>(phi_rows_ - 1) - (wy - map_origin_y_) / map_resolution_));

			if (phi_r < 0 || phi_r >= phi_rows_ || phi_c < 0 || phi_c >= phi_cols_) {
				continue;
			}

			delta_phi_[phi_r][phi_c] = static_cast<double>(delta_val);
		}
	}
}


void RobotNavPlanner::debug_trigger_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
	std::vector<std::vector<double>> combined(phi_rows_, std::vector<double>(phi_cols_));
	for (int r = 0; r < phi_rows_; ++r) {
		for (int c = 0; c < phi_cols_; ++c) {
			combined[r][c] = phi_[r][c] + delta_phi_[r][c];
		}
	}

	try {
		laplace_pathfinder::save_npy_float64(debug_path_, combined);
		RCLCPP_INFO(get_logger(), "Debug: saved phi+delta_phi (%dx%d) to %s",
			phi_rows_, phi_cols_, debug_path_.c_str());
	} catch (const std::exception & e) {
		RCLCPP_ERROR(get_logger(), "Debug save failed: %s", e.what());
	}
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

	// Follow gradient
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
		const double v = phi_[nr][nc] + delta_phi_[nr][nc];
		if (phi_[nr][nc] == 0.0) {
			continue;
		}
		if (v == 0) {
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
		RCLCPP_WARN(
			get_logger(),
			"No valid gradient step from grid (%d, %d). Staying put.",
			row, col);
		return {row, col};
	}
	
	return {best_row, best_col};
}

}  // namespace laplace_pathfinder
