#include "laplace_pathfinder/robot_nav_planner.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

// ---------------------------------------------------------------------------
// Minimal .npy reader — supports v1/v2, little-endian float64, C order
// ---------------------------------------------------------------------------
std::vector<std::vector<double>> load_npy_float64(
	const std::string & path, int & rows, int & cols)
{
	std::ifstream f(path, std::ios::binary);
	if (!f) {
		throw std::runtime_error("Cannot open .npy file: " + path);
	}

	// Validate magic "\x93NUMPY"
	char magic[6];
	f.read(magic, 6);
	const std::string npy_magic("\x93NUMPY", 6);
	if (std::string(magic, 6) != npy_magic) {
		throw std::runtime_error("Not a .npy file: " + path);
	}

	// Version
	uint8_t major = 0, minor = 0;
	f.read(reinterpret_cast<char *>(&major), 1);
	f.read(reinterpret_cast<char *>(&minor), 1);
	(void)minor;

	// Header length: 2 bytes for v1, 4 bytes for v2+
	uint32_t hdr_len = 0;
	if (major == 1) {
		uint16_t h16 = 0;
		f.read(reinterpret_cast<char *>(&h16), 2);
		hdr_len = h16;
	} else {
		f.read(reinterpret_cast<char *>(&hdr_len), 4);
	}

	std::string header(hdr_len, '\0');
	f.read(header.data(), hdr_len);

	// Require little-endian float64
	if (header.find("'<f8'") == std::string::npos &&
		header.find("\"<f8\"") == std::string::npos)
	{
		throw std::runtime_error(
			"Expected '<f8' (little-endian float64) in .npy header: " + path);
	}

	// Parse shape — header contains e.g. "'shape': (373, 396)"
	auto pos = header.find("'shape'");
	if (pos == std::string::npos) {
		throw std::runtime_error("Cannot find 'shape' in .npy header: " + path);
	}
	const auto open  = header.find('(', pos);
	const auto close = header.find(')', open);
	const std::string shape_str = header.substr(open + 1, close - open - 1);
	const auto comma = shape_str.find(',');
	rows = std::stoi(shape_str.substr(0, comma));
	cols = std::stoi(shape_str.substr(comma + 1));

	// Read raw doubles (row-major)
	const std::size_t n = static_cast<std::size_t>(rows) * cols;
	std::vector<double> flat(n);
	f.read(
		reinterpret_cast<char *>(flat.data()),
		static_cast<std::streamsize>(n * sizeof(double)));

	if (!f) {
		throw std::runtime_error("File ended prematurely while reading .npy data: " + path);
	}

	std::vector<std::vector<double>> result(rows, std::vector<double>(cols));
	for (int r = 0; r < rows; ++r) {
		for (int c = 0; c < cols; ++c) {
			result[r][c] = flat[static_cast<std::size_t>(r) * cols + c];
		}
	}
	return result;
}

}  // namespace

namespace laplace_pathfinder
{

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
RobotNavPlanner::RobotNavPlanner(const rclcpp::NodeOptions & options)
: Node("robot_nav_planner", options)
{
	odom_topic_     = declare_parameter<std::string>("odom_topic",     "/odom");
	waypoint_topic_ = declare_parameter<std::string>("waypoint_topic", "/waypoint");

	map_origin_x_   = declare_parameter<double>("map_origin_x",   -9.9);
	map_origin_y_   = declare_parameter<double>("map_origin_y",   -9.33);
	map_resolution_ = declare_parameter<double>("map_resolution",   0.05);

	const double timer_period_s = declare_parameter<double>("timer_period_s", 0.5);
	step_lookahead_     = declare_parameter<int>("step_lookahead",     6);
	goal_phi_threshold_ = declare_parameter<double>("goal_phi_threshold", 18.0);

	// Resolve path to the precomputed phi field
	std::string phi_path = declare_parameter<std::string>("phi_file_path", "");
	if (phi_path.empty()) {
		phi_path =
			ament_index_cpp::get_package_share_directory("laplace_pathfinder") +
			"/maps/bookstore_map_phi.npy";
	}

	load_phi(phi_path);
	RCLCPP_INFO(
		get_logger(),
		"Loaded phi field: %d rows x %d cols from %s",
		phi_rows_, phi_cols_, phi_path.c_str());

	waypoint_publisher_ = create_publisher<geometry_msgs::msg::Point>(waypoint_topic_, 10);

	odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
		odom_topic_, 10,
		std::bind(&RobotNavPlanner::odom_callback, this, std::placeholders::_1));

	timer_ = create_wall_timer(
		std::chrono::duration_cast<std::chrono::nanoseconds>(
			std::chrono::duration<double>(timer_period_s)),
		std::bind(&RobotNavPlanner::timer_callback, this));

	RCLCPP_INFO(
		get_logger(),
		"Planner ready. Timer: %.2f s, lookahead: %d cells (%.2f m), "
		"waypoint topic: %s, odom topic: %s",
		timer_period_s,
		step_lookahead_,
		step_lookahead_ * map_resolution_,
		waypoint_topic_.c_str(),
		odom_topic_.c_str());
}

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------
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

	// Stop issuing waypoints once phi indicates the robot is at the goal
	if (phi_[row][col] >= goal_phi_threshold_) {
		RCLCPP_INFO_THROTTLE(
			get_logger(), *get_clock(), 5000,
			"Goal reached (phi=%.2f >= threshold=%.2f). No waypoint issued.",
			phi_[row][col], goal_phi_threshold_);
		return;
	}

	// Follow the gradient for step_lookahead_ steps
	int cur_row = row, cur_col = col;
	for (int i = 0; i < step_lookahead_; ++i) {
		auto [nr, nc] = find_next_step(cur_row, cur_col);
		if (nr == cur_row && nc == cur_col) {
			RCLCPP_WARN_THROTTLE(
				get_logger(), *get_clock(), 5000,
				"No valid gradient step from grid (%d, %d) at step %d. "
				"Robot may be trapped in a wall cell or at a local maximum.",
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
		"Pose=(%.3f, %.3f) grid=(%d,%d) -> waypoint=(%.3f, %.3f) grid=(%d,%d) phi=%.2f",
		current_x_, current_y_, row, col,
		wp_x, wp_y, cur_row, cur_col, phi_[cur_row][cur_col]);
}

// ---------------------------------------------------------------------------
// Coordinate conversions
// ---------------------------------------------------------------------------
std::pair<int, int> RobotNavPlanner::loc2grid(double x, double y) const
{
	const int col = static_cast<int>(
		std::round((x - map_origin_x_) / map_resolution_));
	const int row = static_cast<int>(
		std::round(
			static_cast<double>(phi_rows_ - 1) - (y - map_origin_y_) / map_resolution_));
	return {row, col};
}

std::pair<double, double> RobotNavPlanner::grid2loc(int row, int col) const
{
	const double x = map_origin_x_ + col * map_resolution_;
	const double y = map_origin_y_ +
		(static_cast<double>(phi_rows_ - 1) - row) * map_resolution_;
	return {x, y};
}

// ---------------------------------------------------------------------------
// Gradient step
// ---------------------------------------------------------------------------
std::pair<int, int> RobotNavPlanner::find_next_step(int row, int col) const
{
	constexpr int dr[] = {-1, 1,  0, 0};
	constexpr int dc[] = { 0, 0, -1, 1};

	int    best_row = -1, best_col = -1;
	double best_phi = -std::numeric_limits<double>::infinity();

	for (int i = 0; i < 4; ++i) {
		const int nr = row + dr[i];
		const int nc = col + dc[i];
		if (nr < 0 || nr >= phi_rows_ || nc < 0 || nc >= phi_cols_) {
			continue;
		}
		const double v = phi_[nr][nc];
		if (v == 0.0) {
			continue;  // wall cell
		}
		if (v > best_phi) {
			best_phi = v;
			best_row = nr;
			best_col = nc;
		}
	}

	if (best_row < 0) {
		return {row, col};  // no valid neighbor — stay at current cell
	}
	return {best_row, best_col};
}

// ---------------------------------------------------------------------------
// NPY loader
// ---------------------------------------------------------------------------
void RobotNavPlanner::load_phi(const std::string & path)
{
	phi_ = load_npy_float64(path, phi_rows_, phi_cols_);
}

}  // namespace laplace_pathfinder
