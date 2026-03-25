#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <utility>
#include <vector>

namespace laplace_pathfinder
{

class RobotNavPlanner : public rclcpp::Node
{
public:
	explicit RobotNavPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
	void timer_callback();
	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

	// Convert world coordinates (meters) to grid indices (row, col).
	// Row 0 is the top of the map (maximum Y).
	std::pair<int, int> loc2grid(double x, double y) const;

	// Convert grid indices (row, col) to the center of the corresponding
	// world cell (meters).
	std::pair<double, double> grid2loc(int row, int col) const;

	// Return the 4-connected neighbor with the highest phi value.
	// Returns {row, col} unchanged when all neighbors are walls or out-of-bounds.
	std::pair<int, int> find_next_step(int row, int col) const;

	// Load a 2-D little-endian float64 .npy file into phi_.
	void load_phi(const std::string & path);

	rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_publisher_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
	rclcpp::TimerBase::SharedPtr timer_;

	// Latest robot pose in the map frame
	double current_x_{0.0};
	double current_y_{0.0};
	bool has_pose_{false};

	// Precomputed Laplace potential field (phi_[row][col])
	std::vector<std::vector<double>> phi_;
	int phi_rows_{0};
	int phi_cols_{0};

	// Map metadata (must match the .yaml alongside the .pgm file)
	double map_origin_x_{-9.9};
	double map_origin_y_{-9.33};
	double map_resolution_{0.05};

	// Runtime parameters
	std::string odom_topic_;
	std::string waypoint_topic_;
	int    step_lookahead_{6};
	double goal_phi_threshold_{18.0};
};

}  // namespace laplace_pathfinder
