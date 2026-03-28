#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
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
	// Publishers
	std::string waypoint_topic_;
	rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_publisher_;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_publisher_;

	// Subscribers
	std::string odom_topic_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

	// Functions
	// Callbacks
	void timer_callback();
	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
	void publish_global_costmap();

	// Coordinate conversions
	std::pair<int, int> loc2grid(double x, double y) const;
	std::pair<double, double> grid2loc(int row, int col) const;

	// Gradient ascent
	std::pair<int, int> find_next_step(int row, int col) const;

	// Parameters
	// Map
	double map_origin_x_{0.0};
	double map_origin_y_{0.0};
	double map_resolution_{0.05};

	// Runtime
	int step_lookahead_{6};
	double goal_phi_threshold_{100.0};

	// Timer
	rclcpp::TimerBase::SharedPtr timer_;
	double timer_period_s_{0.5};

	// Robot state
	double current_x_{0.0};
	double current_y_{0.0};
	bool has_pose_{false};

	// Potential field
	std::string phi_path_;
	std::vector<std::vector<double>> phi_;
	int phi_rows_{0};
	int phi_cols_{0};

	// Global costmap
	std::string map_file_path_;
};

}  // namespace laplace_pathfinder
