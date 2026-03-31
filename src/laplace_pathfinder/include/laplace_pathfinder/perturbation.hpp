#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace laplace_pathfinder
{

class Perturbation : public rclcpp::Node
{
public:
	explicit Perturbation(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
	enum class SolverMode {
		kPureSor,
		kNnWarmstartSor,
		kNnOnly,
	};

private:
	// Publishers
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_publisher_;
	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr perturbation_field_publisher_;

	// Subscribers
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_subscriber_;

    // Functions
	void publish_global_costmap();
	void compare_costmaps();
	void solve_local_laplace();
	void world_to_phi_grid(double wx, double wy, int & pr, int & pc) const;
	static SolverMode parse_solver_mode(const std::string & mode);
	void run_sor_on_patch(
		std::vector<std::vector<double>> & patch_phi,
		const std::vector<std::vector<bool>> & patch_fixed,
		const std::vector<std::vector<bool>> & patch_wall,
		int max_iters) const;
	bool try_nn_patch_inference(
		std::vector<std::vector<double>> & patch_phi,
		const std::vector<std::vector<bool>> & patch_fixed,
		const std::vector<std::vector<bool>> & patch_wall) const;

	// Callbacks
	void local_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

	// Parameters
	// Global Map
	std::string map_file_path_;
	std::vector<std::vector<int64_t>> map_;
	nav_msgs::msg::OccupancyGrid global_costmap_;
	int map_rows_{0};
	int map_cols_{0};
	double map_origin_x_{0.0};
	double map_origin_y_{0.0};
	double map_resolution_{0.05};

    // Potential field
	std::string phi_file_path_;
	std::vector<std::vector<double>> phi_;
	int phi_rows_{0};
	int phi_cols_{0};

    // Local Map
    nav_msgs::msg::OccupancyGrid local_costmap_;
	bool has_local_costmap_{false};

	// Solver parameters
	std::string solver_mode_str_;
	SolverMode solver_mode_{SolverMode::kPureSor};
	int inflate_radius_{5};
	int sor_max_iters_{2000};
	int nn_warmstart_sor_iters_{40};
	double sor_tolerance_{1e-3};
	double sor_omega_{1.7};
};

}  // namespace laplace_pathfinder
