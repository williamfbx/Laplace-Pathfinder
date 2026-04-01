#include "laplace_pathfinder/perturbation.hpp"
#include "laplace_pathfinder/utils.hpp"

#include <std_msgs/msg/float32_multi_array.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>

#include <torch/script.h>

namespace laplace_pathfinder
{

Perturbation::Perturbation(const rclcpp::NodeOptions & options) : Node("perturbation", options)
{

    // Parameters
	map_origin_x_ = declare_parameter<double>("map_origin_x", 0.0);
	map_origin_y_ = declare_parameter<double>("map_origin_y", 0.0);
	map_resolution_ = declare_parameter<double>("map_resolution", 0.05);

	map_file_path_ = declare_parameter<std::string>("map_file_path", "");
	if (map_file_path_.empty()) {
		throw std::runtime_error("map_file_path is not set.");
	}

	phi_file_path_ = declare_parameter<std::string>("phi_file_path", "");
	if (phi_file_path_.empty()) {
		throw std::runtime_error("phi_file_path is not set.");
	}

	nn_model_path_ = declare_parameter<std::string>("nn_model_path", "");
	if (nn_model_path_.empty()) {
		RCLCPP_WARN(get_logger(), "nn_model_path is not set.");
	}

	solver_mode_str_ = declare_parameter<std::string>("solver_mode", "pure_sor");
	inflate_radius_ = declare_parameter<int>("inflate_radius", 5);
	sor_max_iters_ = declare_parameter<int>("sor_max_iters", 2000);
	sor_tolerance_ = declare_parameter<double>("sor_tolerance", 1e-3);
	sor_omega_ = declare_parameter<double>("sor_omega", 1.7);
	nn_warmstart_sor_iters_ = declare_parameter<int>("nn_warmstart_sor_iters", 40);

    // Publisher
	global_costmap_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
		"/global_costmap", rclcpp::QoS(1).transient_local());

	perturbation_field_publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>(
		"/perturbation_field", rclcpp::QoS(1));

	// Subscriber
	local_costmap_subscriber_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
		"/local_costmap", rclcpp::QoS(1),
		std::bind(&Perturbation::local_costmap_callback, this, std::placeholders::_1));

	publish_global_costmap();

	// Load solver mode
	solver_mode_ = parse_solver_mode(solver_mode_str_);
	RCLCPP_INFO(
		get_logger(),
		"Perturbation solver mode: %s",
		solver_mode_str_.c_str());

	// Load potential field
	phi_ = laplace_pathfinder::load_npy_float64(phi_file_path_, phi_rows_, phi_cols_);
	RCLCPP_INFO(
		get_logger(),
		"Loaded phi field: %d rows x %d cols from %s",
		phi_rows_, phi_cols_, phi_file_path_.c_str());

	// Load NN model
	if (solver_mode_ != SolverMode::kPureSor) {
		if (!ensure_torch_model_ready()) {
			throw std::runtime_error("Failed to load Torch model for NN solver mode.");
		}
	}
}


void Perturbation::publish_global_costmap()
{
	map_ = laplace_pathfinder::load_npy_int64(map_file_path_, map_rows_, map_cols_);

	RCLCPP_INFO(
		get_logger(),
		"Loaded map: %d rows x %d cols from %s",
		map_rows_, map_cols_, map_file_path_.c_str());

	global_costmap_.header.stamp = now();
	global_costmap_.header.frame_id = "odom";

	global_costmap_.info.resolution = map_resolution_;
	global_costmap_.info.width = static_cast<uint32_t>(map_cols_);
	global_costmap_.info.height = static_cast<uint32_t>(map_rows_);
	global_costmap_.info.origin.position.x = map_origin_x_;
	global_costmap_.info.origin.position.y = map_origin_y_;
	global_costmap_.info.origin.position.z = 0.0;
	global_costmap_.info.origin.orientation.w = 1.0;

	global_costmap_.data.resize(static_cast<std::size_t>(map_rows_) * map_cols_);
	for (int r = 0; r < map_rows_; ++r) {
		for (int c = 0; c < map_cols_; ++c) {
			// OccupancyGrid row 0 = bottom of world (lowest y).
			// .npy row 0 = top of image (highest y). Flip rows.
			const int npy_row = (map_rows_ - 1) - r;
			const std::size_t idx = static_cast<std::size_t>(r) * map_cols_ + c;
			global_costmap_.data[idx] = (map_[npy_row][c] != 0) ? 100 : 0;
		}
	}

	global_costmap_publisher_->publish(global_costmap_);
	RCLCPP_INFO(get_logger(), "Published global costmap.");
}


void Perturbation::local_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
	local_costmap_ = *msg;
	has_local_costmap_ = true;
	compare_costmaps();
}


void Perturbation::compare_costmaps()
{
	// Both maps must be in odom frame
	const double lox = local_costmap_.info.origin.position.x;
	const double loy = local_costmap_.info.origin.position.y;
	const double lres = local_costmap_.info.resolution;
	const int lw = static_cast<int>(local_costmap_.info.width);
	const int lh = static_cast<int>(local_costmap_.info.height);

	const double gox = global_costmap_.info.origin.position.x;
	const double goy = global_costmap_.info.origin.position.y;
	const double gres = global_costmap_.info.resolution;
	const int gw = static_cast<int>(global_costmap_.info.width);
	const int gh = static_cast<int>(global_costmap_.info.height);

	bool all_within_global = true;

	for (int r = 0; r < lh; ++r) {
		for (int c = 0; c < lw; ++c) {
			const std::size_t local_idx = static_cast<std::size_t>(r) * lw + c;
			if (local_costmap_.data[local_idx] != 100) {
				continue;
			}

			// Convert local cell centre to world coordinates
			const double wx = lox + (c + 0.5) * lres;
			const double wy = loy + (r + 0.5) * lres;

			// Convert world coordinates to global cell
			const int gc = static_cast<int>(std::floor((wx - gox) / gres));
			const int gr = static_cast<int>(std::floor((wy - goy) / gres));

			if (gc < 0 || gc >= gw || gr < 0 || gr >= gh) {
				all_within_global = false;
				break;
			}

			const std::size_t global_idx = static_cast<std::size_t>(gr) * gw + gc;
			if (global_costmap_.data[global_idx] != 100) {
				all_within_global = false;
				break;
			}
		}
		if (!all_within_global) { break; }
	}

	if (all_within_global) {
		// Send a perturbation field with all zeros
		// Layout: [origin_x, origin_y, resolution, width, height, delta_0, delta_1, ...]
		std_msgs::msg::Float32MultiArray pf;
		pf.data.reserve(5 + static_cast<std::size_t>(lw) * lh);
		pf.data.push_back(static_cast<float>(lox));
		pf.data.push_back(static_cast<float>(loy));
		pf.data.push_back(static_cast<float>(lres));
		pf.data.push_back(static_cast<float>(lw));
		pf.data.push_back(static_cast<float>(lh));
		pf.data.insert(pf.data.end(), static_cast<std::size_t>(lw) * lh, 0.0f);
		perturbation_field_publisher_->publish(pf);
		RCLCPP_INFO(get_logger(), "Found no new obstacles. Published zero perturbation field.");
	} else {
		solve_local_laplace();
	}
}


void Perturbation::solve_local_laplace()
{
	const double lox = local_costmap_.info.origin.position.x;
	const double loy = local_costmap_.info.origin.position.y;
	const double lres = local_costmap_.info.resolution;
	const int lw = static_cast<int>(local_costmap_.info.width);
	const int lh = static_cast<int>(local_costmap_.info.height);

	// Map the four corners of the local costmap to phi_ grid coordinates
	int pr_corners[4], pc_corners[4];
	world_to_phi_grid(lox, loy, pr_corners[0], pc_corners[0]);
	world_to_phi_grid(lox + lw * lres, loy, pr_corners[1], pc_corners[1]);
	world_to_phi_grid(lox, loy + lh * lres, pr_corners[2], pc_corners[2]);
	world_to_phi_grid(lox + lw * lres, loy + lh * lres, pr_corners[3], pc_corners[3]);

	const int pr_min = *std::min_element(pr_corners, pr_corners + 4);
	const int pr_max = *std::max_element(pr_corners, pr_corners + 4);
	const int pc_min = *std::min_element(pc_corners, pc_corners + 4);
	const int pc_max = *std::max_element(pc_corners, pc_corners + 4);

	// Patch spans [patch_r0, patch_r1] x [patch_c0, patch_c1]
	const int patch_r0 = std::max(0, pr_min);
	const int patch_r1 = std::min(phi_rows_ - 1, pr_max);
	const int patch_c0 = std::max(0, pc_min);
	const int patch_c1 = std::min(phi_cols_ - 1, pc_max);

	const int ph = patch_r1 - patch_r0 + 1;
	const int pw = patch_c1 - patch_c0 + 1;

	if (ph <= 2 || pw <= 2) {
		return;
	}

	// Initialise patch arrays from the global phi_ field.
	std::vector<std::vector<double>> patch_phi(ph, std::vector<double>(pw));
	std::vector<std::vector<bool>> patch_fixed(ph, std::vector<bool>(pw, false));
	std::vector<std::vector<bool>> patch_wall(ph, std::vector<bool>(pw, false));

	for (int pr = 0; pr < ph; ++pr) {
		for (int pc = 0; pc < pw; ++pc) {
			const int gr = patch_r0 + pr;
			const int gc = patch_c0 + pc;
			patch_phi[pr][pc] = phi_[gr][gc];

			// Obstacles
			if (map_[gr][gc] != 0) {
				patch_phi[pr][pc] = 0.0;
				patch_fixed[pr][pc] = true;
				patch_wall[pr][pc] = true;
			}
			// Perimeter
			if (pr == 0 || pr == ph - 1 || pc == 0 || pc == pw - 1) {
				patch_fixed[pr][pc] = true;
			}
		}
	}

	// Overlay new obstacles from the local costmap
	std::vector<std::pair<int,int>> new_obstacle_cells;
	for (int lr = 0; lr < lh; ++lr) {
		for (int lc = 0; lc < lw; ++lc) {
			const std::size_t local_idx = static_cast<std::size_t>(lr) * lw + lc;

			// Local costmap obstacles have value 100 in OccupancyGrid
			if (local_costmap_.data[local_idx] != 100) {
				continue;
			}

			// World coordinates
			const double wx = lox + (lc + 0.5) * lres;
			const double wy = loy + (lr + 0.5) * lres;

			// Phi grid cell
			int phi_r_cell, phi_c_cell;
			world_to_phi_grid(wx, wy, phi_r_cell, phi_c_cell);

			if (phi_r_cell < 0 || phi_r_cell >= phi_rows_ ||
				phi_c_cell < 0 || phi_c_cell >= phi_cols_) {
				continue;
			}

			// Already a global obstacle
			if (map_[phi_r_cell][phi_c_cell] != 0) {
				continue;
			}

			// Patch grid cell
			const int pr = phi_r_cell - patch_r0;
			const int pc = phi_c_cell - patch_c0;

			// Skip perimeter
			if (pr <= 0 || pr >= ph - 1 || pc <= 0 || pc >= pw - 1) {
				continue;
			}

			patch_phi[pr][pc] = 0.0;
			patch_fixed[pr][pc] = true;
			patch_wall[pr][pc] = true;
			new_obstacle_cells.emplace_back(pr, pc);
		}
	}

	// Inflate new obstacles with buffer radius
	const int INFLATE_RADIUS = inflate_radius_;

	std::vector<std::vector<bool>> is_inflated(ph, std::vector<bool>(pw, false));

	for (const auto & [or_, oc_] : new_obstacle_cells) {
		for (int dr = -INFLATE_RADIUS; dr <= INFLATE_RADIUS; ++dr) {
			for (int dc = -INFLATE_RADIUS; dc <= INFLATE_RADIUS; ++dc) {

				// Skip cells outside inflation radius
				if (dr * dr + dc * dc > INFLATE_RADIUS * INFLATE_RADIUS) {
					continue;
				}

				const int ip = or_ + dr;
				const int ic = oc_ + dc;
				// Outside patch bounds
				if (ip <= 0 || ip >= ph - 1 || ic <= 0 || ic >= pw - 1) {
					continue;
				}
				// Obstacle
				if (map_[patch_r0 + ip][patch_c0 + ic] != 0) {
					continue;
				}

				is_inflated[ip][ic] = true;
				patch_phi[ip][ic] = 0.0;
				patch_fixed[ip][ic] = true;
				patch_wall[ip][ic] = true;
			}
		}
	}

	// Solve Laplace's equation using SOR
	bool patch_already_perturbation = false;
	if (solver_mode_ == SolverMode::kPureSor) {
		run_sor_on_patch(patch_phi, patch_fixed, patch_wall, sor_max_iters_);
	}
	
	// Solve Laplace's equation using NN warm-start + SOR refinement
	else if (solver_mode_ == SolverMode::kNnWarmstartSor) {
		const bool nn_ok = try_nn_patch_inference(patch_phi, patch_fixed, patch_wall);

		if (!nn_ok) {
			RCLCPP_WARN(get_logger(), "NN warm-start unavailable. Falling back to pure SOR.");
			run_sor_on_patch(patch_phi, patch_fixed, patch_wall, sor_max_iters_);
		} else {
			for (int pr = 0; pr < ph; ++pr) {
				for (int pc = 0; pc < pw; ++pc) {
					if (patch_wall[pr][pc]) {
						continue;
					}
					const int gr = patch_r0 + pr;
					const int gc = patch_c0 + pc;
					patch_phi[pr][pc] += phi_[gr][gc];
				}
			}
			run_sor_on_patch(patch_phi, patch_fixed, patch_wall, nn_warmstart_sor_iters_);
		}
	}
	
	// Solve Laplace's equation using NN only
	else if (solver_mode_ == SolverMode::kNnOnly) {
		const bool nn_ok = try_nn_patch_inference(patch_phi, patch_fixed, patch_wall);

		if (!nn_ok) {
			RCLCPP_WARN(get_logger(), "NN inference failed. Falling back to pure SOR.");
			run_sor_on_patch(patch_phi, patch_fixed, patch_wall, sor_max_iters_);
		} else {
			patch_already_perturbation = true;
			RCLCPP_INFO(get_logger(), "Used NN-only inference for perturbation patch.");
		}
	}

	// Layout: [origin_x, origin_y, resolution, width, height, delta_0, ...]
	const double patch_world_ox = map_origin_x_ + patch_c0 * map_resolution_;
	const double patch_world_oy = map_origin_y_ + (static_cast<double>(phi_rows_ - 1) - patch_r1) * map_resolution_;

	std_msgs::msg::Float32MultiArray pf;
	pf.data.reserve(5 + static_cast<std::size_t>(ph) * pw);
	pf.data.push_back(static_cast<float>(patch_world_ox));
	pf.data.push_back(static_cast<float>(patch_world_oy));
	pf.data.push_back(static_cast<float>(map_resolution_));
	pf.data.push_back(static_cast<float>(pw));
	pf.data.push_back(static_cast<float>(ph));
	pf.data.insert(pf.data.end(), static_cast<std::size_t>(ph) * pw, 0.0f);
	prepare_perturbation_field_data(
		pf,
		patch_phi,
		patch_fixed,
		patch_wall,
		patch_r0,
		patch_r1,
		patch_c0,
		ph,
		pw,
		patch_already_perturbation);

	perturbation_field_publisher_->publish(pf);
	RCLCPP_INFO(get_logger(), "New obstacles detected. Published perturbation field.");
}


Perturbation::SolverMode Perturbation::parse_solver_mode(const std::string & mode)
{
	if (mode == "pure_sor") {
		return Perturbation::SolverMode::kPureSor;
	}
	if (mode == "nn_warmstart_sor") {
		return Perturbation::SolverMode::kNnWarmstartSor;
	}
	if (mode == "nn_only") {
		return Perturbation::SolverMode::kNnOnly;
	}
	return Perturbation::SolverMode::kPureSor;
}


void Perturbation::prepare_perturbation_field_data(
	std_msgs::msg::Float32MultiArray & pf,
	const std::vector<std::vector<double>> & patch_phi,
	const std::vector<std::vector<bool>> & patch_fixed,
	const std::vector<std::vector<bool>> & patch_wall,
	int patch_r0,
	int patch_r1,
	int patch_c0,
	int ph,
	int pw,
	bool already_perturbation) const
{
	for (int mr = 0; mr < ph; ++mr) {
		const int phi_r = patch_r1 - mr;
		for (int mc = 0; mc < pw; ++mc) {
			const int phi_c = patch_c0 + mc;
			const int pr = phi_r - patch_r0;
			const int pc = phi_c - patch_c0;
			const std::size_t data_idx = 5 + static_cast<std::size_t>(mr) * pw + mc;

			if (patch_wall[pr][pc]) {
				// Dynamic obstacles have delta = -phi_ so the total potential becomes zero.
				if (map_[phi_r][phi_c] == 0) {
					pf.data[data_idx] = static_cast<float>(-phi_[phi_r][phi_c]);
				}
				continue;
			}

			if (already_perturbation) {
				if (patch_fixed[pr][pc]) {
					pf.data[data_idx] = 0.0f;
				} else {
					pf.data[data_idx] = static_cast<float>(patch_phi[pr][pc]);
				}
				continue;
			}

			pf.data[data_idx] = static_cast<float>(patch_phi[pr][pc] - phi_[phi_r][phi_c]);
		}
	}
}


void Perturbation::run_sor_on_patch(
	std::vector<std::vector<double>> & patch_phi,
	const std::vector<std::vector<bool>> & patch_fixed,
	const std::vector<std::vector<bool>> & patch_wall,
	int max_iters) const
{
	const int ph = static_cast<int>(patch_phi.size());
	if (ph == 0) {
		return;
	}
	const int pw = static_cast<int>(patch_phi[0].size());
	constexpr int kDR[4] = {-1, 1, 0, 0};
	constexpr int kDC[4] = {0, 0, -1, 1};

	for (int iter = 0; iter < max_iters; ++iter) {
		double max_change = 0.0;
		for (int pr = 1; pr < ph - 1; ++pr) {
			for (int pc = 1; pc < pw - 1; ++pc) {

				// Fixed cell
				if (patch_fixed[pr][pc]) {
					continue;
				}

				double sum = 0.0;
				int cnt = 0;
				for (int k = 0; k < 4; ++k) {
					const int nr = pr + kDR[k];
					const int nc = pc + kDC[k];
					if (patch_wall[nr][nc]) {
						continue;
					}
					sum += patch_phi[nr][nc];
					++cnt;
				}
				if (cnt == 0) {
					continue;
				}

				const double gs_val = sum / cnt;
				const double new_val = (1.0 - sor_omega_) * patch_phi[pr][pc] + sor_omega_ * gs_val;
				max_change = std::max(max_change, std::abs(new_val - patch_phi[pr][pc]));
				patch_phi[pr][pc] = new_val;
			}
		}
		if (max_change < sor_tolerance_) {
			const int current_iters = iter + 1;
			++sor_solve_count_;
			sor_total_iterations_ += static_cast<std::uint64_t>(current_iters);
			const double avg_iters = static_cast<double>(sor_total_iterations_) /
				static_cast<double>(sor_solve_count_);
			RCLCPP_INFO(
				get_logger(),
				"SOR converged in %d iterations (running avg %.2f over %lu solves).",
				current_iters,
				avg_iters,
				static_cast<unsigned long>(sor_solve_count_));
			return;
		}
	}

	++sor_solve_count_;
	sor_total_iterations_ += static_cast<std::uint64_t>(max_iters);
	const double avg_iters = static_cast<double>(sor_total_iterations_) /
		static_cast<double>(sor_solve_count_);
	RCLCPP_WARN(
		get_logger(),
		"SOR reached max iterations (%d) without convergence (running avg %.2f over %lu solves).",
		max_iters,
		avg_iters,
		static_cast<unsigned long>(sor_solve_count_));
}


bool Perturbation::ensure_torch_model_ready() const
{
	if (nn_model_ready_) {
		return true;
	}
	if (nn_model_load_attempted_) {
		return false;
	}
	nn_model_load_attempted_ = true;

	if (nn_model_path_.empty()) {
		RCLCPP_ERROR(get_logger(), "Model path is empty. Cannot run NN inference.");
		return false;
	}

	try {
		nn_module_ = std::make_unique<torch::jit::script::Module>(torch::jit::load(nn_model_path_));
		nn_module_->to(torch::kCPU);
		RCLCPP_INFO(get_logger(), "Loaded torch model on CPU: %s", nn_model_path_.c_str());

		nn_module_->eval();
		nn_model_ready_ = true;
		return true;
	}
	catch (const c10::Error & e) {
		RCLCPP_ERROR(get_logger(), "Failed to load torch model '%s': %s", nn_model_path_.c_str(), e.what());
		return false;

	}
	catch (const std::exception & e) {
		RCLCPP_ERROR(get_logger(), "Torch model init failed: %s", e.what());
		return false;
	}
}


bool Perturbation::try_nn_patch_inference(
	std::vector<std::vector<double>> & patch_phi,
	const std::vector<std::vector<bool>> & patch_fixed,
	const std::vector<std::vector<bool>> & patch_wall) const
{
	if (!ensure_torch_model_ready()) {
		return false;
	}

	const int ph = static_cast<int>(patch_phi.size());
	if (ph == 0) {
		return false;
	}
	const int pw = static_cast<int>(patch_phi[0].size());

	std::vector<float> input_data(static_cast<std::size_t>(3) * ph * pw, 0.0f);
	auto input_at = [&](int ch, int r, int c) -> float & {
		const std::size_t idx = static_cast<std::size_t>(ch) * ph * pw + static_cast<std::size_t>(r) * pw + c;
		return input_data[idx];
	};

	for (int r = 0; r < ph; ++r) {
		for (int c = 0; c < pw; ++c) {
			input_at(0, r, c) = static_cast<float>(patch_phi[r][c]);
			input_at(1, r, c) = patch_fixed[r][c] ? 1.0f : 0.0f;
			input_at(2, r, c) = patch_wall[r][c] ? 1.0f : 0.0f;
		}
	}

	try {
		torch::Tensor input = torch::from_blob(
			input_data.data(),
			{1, 3, ph, pw},
			torch::TensorOptions().dtype(torch::kFloat32)).clone();
		input = input.to(torch::kCPU);

		std::vector<torch::jit::IValue> torch_inputs;
		torch_inputs.emplace_back(input);

		torch::Tensor output = nn_module_->forward(torch_inputs).toTensor();
		output = output.to(torch::kCPU);

		if (output.dim() != 4 || output.size(0) != 1 || output.size(1) != 1) {
			RCLCPP_ERROR(
				get_logger(),
				"Torch model output must be shaped [1, 1, H, W], got rank=%ld with dims=[%ld, %ld, %ld, %ld].",
				output.dim(),
				output.dim() > 0 ? output.size(0) : -1,
				output.dim() > 1 ? output.size(1) : -1,
				output.dim() > 2 ? output.size(2) : -1,
				output.dim() > 3 ? output.size(3) : -1);
			return false;
		}

		if (output.size(2) != ph || output.size(3) != pw) {
			RCLCPP_ERROR(
				get_logger(),
				"Torch model output spatial shape [%ld, %ld] does not match patch [%d, %d].",
				output.size(2),
				output.size(3),
				ph,
				pw);
			return false;
		}

		output = output[0][0];

		output = output.contiguous();
		const float * out_ptr = output.data_ptr<float>();
		for (int r = 0; r < ph; ++r) {
			for (int c = 0; c < pw; ++c) {
				if (patch_fixed[r][c]) {
					patch_phi[r][c] = 0.0;
					continue;
				}
				const std::size_t idx = static_cast<std::size_t>(r) * pw + c;
				const double value = static_cast<double>(out_ptr[idx]);
				if (!std::isfinite(value)) {
					RCLCPP_ERROR(get_logger(), "Torch output contains non-finite values.");
					return false;
				}
				patch_phi[r][c] = value;
			}
		}

		return true;
	} catch (const c10::Error & e) {
		RCLCPP_ERROR(get_logger(), "Torch forward pass failed: %s", e.what());
		return false;
	} catch (const std::exception & e) {
		RCLCPP_ERROR(get_logger(), "Torch inference exception: %s", e.what());
		return false;
	}
}


void Perturbation::world_to_phi_grid(double wx, double wy, int & pr, int & pc) const
{
	pc = static_cast<int>(std::round((wx - map_origin_x_) / map_resolution_));
	pr = static_cast<int>(std::round(static_cast<double>(phi_rows_ - 1) - (wy - map_origin_y_) / map_resolution_));
}

}  // namespace laplace_pathfinder
