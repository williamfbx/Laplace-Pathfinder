import os
import importlib.util

import numpy as np
import yaml
from collections import deque

from laplace_solver import calculate_gradient, solve_laplace, find_next_step
from plotter import plot_solution
from visualize import animate_agent_path


def inflate_obstacles(map, buffer_radius):
	if buffer_radius <= 0:
		return map

	occupied = map == 1
	inflated = occupied.copy()
	rows, cols = map.shape

	# Apply circular inflation in grid cells.
	for dr in range(-buffer_radius, buffer_radius + 1):
		for dc in range(-buffer_radius, buffer_radius + 1):
			if dr * dr + dc * dc > buffer_radius * buffer_radius:
				continue

			src_r0 = max(0, -dr)
			src_r1 = min(rows, rows - dr)
			src_c0 = max(0, -dc)
			src_c1 = min(cols, cols - dc)

			dst_r0 = max(0, dr)
			dst_r1 = min(rows, rows + dr)
			dst_c0 = max(0, dc)
			dst_c1 = min(cols, cols + dc)

			inflated[dst_r0:dst_r1, dst_c0:dst_c1] |= occupied[src_r0:src_r1, src_c0:src_c1]

	inflated_map = map.copy()
	inflated_map[inflated] = 1
	return inflated_map


def fill_isolated_pockets(map, min_pocket_size=50):
	rows, cols = map.shape
	free = map == 0
	visited = np.zeros_like(free, dtype=bool)

	neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]
	result_map = map.copy()
	filled_count = 0

	for start_i in range(rows):
		for start_j in range(cols):
			if not free[start_i, start_j] or visited[start_i, start_j]:
				continue

			# BFS to label this connected component
			component = []
			q = deque([(start_i, start_j)])
			visited[start_i, start_j] = True

			while q:
				i, j = q.popleft()
				component.append((i, j))
				for di, dj in neighbors:
					ni, nj = i + di, j + dj
					if (
						0 <= ni < rows and 0 <= nj < cols and
						free[ni, nj] and not visited[ni, nj]
					):
						visited[ni, nj] = True
						q.append((ni, nj))

			# Check if component touches the boundary (potentially connecting to infinity)
			touches_boundary = any(
				(i == 0 or i == rows - 1 or j == 0 or j == cols - 1)
				for i, j in component
			)

			# If isolated and small, fill it with walls
			if not touches_boundary and len(component) < min_pocket_size:
				for i, j in component:
					result_map[i, j] = 1
				filled_count += len(component)

	if filled_count > 0:
		print(f"Filled {filled_count} isolated pocket cells")
	return result_map


def load_pgm_yaml_map(pgm_path, yaml_path, buffer_radius=0):
	with open(yaml_path, "r", encoding="utf-8") as f:
		cfg = yaml.safe_load(f)

	negate = cfg.get("negate", 0)
	occupied_thresh = cfg.get("occupied_thresh", 0.65)
	free_thresh = cfg.get("free_thresh", 0.25)

	# Parse P5 (binary) PGM
	with open(pgm_path, "rb") as f:
		magic = f.readline().decode("ascii").strip()
		assert magic == "P5", f"Expected P5 PGM, got {magic}"

		line = f.readline().decode("ascii").strip()
		while line.startswith("#"):
			line = f.readline().decode("ascii").strip()

		width, height = (int(x) for x in line.split())
		max_val = int(f.readline().decode("ascii").strip())

		dtype = np.uint8 if max_val < 256 else np.uint16
		raw = np.frombuffer(f.read(), dtype=dtype).reshape((height, width))

	pixel_norm = raw.astype(np.float64) / max_val
	if negate:
		occupancy = pixel_norm
	else:
		occupancy = 1.0 - pixel_norm

	map = np.zeros((height, width), dtype=int)
	map[occupancy >= occupied_thresh] = 1
	map[(occupancy >= free_thresh) & (occupancy < occupied_thresh)] = 1

	map = inflate_obstacles(map, buffer_radius)
	map = fill_isolated_pockets(map)

	return map


def world_to_grid(x, y, resolution, origin, height):
	origin_x, origin_y = origin[:2]
	col = int(round((x - origin_x) / resolution))
	row = int(round(height - 1 - ((y - origin_y) / resolution)))
	return row, col


def load_start_end_from_scenario(scenario_path, map_yaml_path, map):
	with open(scenario_path, "r", encoding="utf-8") as f:
		scenario_cfg = yaml.safe_load(f)

	with open(map_yaml_path, "r", encoding="utf-8") as f:
		map_cfg = yaml.safe_load(f)

	turtlebot_cfg = scenario_cfg["entities"]["turtlebot3"]
	start_pose = turtlebot_cfg["start_pose"]
	goal_pose = turtlebot_cfg["goal_pose"]

	resolution = map_cfg["resolution"]
	origin = map_cfg["origin"]
	height = map.shape[0]

	start = world_to_grid(start_pose["x"], start_pose["y"], resolution, origin, height)
	end = world_to_grid(goal_pose["x"], goal_pose["y"], resolution, origin, height)

	for name, cell in (("start", start), ("end", end)):
		row, col = cell
		if not (0 <= row < map.shape[0] and 0 <= col < map.shape[1]):
			raise ValueError(f"{name} pose discretized out of bounds: {cell}")
		if map[row, col] != 0:
			raise ValueError(f"{name} pose discretized to an occupied cell: {cell}")

	return start, end


def save_map_as_python(map, start, end, out_dir, map_name):
	npy_filename = f"{map_name}_map.npy"
	npy_path = os.path.join(out_dir, npy_filename)
	py_path = os.path.join(out_dir, f"{map_name}_map.py")

	np.save(npy_path, map)

	py_content = (
		"import numpy as np\n"
		"import os\n"
		"\n"
		f"map   = np.load(os.path.join(os.path.dirname(__file__), {repr(npy_filename)}))\n"
		f"start = {repr(start)}\n"
		f"end   = {repr(end)}\n"
	)

	with open(py_path, "w", encoding="utf-8") as f:
		f.write(py_content)

	print(f"Saved array : {npy_path}")
	print(f"Saved module: {py_path}")


def load_py_map(map_path):
	module_name = os.path.splitext(os.path.basename(map_path))[0]
	if not os.path.isfile(map_path):
		raise FileNotFoundError(f"Map module file not found: {map_path}")

	spec = importlib.util.spec_from_file_location(module_name, map_path)
	if spec is None or spec.loader is None:
		raise ImportError(f"Unable to load map module from file: {map_path}")

	module = importlib.util.module_from_spec(spec)
	spec.loader.exec_module(module)
	return module.map, module.start, module.end


def save_phi(phi, phi_path):
	os.makedirs(os.path.dirname(phi_path) or ".", exist_ok=True)
	np.save(phi_path, phi)
	print(f"Saved phi to: {phi_path}")
	return phi_path


def run_laplace_stage(map_path, output_dir, visualize):
	print(f"Loading .py file: {map_path}")
	map, start, end = load_py_map(map_path)

	map_base_name = os.path.splitext(os.path.basename(map_path))[0]
	phi_output_path = os.path.join(output_dir, f"{map_base_name}_phi.npy")
	plot_output_path = os.path.join(output_dir, f"{map_base_name}_plot.png")
	animation_output_path = os.path.join(output_dir, f"{map_base_name}_annimation.mp4")

	print(f"Loaded map: {map_base_name}")
	print(f"Start: {start}, End: {end}")

	phi = solve_laplace(map, start, end)
	save_phi(phi, phi_output_path)

	path = [start]
	curr_loc = start
	while curr_loc != end:
		next_loc = find_next_step(phi, curr_loc, map)

		if next_loc is None:
			print(f"No valid next step found from {curr_loc}. Stopping pathfinding.")
			break
		path.append(next_loc)
		curr_loc = next_loc

	grad_phi = calculate_gradient(phi)
	plot_solution(map, start, end, phi, path, grad_phi, plot_output_path)

	if visualize:
		print("Generating animation...")
		animate_agent_path(map, path, start, end, animation_output_path)
		print(f"Saved animation to: {animation_output_path}")


def main(map_stem, pgm_path, map_yaml_path, scenario_path, maps_dir, output_dir, buffer_radius=3, visualize=False):
	map = load_pgm_yaml_map(pgm_path, map_yaml_path, buffer_radius=buffer_radius)
	
	start, end = load_start_end_from_scenario( scenario_path, map_yaml_path, map)

	print(f"Grid shape : {map.shape}")
	print(f"Wall cells : {map.sum()}")
	print(f"Free cells : {(map == 0).sum()}")
	print(f"Start      : {start}")
	print(f"End        : {end}")

	save_map_as_python(map, start, end, out_dir=maps_dir, map_name=map_stem)

	laplace_map_path = os.path.join(maps_dir, f"{map_stem}_map.py")
	run_laplace_stage(map_path=laplace_map_path, output_dir=output_dir, visualize=visualize)


if __name__ == "__main__":
	base_dir = os.path.dirname(__file__)
	package_root = os.path.dirname(base_dir)
	maps_dir = os.path.join(package_root, "maps")
	
    # Change this to match map name in maps/
	map_stem = "bookstore"
	pgm_path = os.path.join(maps_dir, f"{map_stem}.pgm")
	map_yaml_path = os.path.join(maps_dir, f"{map_stem}.yaml")
	scenario_path = os.path.join(package_root, "config", "scenario_config.yaml")
	output_dir = maps_dir

	main(
		map_stem=map_stem,
		pgm_path=pgm_path,
		map_yaml_path=map_yaml_path,
		scenario_path=scenario_path,
		maps_dir=maps_dir,
		output_dir=output_dir,
		buffer_radius=3,
		visualize=False,
	)
