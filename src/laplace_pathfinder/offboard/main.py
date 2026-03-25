import argparse
import importlib
import os
import numpy as np
from laplace_solver import calculate_gradient, solve_laplace, calculate_perturbation, find_next_step
from plotter import plot_solution
from visualize import animate_agent_path

def load_map_map(map_name):
	# Support absolute paths and map paths relative to src/.
	map_path = map_name
	if not os.path.isabs(map_path):
		candidates = [
			map_path,
			os.path.join(os.path.dirname(__file__), map_path),
			os.path.join(os.path.dirname(__file__), "maps", map_path),
		]
		for candidate in candidates:
			if os.path.isfile(candidate):
				map_path = candidate
				break

	if not os.path.isfile(map_path):
		raise FileNotFoundError(f"Map file not found: {map_name}")

	with open(map_path, "r", encoding="utf-8") as f:
		lines = [line.rstrip("\n") for line in f]

	try:
		map_start = lines.index("map") + 1
	except ValueError as exc:
		raise ValueError(f"Invalid .map file format (missing 'map' line): {map_name}") from exc

	map_rows = [row for row in lines[map_start:] if row]
	if not map_rows:
		raise ValueError(f"Invalid .map file format (empty map grid): {map_name}")

	width = len(map_rows[0])
	if any(len(row) != width for row in map_rows):
		raise ValueError(f"Invalid .map file format (inconsistent row widths): {map_name}")

	# MovingAI octile maps: these terrain symbols are traversable for grid agents.
	traversable = {".", "G", "S"}
	maze = np.array(
		[[0 if ch in traversable else 1 for ch in row] for row in map_rows],
		dtype=int,
	)

	h, w = maze.shape
	interior_free = [
		(i, j)
		for i in range(1, h - 1)
		for j in range(1, w - 1)
		if maze[i, j] == 0
	]
	free_cells = interior_free if len(interior_free) >= 2 else list(zip(*np.where(maze == 0)))

	if len(free_cells) < 2:
		raise ValueError(f"Map must contain at least two traversable cells: {map_name}")

	start = tuple(free_cells[0])
	end = tuple(free_cells[-1])
	return maze, start, end


def load_py_map(map_name):
    module = importlib.import_module(f"maps.{map_name}")
    return module.maze, module.start, module.end


def save_phi(phi, phi_path):
	os.makedirs(os.path.dirname(phi_path) or ".", exist_ok=True)
	np.save(phi_path, phi)
	print(f"Saved phi to: {phi_path}")
	return phi_path


def introduce_dynamic_obstacles(maze, dynamic_obstacles_percent, start=None, end=None):
	dynamic_maze = maze.copy()
	if dynamic_obstacles_percent <= 0:
		return dynamic_maze

	free_cells = list(zip(*np.where(dynamic_maze == 0)))
	protected = {tuple(s) for s in filter(None, [start, end])}
	candidates = [c for c in free_cells if c not in protected]
	if not candidates:
		return dynamic_maze

	n = int(round(dynamic_obstacles_percent / 100.0 * len(candidates)))
	n = min(n, len(candidates))
	if n == 0:
		return dynamic_maze

	chosen = np.random.choice(len(candidates), size=n, replace=False)
	for idx in chosen:
		i, j = candidates[idx]
		dynamic_maze[i, j] = 1

	print(f"Introduced {n} dynamic obstacles ({dynamic_obstacles_percent:.1f}% of {len(candidates)} free cells)")
	return dynamic_maze


# DEBUG
# def introduce_dynamic_obstacles(maze, dynamic_obstacles_percent, start=None, end=None):
# 	dynamic_maze = maze.copy()
# 	dynamic_maze[7, 5] = 1
# 	dynamic_maze[6, 5] = 1
# 	dynamic_maze[6, 2] = 1
# 	dynamic_maze[5, 2] = 1
# 	return dynamic_maze


def main(map_name, output_path, max_iter, visualize, r, dynamic_obstacles_percent):
	if map_name.endswith(".map"):
		print(f"Loading .map file: {map_name}")
		maze, start, end = load_map_map(map_name)
	else:
		print(f"Loading .py file: {map_name}")
		maze, start, end = load_py_map(map_name)

	map_base_name = os.path.splitext(os.path.basename(map_name))[0]
	phi_output_path = os.path.join(output_path, f"{map_base_name}_phi.npy")
	plot_output_path = os.path.join(output_path, f"{map_base_name}_plot.png")
	animation_output_path = os.path.join(output_path, f"{map_base_name}_annimation.mp4")

	print(f"Loaded map: {map_name}")
	print(f"Start: {start}, End: {end}")

	phi = solve_laplace(maze, start, end, max_iter=max_iter)
	save_phi(phi, phi_output_path)
 
	dynamic_maze = introduce_dynamic_obstacles(maze, dynamic_obstacles_percent, start=start, end=end)
 
	path = [start]
	curr_loc = start
	while curr_loc != end:
		delta_phi = calculate_perturbation(phi, curr_loc, r, dynamic_maze, maze)
		phi_perturbed = phi + delta_phi
		next_loc = find_next_step(phi_perturbed, curr_loc, dynamic_maze)
		
		if next_loc is None:
			print(f"No valid next step found from {curr_loc}. Stopping pathfinding.")
			break
		path.append(next_loc)
		curr_loc = next_loc
  
	grad_phi = calculate_gradient(phi)
	plot_solution(maze, start, end, phi, path, grad_phi, plot_output_path)
 
	if visualize:
		print("Generating animation...")
		animate_agent_path(maze, path, start, end, animation_output_path)
		print(f"Saved animation to: {animation_output_path}")


if __name__ == "__main__":
	parser = argparse.ArgumentParser(description="Path finding using Laplace's equation")
	parser.add_argument(
		"--map",
		default="bookstore_map",
		help="Map module name in maps/ (default: bookstore_map)",
	)
	parser.add_argument(
		"--output",
		default="maps",
		help="Output directory path (default: maps/)",
	)
	parser.add_argument(
        "--max_iter",
        type=int,
        default=5000,
        help="Maximum iterations for solving Laplace's equation (default: 5000)",
    )
	parser.add_argument(
        "--visualize",
		action="store_true",
		help="Whether to visualize the solution (default: False)",
	)
	parser.add_argument(
		"--r",
		type=int,
		default=3,
		help="Radius for local perturbation (default: 3)",
	)
	parser.add_argument(
		"--dynamic_obstacles_percent",
		type=float,
		default=0.0,
		help="Percentage of dynamic obstacles to introduce (default: 0.0)",
	)

	args = parser.parse_args()

	main(args.map, args.output, args.max_iter, args.visualize, args.r, args.dynamic_obstacles_percent)
