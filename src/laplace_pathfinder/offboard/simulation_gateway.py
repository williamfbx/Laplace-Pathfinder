import numpy as np
import yaml
import os


def inflate_obstacles(maze, buffer_radius):
    if buffer_radius <= 0:
        return maze

    occupied = maze == 1
    inflated = occupied.copy()
    rows, cols = maze.shape

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

    inflated_maze = maze.copy()
    inflated_maze[inflated] = 1
    return inflated_maze


def fill_isolated_pockets(maze, min_pocket_size=50):
    """
    Identifies small isolated free-cell regions (connected components with fewer
    than min_pocket_size cells that do not touch the boundary) and fills them
    with walls (1). This prevents disconnected components that can cause
    singular Laplace systems while preserving reasonably-sized regions.
    """
    from collections import deque

    rows, cols = maze.shape
    free = maze == 0
    visited = np.zeros_like(free, dtype=bool)

    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    result_maze = maze.copy()
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
                    if (0 <= ni < rows and 0 <= nj < cols and
                        free[ni, nj] and not visited[ni, nj]):
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
                    result_maze[i, j] = 1
                filled_count += len(component)

    if filled_count > 0:
        print(f"Filled {filled_count} isolated pocket cells")
    return result_maze


def load_pgm_yaml_map(pgm_path, yaml_path, buffer_radius=0):

    with open(yaml_path, "r") as f:
        cfg = yaml.safe_load(f)

    negate = cfg.get("negate", 0)
    occupied_thresh = cfg.get("occupied_thresh", 0.65)
    free_thresh = cfg.get("free_thresh", 0.25)

    # --- parse P5 (binary) PGM ---
    with open(pgm_path, "rb") as f:
        # read magic number
        magic = f.readline().decode("ascii").strip()
        assert magic == "P5", f"Expected P5 PGM, got {magic}"

        # skip comment lines
        line = f.readline().decode("ascii").strip()
        while line.startswith("#"):
            line = f.readline().decode("ascii").strip()

        width, height = map(int, line.split())
        max_val = int(f.readline().decode("ascii").strip())

        dtype = np.uint8 if max_val < 256 else np.uint16
        raw = np.frombuffer(f.read(), dtype=dtype).reshape((height, width))

    # --- pixel → occupancy probability (ROS convention) ---
    pixel_norm = raw.astype(np.float64) / max_val   # [0, 1]
    if negate:
        occupancy = pixel_norm
    else:
        occupancy = 1.0 - pixel_norm                # dark pixel → high occupancy

    # --- occupancy → grid (1 = wall, 0 = free, unknown → 1) ---
    maze = np.zeros((height, width), dtype=int)     # default: free
    maze[occupancy >= occupied_thresh] = 1          # occupied cells
    maze[(occupancy >= free_thresh) & (occupancy < occupied_thresh)] = 1  # unknown cells

    maze = inflate_obstacles(maze, buffer_radius)
    maze = fill_isolated_pockets(maze)

    start = None
    end = None

    return maze, start, end


def world_to_grid(x, y, resolution, origin, height):
    origin_x, origin_y = origin[:2]
    col = int(round((x - origin_x) / resolution))
    row = int(round(height - 1 - ((y - origin_y) / resolution)))
    return row, col


def load_start_end_from_scenario(scenario_path, map_yaml_path, maze):
    with open(scenario_path, "r") as f:
        scenario_cfg = yaml.safe_load(f)

    with open(map_yaml_path, "r") as f:
        map_cfg = yaml.safe_load(f)

    turtlebot_cfg = scenario_cfg["entities"]["turtlebot3"]
    start_pose = turtlebot_cfg["start_pose"]
    goal_pose = turtlebot_cfg["goal_pose"]

    resolution = map_cfg["resolution"]
    origin = map_cfg["origin"]
    height = maze.shape[0]

    start = world_to_grid(start_pose["x"], start_pose["y"], resolution, origin, height)
    end = world_to_grid(goal_pose["x"], goal_pose["y"], resolution, origin, height)

    for name, cell in (("start", start), ("end", end)):
        row, col = cell
        if not (0 <= row < maze.shape[0] and 0 <= col < maze.shape[1]):
            raise ValueError(f"{name} pose discretized out of bounds: {cell}")
        if maze[row, col] != 0:
            raise ValueError(f"{name} pose discretized to an occupied cell: {cell}")

    return start, end


def save_map_as_python(maze, start, end, out_dir, map_name):
    npy_filename = f"{map_name}_maze.npy"
    npy_path     = os.path.join(out_dir, npy_filename)
    py_path      = os.path.join(out_dir, f"{map_name}_map.py")

    np.save(npy_path, maze)

    py_content = (
        "import numpy as np\n"
        "import os\n"
        "\n"
        f"maze  = np.load(os.path.join(os.path.dirname(__file__), {repr(npy_filename)}))\n"
        f"start = {repr(start)}\n"
        f"end   = {repr(end)}\n"
    )

    with open(py_path, "w") as f:
        f.write(py_content)

    print(f"Saved array : {npy_path}")
    print(f"Saved module: {py_path}")


if __name__ == "__main__":
    base = os.path.dirname(__file__)
    pgm  = os.path.join(base, "maps", "bookstore.pgm")
    yml  = os.path.join(base, "maps", "bookstore.yaml")
    scenario = os.path.join(base, "maps", "scenario_config.yaml")
    buffer_radius = 3

    maze, start, end = load_pgm_yaml_map(pgm, yml, buffer_radius=buffer_radius)
    start, end = load_start_end_from_scenario(scenario, yml, maze)
    print(f"Grid shape : {maze.shape}")
    print(f"Wall cells : {maze.sum()}")
    print(f"Free cells : {(maze == 0).sum()}")
    print(f"Start      : {start}")
    print(f"End        : {end}")

    save_map_as_python(
        maze, start, end,
        out_dir  = os.path.join(base, "maps"),
        map_name = "bookstore",
    )
