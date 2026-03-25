import numpy as np
import tqdm
import scipy.sparse
import scipy.sparse.linalg


# Solve Laplace's equation
def solve_laplace(maze, start, end, max_iter=None):
    h, w = maze.shape
    n = h * w
    wall_mask = (maze == 1).ravel()
    flat_idx = np.arange(n).reshape(h, w)

    # Build neighbor pairs for free cells
    def neighbor(di, dj):
        si = flat_idx[max(0,di):h+min(0,di), max(0,dj):w+min(0,dj)]
        ti = flat_idx[max(0,-di):h+min(0,-di), max(0,-dj):w+min(0,-dj)]
        wall_src = (maze == 1)[max(0,di):h+min(0,di), max(0,dj):w+min(0,dj)]
        wall_tgt = (maze == 1)[max(0,-di):h+min(0,-di), max(0,-dj):w+min(0,-dj)]
        valid = ~wall_src.ravel() & ~wall_tgt.ravel()
        return si.ravel()[valid], ti.ravel()[valid]

    pairs = [neighbor(di, dj) for di, dj in [(-1,0),(1,0),(0,-1),(0,1)]]

    # Off-diagonal entries: -1 for each free-neighbor pair
    off_r = np.concatenate([s for s, _ in pairs])
    off_c = np.concatenate([t for _, t in pairs])
    off_v = np.full(len(off_r), -1.0)

    # Diagonal: degree (number of free neighbors) for free cells, 1 for fixed
    degree = np.zeros(n)
    np.add.at(degree, off_r, 1.0)

    free_mask = ~wall_mask
    fixed = np.zeros(n, dtype=bool)
    fixed[start[0] * w + start[1]] = True
    fixed[end[0] * w + end[1]] = True
    update = free_mask & ~fixed

    # For fixed and wall rows: identity row
    diag = np.where(update, degree, 1.0)

    # Remove off-diagonal entries belonging to fixed/wall rows
    keep = update[off_r]
    off_r, off_c, off_v = off_r[keep], off_c[keep], off_v[keep]

    diag_r = np.arange(n)
    all_r = np.concatenate([diag_r, off_r])
    all_c = np.concatenate([diag_r, off_c])
    all_v = np.concatenate([diag, off_v])

    A = scipy.sparse.csr_matrix((all_v, (all_r, all_c)), shape=(n, n))

    b = np.zeros(n)
    b[start[0] * w + start[1]] = -100
    b[end[0] * w + end[1]] = 100

    print("Solving sparse linear system...")
    x = scipy.sparse.linalg.spsolve(A, b)
    phi = x.reshape(h, w)
    phi[maze == 1] = 0.0
    return phi


def solve_laplace_dirichlet(maze, start, end, max_iter=5000):
    phi = np.zeros_like(maze, dtype=float)
    phi[start] = -1e6
    phi[end] = 1e6
    wall_mask = (maze == 1)
    for iter in tqdm.tqdm(range(max_iter), desc="Solving Laplace's equation"):
        old_phi = phi.copy()
        for i in range(1, phi.shape[0] - 1):
            for j in range(1, phi.shape[1] - 1):
                if not wall_mask[i, j] and (i, j) not in [start, end]:
                    phi[i, j] = 0.25 * (
                        phi[i - 1, j] + phi[i + 1, j] + phi[i, j - 1] + phi[i, j + 1]
                    )
        if np.allclose(old_phi, phi, atol=1e-6):
            print("Converged after {} iterations".format(iter + 1))
            break
    return phi


# Find the gradient path
def get_next_positions(phi, current_pos, maze=None, visited=None, atol=1e-6):
    neighbors = [
        (current_pos[0] + i, current_pos[1] + j)
        for i, j in [(-1, 0), (1, 0), (0, -1), (0, 1)]
    ]
    visited = set() if visited is None else set(visited)

    neighbors = [
        pos
        for pos in neighbors
        if 0 <= pos[0] < phi.shape[0]
        and 0 <= pos[1] < phi.shape[1]
        and phi[pos] != 0
        and (maze is None or maze[pos] != 1)
        and pos not in visited
    ]

    if not neighbors:
        return []

    max_phi = max(phi[pos] for pos in neighbors)
    next_positions = sorted(
        [pos for pos in neighbors if np.isclose(phi[pos], max_phi, atol=atol)]
    )

    if len(next_positions) > 1:
        print(
            f"Multiple next steps found at {current_pos} with phi={max_phi:.6f}: {next_positions}"
        )

    return next_positions


def find_gradient_path(phi, start, end, max_steps=10000):
    stack = [(start, [start], 0)]
    paths = []

    while stack:
        current_pos, path, step_count = stack.pop()

        if current_pos == end:
            paths.append(path)
            continue

        if step_count >= max_steps:
            continue

        next_positions = get_next_positions(phi, current_pos, visited=path)

        if not next_positions:
            continue

        for next_pos in reversed(next_positions):
            stack.append((next_pos, path + [next_pos], step_count + 1))

    if not paths:
        return []
    
    print(f"Found {len(paths)} path(s) from start to end. Returning the shortest one.")

    return min(paths, key=len)


# Calculate local perturbation for dynamic obstacles
def calculate_perturbation(phi, current_pos, r, dynamic_maze, maze, n_iters=50):
    h, w = phi.shape
    ci, cj = current_pos

    # Local patch bounds
    i_min = max(ci - r, 0)
    i_max = min(ci + r + 1, h)
    j_min = max(cj - r, 0)
    j_max = min(cj + r + 1, w)

    patch_dynamic = dynamic_maze[i_min:i_max, j_min:j_max]
    patch_maze = maze[i_min:i_max, j_min:j_max]
    new_obs = (patch_dynamic == 1) & (patch_maze == 0)

    if not np.any(new_obs):
        return np.zeros_like(phi)

    patch_phi = phi[i_min:i_max, j_min:j_max].copy()
    ph, pw = patch_phi.shape

    # Dynamic obstacles: impose 0 potential
    patch_phi[new_obs] = 0.0

    # Blocked cells: original walls + dynamic obstacles
    blocked = (patch_maze != 0) | new_obs

    # Patch-edge cells retain original potential
    on_edge = np.zeros((ph, pw), dtype=bool)
    on_edge[0, :] = True
    on_edge[-1, :] = True
    on_edge[:, 0] = True
    on_edge[:, -1] = True

    updatable = (~on_edge) & (~blocked)

    # Jacobi iterations to locally solve Laplace's equation in the patch
    for _ in range(n_iters):
        new_patch = patch_phi.copy()
        for i in range(1, ph - 1):
            for j in range(1, pw - 1):
                if updatable[i, j]:
                    vals = []
                    for di, dj in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                        ni, nj = i + di, j + dj
                        if 0 <= ni < ph and 0 <= nj < pw and not blocked[ni, nj]:
                            vals.append(patch_phi[ni, nj])
                    if vals:
                        new_patch[i, j] = sum(vals) / len(vals)
        patch_phi = new_patch

    perturbation = np.zeros_like(phi)
    perturbation[i_min:i_max, j_min:j_max] = patch_phi - phi[i_min:i_max, j_min:j_max]
    return perturbation


# Find next step based on perturbed phi
def find_next_step(phi_perturbed, current_pos, maze):
    next_positions = get_next_positions(phi_perturbed, current_pos, maze=maze)

    if not next_positions:
        return None

    return next_positions[0]


# Calculate gradient
def calculate_gradient(phi):
    grad_phi = np.zeros((phi.shape[0], phi.shape[1], 2))
    for i in range(1, phi.shape[0] - 1):
        for j in range(1, phi.shape[1] - 1):
            if phi[i, j] != 0:
                grad_phi[i, j, 0] = (phi[i, j + 1] - phi[i, j - 1]) / 2
                grad_phi[i, j, 1] = (phi[i + 1, j] - phi[i - 1, j]) / 2
    return grad_phi


__all__ = [
    "solve_laplace",
    "find_gradient_path",
    "calculate_perturbation",
    "find_next_step",
    "calculate_gradient",
]
