import numpy as np
import scipy.sparse
import scipy.sparse.linalg


# Solve Laplace's equation
def solve_laplace(maze, start, end):
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
    "find_next_step",
    "calculate_gradient",
]
