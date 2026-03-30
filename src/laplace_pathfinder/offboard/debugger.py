import argparse
import os
import sys

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

sys.path.insert(0, os.path.dirname(__file__))
from plotter import vector_stride


def _compute_gradient(phi: np.ndarray) -> np.ndarray:
    grad = np.zeros((*phi.shape, 2))
    # axis=1 → column direction → x
    grad[:, :, 0] = np.gradient(phi, axis=1)
    # axis=0 → row direction → y  (rows increase downward)
    grad[:, :, 1] = np.gradient(phi, axis=0)
    return grad


def _greedy_path(phi: np.ndarray, maze: np.ndarray, start: tuple, end: tuple,
                 max_steps: int = 100_000) -> list:
    rows, cols = phi.shape
    dr = [-1, 1, 0, 0]
    dc = [0, 0, -1, 1]

    path = [start]
    visited = {start}
    r, c = start

    for _ in range(max_steps):
        if (r, c) == end:
            break
        best_val = -np.inf
        best_r, best_c = r, c
        for i in range(4):
            nr, nc = r + dr[i], c + dc[i]
            if 0 <= nr < rows and 0 <= nc < cols and maze[nr, nc] == 0:
                if (nr, nc) not in visited and phi[nr, nc] > best_val:
                    best_val = phi[nr, nc]
                    best_r, best_c = nr, nc
        if (best_r, best_c) == (r, c):
            break
        r, c = best_r, best_c
        visited.add((r, c))
        path.append((r, c))

    return path


def plot_debug(maze: np.ndarray, start: tuple, end: tuple,
               phi_debug: np.ndarray, output_path: str) -> None:
    marker_size = 10
    path_linewidth = 0.5
    arrow_width = 0.006
    arrow_alpha = 1.0
    arrow_len = 6.0

    grad_phi = _compute_gradient(phi_debug)
    path = _greedy_path(phi_debug, maze, start, end)
    path_np = np.array(path)

    fig, axs = plt.subplots(2, 2, figsize=(10, 10))

    legend_handles = [
        Line2D([0], [0], marker="o", color="w", label="Start",
               markerfacecolor="green", markersize=6),
        Line2D([0], [0], marker="o", color="w", label="End",
               markerfacecolor="red", markersize=6),
    ]

    axs[0, 0].imshow(maze, cmap="gray_r")
    axs[0, 0].scatter(
        [start[1], end[1]], [start[0], end[0]],
        color=["green", "red"], s=marker_size, zorder=2,
    )
    axs[0, 0].set_title("Original Maze")
    axs[0, 0].axis("off")

    im_phi = axs[0, 1].imshow(phi_debug, cmap="viridis")
    axs[0, 1].scatter(
        [start[1], end[1]], [start[0], end[0]],
        color=["green", "red"], s=marker_size, zorder=2,
    )
    axs[0, 1].set_title("Potential Field (phi + delta_phi)")
    axs[0, 1].axis("off")
    fig.colorbar(im_phi, ax=axs[0, 1], label=r"$\phi + \Delta\phi$",
                 fraction=0.046, pad=0.04)

    axs[1, 0].imshow(phi_debug, cmap="viridis")
    y, x = np.mgrid[0:phi_debug.shape[0], 0:phi_debug.shape[1]]
    grad_mag = np.linalg.norm(grad_phi, axis=2)
    grad_x = np.zeros_like(grad_phi[:, :, 0])
    grad_y = np.zeros_like(grad_phi[:, :, 1])
    np.divide(grad_phi[:, :, 0], grad_mag, out=grad_x, where=grad_mag > 0)
    np.divide(grad_phi[:, :, 1], grad_mag, out=grad_y, where=grad_mag > 0)
    grad_x[maze == 1] = 0
    grad_y[maze == 1] = 0
    step_grad = vector_stride(phi_debug.shape)
    grad_mask = (
        (maze == 0) & (grad_mag > 0) &
        (x % step_grad == 0) & (y % step_grad == 0)
    )
    axs[1, 0].quiver(
        x[grad_mask],
        y[grad_mask],
        grad_x[grad_mask] * arrow_len,
        grad_y[grad_mask] * arrow_len,
        color="w",
        alpha=arrow_alpha,
        angles="xy",
        scale_units="xy",
        scale=1,
        width=arrow_width,
        headwidth=10,
        headlength=12,
        headaxislength=10,
        pivot="tail",
        minlength=1,
        linewidths=0.5,
        edgecolor="k",
        zorder=3,
    )
    axs[1, 0].scatter(
        [start[1], end[1]], [start[0], end[0]],
        color=["green", "red"], s=marker_size, zorder=2,
    )
    axs[1, 0].set_title("Gradient Flow")
    axs[1, 0].axis("off")

    axs[1, 1].imshow(phi_debug, cmap="viridis")
    if path_np.ndim == 2 and path_np.shape[1] == 2 and len(path_np) > 0:
        axs[1, 1].plot(
            path_np[:, 1], path_np[:, 0],
            color="white", linewidth=path_linewidth, label="Path",
        )
    else:
        axs[1, 1].text(
            0.5, 0.5, "No path found",
            transform=axs[1, 1].transAxes,
            ha="center", va="center", color="white", fontsize=12,
            bbox=dict(facecolor="black", alpha=0.5, edgecolor="none"),
        )
    axs[1, 1].scatter(
        [start[1], end[1]], [start[0], end[0]],
        color=["green", "red"], s=marker_size, zorder=2,
    )
    axs[1, 1].set_title("Path Following")
    axs[1, 1].axis("off")

    fig.legend(
        handles=legend_handles,
        loc="lower center",
        ncol=2,
        frameon=True,
        bbox_to_anchor=(0.5, 0.01),
    )

    plt.tight_layout(rect=(0, 0.05, 1, 1))
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    plt.savefig(output_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved debug plot to {output_path}")


def _parse_args() -> argparse.Namespace:
    maps_dir = os.path.join(os.path.dirname(__file__), "..", "maps")
    p = argparse.ArgumentParser(description="Visualize phi_debug.npy")
    p.add_argument("--phi",   default=os.path.join(maps_dir, "phi_debug.npy"),
                   help="Path to phi_debug.npy (phi + delta_phi)")
    p.add_argument("--map",   default=os.path.join(maps_dir, "bookstore_map.npy"),
                   help="Path to maze .npy (0=free, 1=wall)")
    p.add_argument("--start", nargs=2, type=int, metavar=("ROW", "COL"),
                   default=[55, 78])
    p.add_argument("--end",   nargs=2, type=int, metavar=("ROW", "COL"),
                   default=[311, 338])
    p.add_argument("--out",   default=os.path.join(maps_dir, "phi_debug_plot.png"),
                   help="Output image path")
    return p.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    phi_debug = np.load(args.phi)
    maze = np.load(args.map)
    plot_debug(maze, tuple(args.start), tuple(args.end), phi_debug, args.out)
