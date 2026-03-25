import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


def annotate_values(ax, data, fontsize=8):
    for i in range(data.shape[0]):
        for j in range(data.shape[1]):
            ax.text(
                j,
                i,
                f"{data[i, j]:.2f}",
                ha="center",
                va="center",
                color="black",
                fontsize=fontsize,
            )


def plot_circles(ax, data, cmap="viridis", s_factor=1000, min_size=10):
    y, x = np.mgrid[0:data.shape[0], 0:data.shape[1]]
    sizes = np.maximum(data.flatten() * s_factor, min_size)
    ax.scatter(x, y, s=sizes, c="white", alpha=0.6)


def vector_stride(shape, target_vectors_per_axis=40):
    return max(1, int(max(shape) / target_vectors_per_axis))


def plot_solution(maze, start, end, phi, path, grad_phi, output_path):
    path_np = np.array(path)
    marker_size = 10
    path_linewidth = 0.5
    arrow_width = 0.006
    arrow_alpha = 1.0
    arrow_len = 6.0

    fig, axs = plt.subplots(2, 2, figsize=(10, 10))

    # [0,0] Original Maze
    axs[0, 0].imshow(maze, cmap="gray_r")
    axs[0, 0].scatter(
        [start[1], end[1]],
        [start[0], end[0]],
        color=["green", "red"],
        s=marker_size,
        zorder=2,
    )
    axs[0, 0].set_title("Original Maze")
    legend_handles = [
        Line2D([0], [0], marker="o", color="w", label="Start", markerfacecolor="green", markersize=6),
        Line2D([0], [0], marker="o", color="w", label="End", markerfacecolor="red", markersize=6),
    ]
    axs[0, 0].axis("off")

    # [0,1] Potential Field Phi
    im_phi = axs[0, 1].imshow(phi, cmap="viridis")
    axs[0, 1].scatter(
        [start[1], end[1]],
        [start[0], end[0]],
        color=["green", "red"],
        s=marker_size,
        zorder=2,
    )
    axs[0, 1].set_title("Potential Field")
    axs[0, 1].axis("off")
    fig.colorbar(im_phi, ax=axs[0, 1], label=r"$\phi$", fraction=0.046, pad=0.04)

    # [1,0] Gradient Field
    axs[1, 0].imshow(phi, cmap="viridis")
    y, x = np.mgrid[0 : phi.shape[0], 0 : phi.shape[1]]
    grad_mag = np.linalg.norm(grad_phi, axis=2)
    grad_x = np.zeros_like(grad_phi[:, :, 0])
    grad_y = np.zeros_like(grad_phi[:, :, 1])
    np.divide(grad_phi[:, :, 0], grad_mag, out=grad_x, where=grad_mag > 0)
    np.divide(grad_phi[:, :, 1], grad_mag, out=grad_y, where=grad_mag > 0)
    grad_x[maze == 1] = 0
    grad_y[maze == 1] = 0
    step_grad = vector_stride(phi.shape)
    grad_mask = (maze == 0) & (grad_mag > 0) & (x % step_grad == 0) & (y % step_grad == 0)
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
        [start[1], end[1]],
        [start[0], end[0]],
        color=["green", "red"],
        s=marker_size,
        zorder=2,
    )
    axs[1, 0].set_title("Gradient Field")
    axs[1, 0].axis("off")

    # [1,1] Path Following
    axs[1, 1].imshow(phi, cmap="viridis")
    if path_np.ndim == 2 and path_np.shape[1] == 2 and len(path_np) > 0:
        axs[1, 1].plot(
            path_np[:, 1],
            path_np[:, 0],
            color="white",
            linewidth=path_linewidth,
            label="Path",
        )
    else:
        axs[1, 1].text(
            0.5,
            0.5,
            "No path found",
            transform=axs[1, 1].transAxes,
            ha="center",
            va="center",
            color="white",
            fontsize=12,
            bbox=dict(facecolor="black", alpha=0.5, edgecolor="none"),
        )
    axs[1, 1].scatter(
        [start[1], end[1]],
        [start[0], end[0]],
        color=["green", "red"],
        s=marker_size,
        zorder=2,
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
    plt.savefig(output_path, dpi=300, bbox_inches="tight")


__all__ = ["annotate_values", "plot_circles", "plot_solution"]
