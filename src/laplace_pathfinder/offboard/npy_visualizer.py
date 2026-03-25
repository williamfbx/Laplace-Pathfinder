import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def visualize_npy_map(npy_path, title=None, save_path=None):
    maze = np.load(npy_path)

    if maze.ndim != 2:
        raise ValueError(f"Expected a 2-D array, got shape {maze.shape}")

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(maze, cmap="gray_r")
    ax.set_title(title or Path(npy_path).stem)
    ax.set_xlabel("Column")
    ax.set_ylabel("Row")

    free_cells = int(np.count_nonzero(maze == 0))
    occupied_cells = int(np.count_nonzero(maze != 0))
    print(f"Loaded: {npy_path}")
    print(f"Shape: {maze.shape}")
    print(f"Free cells: {free_cells}")
    print(f"Occupied cells: {occupied_cells}")

    fig.tight_layout()

    if save_path:
        output_path = Path(save_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=200, bbox_inches="tight")
        print(f"Saved figure to: {output_path}")
        plt.close(fig)
        return output_path

    plt.show()
    return None


def main():
    parser = argparse.ArgumentParser(description="Visualize a maze stored in a .npy file.")
    parser.add_argument(
        "npy_path",
        nargs="?",
        default=str(Path(__file__).with_name("maps") / "bookstore_maze.npy"),
        help="Path to the .npy maze file.",
    )
    parser.add_argument("--title", help="Optional plot title.")
    parser.add_argument("--save", default=str(Path(__file__).with_name("results") / "bookstore_maze.png"), help="Optional path to save the rendered figure.")
    args = parser.parse_args()

    visualize_npy_map(args.npy_path, title=args.title, save_path=args.save)


if __name__ == "__main__":
    main()
