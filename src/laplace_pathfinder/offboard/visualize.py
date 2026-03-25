import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter, FuncAnimation


def animate_agent_path(maze, path, start, end, output_file, interval_ms=240):
	if path is None or len(path) == 0:
		path = [start]

	path_np = np.array(path, dtype=float)

	fig, ax = plt.subplots(figsize=(8, 8))
	ax.imshow(maze, cmap="gray_r")
	ax.set_title("Agent Motion Animation")
	ax.axis("off")

	ax.scatter(start[1], start[0], c="green", s=100, zorder=3, label="Start")
	ax.scatter(end[1], end[0], c="red", s=100, zorder=3, label="End")

	trail_line, = ax.plot([], [], color="dodgerblue", linewidth=2, zorder=2, label="Path")
	agent_dot = ax.scatter([], [], c="orange", s=120, zorder=4, label="Agent")
	ax.legend(loc="upper right")

	def _init():
		trail_line.set_data([], [])
		agent_dot.set_offsets(np.array([[start[1], start[0]]], dtype=float))
		return trail_line, agent_dot

	def _update(frame_idx):
		sub_path = path_np[: frame_idx + 1]
		trail_line.set_data(sub_path[:, 1], sub_path[:, 0])
		agent_dot.set_offsets(np.array([[sub_path[-1, 1], sub_path[-1, 0]]], dtype=float))
		return trail_line, agent_dot

	anim = FuncAnimation(
		fig,
		_update,
		init_func=_init,
		frames=len(path_np),
		interval=interval_ms,
		blit=True,
		repeat=False,
	)

	os.makedirs(os.path.dirname(output_file) or ".", exist_ok=True)
	anim.save(
		output_file,
		writer=FFMpegWriter(fps=max(1, int(1000 / interval_ms))),
	)
	plt.close(fig)
	return output_file


__all__ = ["animate_agent_path"]