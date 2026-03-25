import numpy as np
import os

maze  = np.load(os.path.join(os.path.dirname(__file__), 'bookstore_maze.npy'))
start = (55, 78)
end   = (311, 338)
