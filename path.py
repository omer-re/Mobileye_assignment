import matplotlib.pyplot as plt
import numpy as np


# linked list of points
class path:
    # list of points
    path_points = [(2, 8), (1, 3), (0, 2), (0, 0), (6, 1), (9, 3), (8, 4), (7, 4), (6, 4)]
    points_counter = 9
    points_done = 0

    def get_path_array(self):
        return np.array(self.path_points)

    def path_add_point(self, x_val, y_val):
        tup = (x_val, y_val)
        self.path.append(tup)
        self.points_counter += 1

    def plot_path(self):
        data = np.array(self.path_points)
        plt.plot(*data.T)
        plt.plot(*data.T, 'ro')
        plt.plot(*data.T, 'b-')
        plt.show()

# path1=path()
# path1.plot_path()
