import numpy as np

class SlidingWindow:
    def __init__(self, size):
        self.size = size
        print("The size is: " + str(size))

    def plot(self, plt, path, vehicle, current_idx):
        # outline = np.array([[-self.size, -self.size, self.size, self.size, -self.size],
        #             [-self.size, self.size, self.size, -self.size, -self.size]])
                    
        # plt.plot(np.array(outline[0, :]).flatten(),
        #             np.array(outline[1, :]).flatten())

        if((current_idx + 10) > len(path)):
            dx = [ipath.x for ipath in path[current_idx : len(path)]]
            dy = [ipath.y for ipath in path[current_idx : len(path)]]
        else:
            dx = [ipath.x for ipath in path[current_idx : current_idx + 10*3]]
            dy = [ipath.y for ipath in path[current_idx : current_idx + 10*3]]

        plt.plot(dx, dy, ".b")

        