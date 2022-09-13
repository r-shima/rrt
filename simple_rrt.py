import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import numpy as np
import math

class RRT:
    """Implementing the Rapidly-Exploring Random Tree algorithm"""

    def __init__(self, q_init, iterations):
        self.q_init = q_init
        self.iterations = iterations
        self.delta = 1
        self.domain = 100
        self.q_list = [q_init]
    
    def generate_random_config(self):
        """Generate a random position in the 100x100 domain"""
        self.q_rand = [self.domain * np.random.rand(), self.domain * np.random.rand()]
        return self.q_rand

    def find_nearest_vertex(self):
        """Find the nearest vertex from a random position in q_list"""
        dist_list = []
        random_vertex = self.generate_random_config()
        for vertex in self.q_list:
            distance = math.dist(vertex, random_vertex)
            dist_list.append(distance)
        nearest_dist = min(dist_list)
        nearest_index = dist_list.index(nearest_dist)
        self.q_near = self.q_list[nearest_index]
        return self.q_near

    def generate_new_config(self):
        """Generate a new vertex"""
        # magnitude = math.dist(self.q_rand, self.q_near)
        # x = self.q_rand[0] / magnitude
        # y = self.q_rand[1] / magnitude
        # unit_vector = [x, y]
        # q_new_x = self.q_near[0] + unit_vector[0]
        # q_new_y = self.q_near[1] + unit_vector[1]
        # self.q_new = [q_new_x, q_new_y]
        
        distance = math.dist(self.q_rand, self.q_near)
        x_diff = self.q_rand[0] - self.q_near[0]
        y_diff = self.q_rand[1] - self.q_near[1]
        q_new_x = self.q_near[0] + (self.delta / distance) * x_diff
        q_new_y = self.q_near[1] + (self.delta / distance) * y_diff
        self.q_new = [q_new_x, q_new_y]
        return self.q_new

    def plot_result(self):
        """Plot the tree"""
        f, ax = plt.subplots()
        ax.set_xlim(0, 100)
        ax.set_ylim(0, 100)
        q_start = self.q_init
        for iteration in range(self.iterations):
            q_near = self.find_nearest_vertex()
            q_new = self.generate_new_config()
            self.q_list.append(q_new)
            lc = LineCollection([[(q_start[0], q_start[1]), (q_new[0], q_new[1])]])
            ax.add_collection(lc)
            q_start = q_new
        plt.show()
    
def main():
    rrt = RRT([50, 50], 2000)
    rrt.plot_result()

if __name__ == "__main__":
    main()