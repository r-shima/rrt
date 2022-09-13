import matplotlib.pyplot as plt
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
        magnitude = math.dist(self.q_rand, self.q_near)
        x = self.q_rand[0] / magnitude
        y = self.q_rand[1] / magnitude
        unit_vector = [x, y]
        q_new_x = self.q_near[0] + unit_vector[0]
        q_new_y = self.q_near[1] + unit_vector[1]
        self.q_new = [q_new_x, q_new_y]
        return self.q_new
    
def main():
    rrt = RRT([50, 50], 2000)
    rrt.generate_random_config()
    rrt.find_nearest_vertex()
    rrt.generate_new_config()

if __name__ == "__main__":
    main()