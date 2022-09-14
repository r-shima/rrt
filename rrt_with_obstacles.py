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
        self.circles = []
        self.q_goal = []
    
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
        distance = math.dist(self.q_rand, self.q_near)
        x_diff = self.q_rand[0] - self.q_near[0]
        y_diff = self.q_rand[1] - self.q_near[1]
        q_new_x = self.q_near[0] + (self.delta / distance) * x_diff
        q_new_y = self.q_near[1] + (self.delta / distance) * y_diff
        self.q_new = [q_new_x, q_new_y]
        return self.q_new

    def create_obstacle(self, center, radius):
        """Create a circular obstacle"""
        circle = plt.Circle(center, radius, color='black', fill=True)
        return circle
    
    def check_vertex_in_circle(self, vertex, circle):
        """Check if the vertex lies inside or on the circle"""
        distance = math.dist(circle[0], vertex)
        if distance <= circle[1]:
            return True
        return False

    def find_perp_distance(self, point1, point2, point3):
        """Calculate the perpendicular distance"""
        perp_distance = abs((point3[0] - point2[0]) * (point2[1] - point1[1]) - (point2[0] - point1[0]) * (point3[1] - point2[1])) / math.sqrt((point3[0] - point2[0]) ** 2 + (point3[1] - point2[1]) ** 2)
        return perp_distance

    def check_path_collision(self, circle):
        """Check if the path from q_near to q_new intersects with a circle"""
        distance = self.find_perp_distance(circle[0], self.q_near, self.q_new)
        if distance <= circle[1]:
            return True
        else:
            return False
    
    def check_collision(self, vertex):
        """Check if there is any collision"""
        for circle in self.circles:
            if self.check_vertex_in_circle(vertex, circle) or self.check_path_collision(circle):
                return True
        return False

    def check_collision_free_path(self, vertex):
        """Check for a collision free path"""
        for circle in self.circles:
            if not self.check_vertex_in_circle(vertex, circle) or self.check_path_collision(circle):
                return True
        return False

    def generate_random_goal(self):
        self.q_goal = [self.domain * np.random.rand(), self.domain * np.random.rand()]
        for circle in self.circles:
            while self.check_vertex_in_circle(self.q_goal, circle):
                self.q_goal = [self.domain * np.random.rand(), self.domain * np.random.rand()]

    def plot_result(self):
        """Plot the tree"""
        f, ax = plt.subplots()
        ax.set_xlim(0, 100)
        ax.set_ylim(0, 100)
        self.generate_random_goal()
        ax.plot(self.q_init[0], self.q_init[1], '*', color='blue')
        ax.plot(self.q_goal[0], self.q_goal[1], '*', color='blue')
        center1 = (20, 45)
        center2 = (75, 25)
        center3 = (50, 65)
        radius1 = 10
        radius2 = 5
        radius3 = 15
        self.circles.append([center1, radius1])
        self.circles.append([center2, radius2])
        self.circles.append([center3, radius3])
        obstacle1 = self.create_obstacle(center1, radius1)
        obstacle2 = self.create_obstacle(center2, radius2)
        obstacle3 = self.create_obstacle(center3, radius3)
        ax.add_artist(obstacle1)
        ax.add_artist(obstacle2)
        ax.add_artist(obstacle3)
        # for iteration in range(self.iterations):
        #     q_near = self.find_nearest_vertex()
        #     q_new = self.generate_new_config()
        #     self.q_list.append(q_new)
        #     x = [q_near[0], q_new[0]]
        #     y = [q_near[1], q_new[1]]
        #     ax.plot(x, y, color='blue')
        plt.show()
    
def main():
    rrt = RRT([50, 50], 2000)
    rrt.plot_result()

if __name__ == "__main__":
    main()