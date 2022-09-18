import matplotlib.pyplot as plt
import numpy as np
import math

class RRT:
    """Implementing the Rapidly-Exploring Random Tree algorithm"""

    def __init__(self, num_of_obstacles):
        self.q_init = []
        # self.iterations = iterations
        self.delta = 1
        self.domain = 100
        self.q_list = []
        self.circles = []
        self.num_of_obstacles = num_of_obstacles
        self.q_new = None
    
    def generate_random_config(self):
        """Generate a random position in the 100x100 domain"""
        self.q_rand = [self.domain * np.random.rand(), self.domain * np.random.rand()]
        return self.q_rand

    def find_nearest_vertex(self):
        """Find the nearest vertex from a random position in q_list"""
        dist_list = []
        for vertex in self.q_list:
            distance = math.dist(vertex, self.q_rand)
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

    def create_random_obstacle(self): # Original had center and radius as arguments
        """Create a random circular obstacle"""
        while self.num_of_obstacles != 0:
            center_x = np.random.randint(1, 100)
            center_y = np.random.randint(1, 100)
            center = [center_x, center_y]
            radius = np.random.randint(1, 10)
            self.circles.append([center, radius])
            self.num_of_obstacles -= 1
        # circle = plt.Circle(center, radius, color='black', fill=True)
        # return circle
    
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

    def check_path_collision(self, circle, q1, q2):
        """Check if the path from a vertex to another vertex intersects with a circle"""
        distance = self.find_perp_distance(circle[0], q1, q2)
        if distance <= circle[1]:
            return True
        return False
    
    def check_collision(self, vertex):
        """Check if there is any collision"""
        for circle in self.circles:
            if self.check_vertex_in_circle(vertex, circle) or self.check_path_collision(circle, self.q_near, vertex):
                return True
        return False

    def check_collision_free_path(self):
        """Check for a collision free path from a new vertex to the goal"""
        for circle in self.circles:
            if self.check_path_collision(circle, self.q_new, self.q_goal):
                return False
        return True

    def generate_random_start(self):
        self.q_init = [self.domain * np.random.rand(), self.domain * np.random.rand()]
        for circle in self.circles:
            while self.check_vertex_in_circle(self.q_init, circle):
                self.q_init = [self.domain * np.random.rand(), self.domain * np.random.rand()]
        return self.q_init
    
    def generate_random_goal(self):
        self.q_goal = [self.domain * np.random.rand(), self.domain * np.random.rand()]
        for circle in self.circles:
            while self.check_vertex_in_circle(self.q_goal, circle):
                self.q_goal = [self.domain * np.random.rand(), self.domain * np.random.rand()]
        return self.q_goal

    def plot_result(self):
        """Plot the tree"""
        f, ax = plt.subplots()
        ax.set_xlim(0, 100)
        ax.set_ylim(0, 100)
        # center1 = (20, 45)
        # center2 = (75, 25)
        # center3 = (70, 65)
        # radius1 = 10
        # radius2 = 5
        # radius3 = 15
        # self.circles.append([center1, radius1])
        # self.circles.append([center2, radius2])
        # self.circles.append([center3, radius3])
        # obstacle1 = self.create_random_obstacle(center1, radius1)
        # obstacle2 = self.create_random_obstacle(center2, radius2)
        # obstacle3 = self.create_random_obstacle(center3, radius3)
        # ax.add_artist(obstacle1)
        # ax.add_artist(obstacle2)
        # ax.add_artist(obstacle3)
        self.create_random_obstacle()
        self.q_init = self.generate_random_start()
        self.q_list.append(self.q_init)
        self.q_new = self.q_init
        self.q_goal = self.generate_random_goal()
        ax.plot(self.q_init[0], self.q_init[1], 'x', color='blue')
        ax.plot(self.q_goal[0], self.q_goal[1], 'x', color='blue')
        for circle in self.circles:
            circle = plt.Circle(circle[0], circle[1], color='black', fill=True)
            ax.add_artist(circle)
        while not self.check_collision_free_path():
            self.q_rand = self.generate_random_config()
            self.q_near = self.find_nearest_vertex()
            self.q_new = self.generate_new_config()
            if not self.check_collision(self.q_new):
                self.q_list.append(self.q_new)
                x1 = [self.q_near[0], self.q_new[0]]
                y1 = [self.q_near[1], self.q_new[1]]
                ax.plot(x1, y1, color='blue')
        x2 = [self.q_new[0], self.q_goal[0]]
        y2 = [self.q_new[1], self.q_goal[1]]
        ax.plot(x2, y2, color='blue')
        self.q_list.append(self.q_goal)
        # print(self.q_list)
        # print(self.q_goal)
        # self.q_list.reverse()
        plt.show()
    
def main():
    rrt = RRT(20)
    rrt.plot_result()

if __name__ == "__main__":
    main()