import matplotlib.pyplot as plt
import numpy as np
import math

class RRT:
    """Implementing the Rapidly-Exploring Random Tree algorithm"""

    def __init__(self, iterations, num_of_obstacles):
        self.q_init = []
        self.q_goal = []
        self.iterations = iterations
        self.delta = 1
        self.domain = 100
        self.q_list = []
        self.circles = []
        self.num_of_obstacles = num_of_obstacles
        self.q_prev_list = [[]]
    
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

    def create_random_obstacle(self):
        """Create a random circular obstacle"""
        while self.num_of_obstacles != 0:
            center_x = np.random.randint(1, 100)
            center_y = np.random.randint(1, 100)
            center = [center_x, center_y]
            radius = np.random.randint(1, 10)
            self.circles.append([center, radius])
            self.num_of_obstacles -= 1
    
    def check_vertex_in_circle(self, vertex, circle):
        """Check if the vertex lies inside or on the circle"""
        distance = math.dist(circle[0], vertex)
        if distance <= circle[1]:
            return True
        return False

    def find_u(self, point1, point2, point3):
        """Calculate u required to check for path collision"""
        x_delta = point3[0] - point2[0]
        y_delta = point3[1] - point2[1]
        u = ((point1[0] - point2[0]) * x_delta + (point1[1] - point2[1]) * y_delta) / (x_delta * x_delta + y_delta * y_delta)
        return u

    def check_path_collision(self, circle, point1, point2):
        """Check if the path from a vertex to another vertex intersects with a circle"""
        x_delta = point2[0] - point1[0]
        y_delta = point2[1] - point1[1]
        u = self.find_u(circle[0], point1, point2)
        if u < 0:
            closest_point = point1
        elif u > 1:
            closest_point = point2
        else:
            closest_point = (point1[0] + u * x_delta, point1[1] + u * y_delta)
        
        distance = math.dist(closest_point, circle[0])
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
        """Generate a random start location"""
        self.q_init = [self.domain * np.random.rand(), self.domain * np.random.rand()]
        for circle in self.circles:
            while self.check_vertex_in_circle(self.q_init, circle):
                self.q_init = [self.domain * np.random.rand(), self.domain * np.random.rand()]
        return self.q_init
    
    def generate_random_goal(self):
        """Generate a random goal location"""
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
        self.create_random_obstacle()
        self.q_init = self.generate_random_start()
        self.q_list.append(self.q_init)
        self.q_goal = self.generate_random_goal()
        ax.plot(self.q_init[0], self.q_init[1], 'o', color='blue')
        ax.plot(self.q_goal[0], self.q_goal[1], 'x', color='blue')
        for circle in self.circles:
            circle = plt.Circle(circle[0], circle[1], color='black', fill=True)
            ax.add_artist(circle)
        for iteration in range(self.iterations):
            self.q_rand = self.generate_random_config()
            self.q_near = self.find_nearest_vertex()
            self.q_new = self.generate_new_config()
            if not self.check_collision(self.q_new):
                self.q_list.append(self.q_new)
                self.q_prev_list.append(self.q_near)
                x1 = [self.q_near[0], self.q_new[0]]
                y1 = [self.q_near[1], self.q_new[1]]
                ax.plot(x1, y1, color='blue')
                if self.check_collision_free_path():
                    x2 = [self.q_new[0], self.q_goal[0]]
                    y2 = [self.q_new[1], self.q_goal[1]]
                    ax.plot(x2, y2, color='blue')
                    break
        self.q_list.append(self.q_goal)
        self.q_prev_list.append(self.q_new)
        current = self.q_list[-1]
        previous = self.q_prev_list[-1]
        while True:
            if previous == []:
                break
            x3 = [current[0], previous[0]]
            y3 = [current[1], previous[1]]
            ax.plot(x3, y3, color='red')
            current = previous
            if current in self.q_list:
                index = self.q_list.index(current)
            previous = self.q_prev_list[index]
        plt.show()
    
def main():
    rrt = RRT(2000, 40)
    rrt.plot_result()

if __name__ == "__main__":
    main()