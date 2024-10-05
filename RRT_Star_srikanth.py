import numpy as np
import matplotlib.pyplot as plt
import random
import math

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

class RRTStar:
    def __init__(self, start, goal, obstacles, x_max, y_max, step_size=1.0, goal_sample_rate=0.1, max_iter=500):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.obstacles = obstacles
        self.x_max = x_max
        self.y_max = y_max
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.node_list = [self.start]

    def is_in_obstacle(self, node):
        for ox, oy, size in self.obstacles:
            if (ox - node.x)**2 + (oy - node.y)**2 <= size**2:
                return True
        return False

    def sample(self):
        if random.random() > self.goal_sample_rate:
            return Node(random.uniform(0, self.x_max), random.uniform(0, self.y_max))
        return Node(self.goal.x, self.goal.y)

    def nearest_node(self, node):
        return min(self.node_list, key=lambda n: (n.x - node.x)**2 + (n.y - node.y)**2)

    def is_collision_free(self, node1, node2):
        steps = int(self.distance(node1, node2) / (self.step_size / 10))
        for i in range(steps + 1):
            t = i / (1+steps)
            temp_x = node1.x + t * (node2.x - node1.x)
            temp_y = node1.y + t * (node2.y - node1.y)
            temp_node = Node(temp_x, temp_y)
            if self.is_in_obstacle(temp_node):
                return False
        return True

    def get_nearby_nodes(self, node, radius):
        return [n for n in self.node_list if self.distance(n, node) <= radius]

    def distance(self, node1, node2):
        return math.hypot(node1.x - node2.x, node1.y - node2.y)

    def rewire(self, new_node, nearby_nodes):
        for node in nearby_nodes:
            if self.is_collision_free(new_node, node):
                cost = new_node.cost + self.distance(new_node, node)
                if cost < node.cost:
                    node.parent = new_node
                    node.cost = cost

    def plan(self):
        for iter_num in range(self.max_iter):
            sample_node = self.sample()
            nearest_node = self.nearest_node(sample_node)
            direction = np.array([sample_node.x - nearest_node.x, sample_node.y - nearest_node.y])
            length = np.linalg.norm(direction)
            if length == 0:
                continue
            direction = direction / length * min(self.step_size, length)
            new_node = Node(nearest_node.x + direction[0], nearest_node.y + direction[1])
            new_node.parent = nearest_node
            new_node.cost = nearest_node.cost + self.distance(nearest_node, new_node)

            if not self.is_collision_free(nearest_node, new_node):
                continue

            nearby_nodes = self.get_nearby_nodes(new_node, radius=5.0)
            min_cost = new_node.cost
            best_parent = nearest_node

            for node in nearby_nodes:
                if self.is_collision_free(node, new_node):
                    temp_cost = node.cost + self.distance(node, new_node)
                    if temp_cost < min_cost:
                        best_parent = node
                        min_cost = temp_cost

            new_node.parent = best_parent
            new_node.cost = min_cost
            self.node_list.append(new_node)
            self.rewire(new_node, nearby_nodes)

            if self.distance(new_node, self.goal) <= self.step_size:
                if self.is_collision_free(new_node, self.goal):
                    self.goal.parent = new_node
                    self.goal.cost = new_node.cost + self.distance(new_node, self.goal)
                    self.node_list.append(self.goal)
                    print(f"Goal reached at iteration {iter_num}")
                    return self.get_final_path()

        print("No path found within the maximum iterations.")
        return None

    def get_final_path(self):
        path = []
        node = self.goal
        while node is not None:
            path.append([node.x, node.y])
            node = node.parent
        return path[::-1]

def plot_path(rrt_star, path):
    fig, ax = plt.subplots(figsize=(10, 10))
    plt.title("RRT* Path Planning with Highlighted Optimal Path")

    for (ox, oy, size) in rrt_star.obstacles:
        circle = plt.Circle((ox, oy), size, color='black')
        ax.add_patch(circle)

    for node in rrt_star.node_list:
        if node.parent:
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], color='lightgray', linewidth=0.5)

    if path is not None:
        x_coords, y_coords = zip(*path)
        plt.plot(x_coords, y_coords, '-r', linewidth=2.5, label='Optimal Path')

    plt.plot(rrt_star.start.x, rrt_star.start.y, "go", markersize=10, label='Start')
    plt.plot(rrt_star.goal.x, rrt_star.goal.y, "ro", markersize=10, label='Goal')

    if path is not None:
        plt.scatter(x_coords, y_coords, c='red', s=20)

    plt.xlim(0, rrt_star.x_max)
    plt.ylim(0, rrt_star.y_max)
    plt.grid(True)
    plt.legend()
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.show()

if __name__ == "__main__":
    start = [0, 0]
    goal = [10, 10]
    obstacles = [
        (5, 5, 1),
        (7, 8, 1),
        (3, 6, 1),
        (6, 3, 1),
        (8, 5, 1),
        (4, 3, 1),
        (7, 5, 1.5)
    ]
    rrt_star = RRTStar(start, goal, obstacles, x_max=15, y_max=15, step_size=0.5, goal_sample_rate=0.05, max_iter=1000)
    path = rrt_star.plan()
    plot_path(rrt_star, path)

