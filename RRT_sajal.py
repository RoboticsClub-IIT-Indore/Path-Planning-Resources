import numpy as np
import matplotlib.pyplot as plt
import random
from math import sqrt

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

class RRTStar:
    def __init__(self, start, goal, obstacle_list, rand_area, expand_dist=1.0, goal_sample_rate=5, max_iter=100000000):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.obstacle_list = obstacle_list
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dist = expand_dist
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.node_list = [self.start]

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rand = [random.uniform(self.min_rand, self.max_rand), random.uniform(self.min_rand, self.max_rand)]
        else:
            rand = [self.goal.x, self.goal.y]
        return Node(rand[0], rand[1])

    def steer(self, from_node, to_node):
        new_node = Node(from_node.x, from_node.y)
        dist = self.calc_distance(from_node, to_node)
        new_node.x += min(self.expand_dist, dist) * (to_node.x - from_node.x) / dist
        new_node.y += min(self.expand_dist, dist) * (to_node.y - from_node.y) / dist
        new_node.parent = from_node
        return new_node

    def get_nearest_node_index(self, node_list, rand_node):
        dlist = [(node.x - rand_node.x)**2 + (node.y - rand_node.y)**2 for node in node_list]
        return dlist.index(min(dlist))

    def is_collision_free(self, node):
        for (ox, oy, size) in self.obstacle_list:
            dx = ox - node.x
            dy = oy - node.y
            if sqrt(dx**2 + dy**2) <= size:
                return False
        return True

    def planning(self):
        plt.figure()
        for obstacle in self.obstacle_list:
            circle = plt.Circle((obstacle[0], obstacle[1]), obstacle[2], color="k")
            plt.gca().add_artist(circle)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.goal.x, self.goal.y, "xg")
        plt.grid(True)
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])

        for i in range(self.max_iter):
            rand_node = self.get_random_node()
            nearest_index = self.get_nearest_node_index(self.node_list, rand_node)
            nearest_node = self.node_list[nearest_index]
            new_node = self.steer(nearest_node, rand_node)

            if not self.is_collision_free(new_node):
                continue

            self.node_list.append(new_node)
            
            # Draw the tree
            plt.plot([nearest_node.x, new_node.x], [nearest_node.y, new_node.y], "-b")
            plt.pause(0.01)

            if self.calc_distance(new_node, self.goal) <= self.expand_dist:
                return self.generate_final_path(new_node)

        plt.show()
        return None

    def calc_distance(self, node1, node2):
        return sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

    def generate_final_path(self, goal_node):
        path = [[goal_node.x, goal_node.y]]
        node = goal_node
        while node.parent is not None:
            node = node.parent
            path.append([node.x, node.y])
        return path

def main():
    start = [0, 0]
    goal = [10, 10]
    obstacle_list = [(5, 5, 0.5), (3, 6, 0.5), (7, 7, 1)]
    rand_area = [-2, 15]

    rrt_star = RRTStar(start, goal, obstacle_list, rand_area)
    path = rrt_star.planning()

    if path is None:
        print("No path found!")
    else:
        print("Path found!")
        # Draw final path
        path = np.array(path)
        plt.plot(path[:, 0], path[:, 1], '-r')
        plt.pause(0.01)
        plt.show()

if __name__ == '__main__':
    main()
