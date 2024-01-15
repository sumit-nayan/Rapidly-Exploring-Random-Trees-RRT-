# %%
import matplotlib.pyplot as plt
import random
import math

# %%
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


# %%
def distance(node1, node2):
    return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

# %%
def find_nearest_node(nodes, random_point):
    return min(nodes, key=lambda node: distance(node, random_point))


# %%
def is_collision(node, obstacles):
    for obstacle in obstacles:
        if (
            obstacle['x_min'] < node.x < obstacle['x_max'] and
            obstacle['y_min'] < node.y < obstacle['y_max']
        ):
            return True
    return False

# %%
def extend(from_node, to_point, step_size, obstacles):
    theta = math.atan2(to_point.y - from_node.y, to_point.x - from_node.x)
    new_x = from_node.x + step_size * math.cos(theta)
    new_y = from_node.y + step_size * math.sin(theta)

    new_node = Node(new_x, new_y)

    if not is_collision(new_node, obstacles):
        new_node.parent = from_node
        return new_node

    return None

# %%
def generate_rrt(start, goal, num_nodes, step_size, obstacles):
    nodes = [start]
    iterations = 0

    for _ in range(num_nodes):
        random_point = Node(random.uniform(0, 10), random.uniform(0, 10))
        nearest_node = find_nearest_node(nodes, random_point)

        new_node = extend(nearest_node, random_point, step_size, obstacles)

        if new_node:
            nodes.append(new_node)

            if distance(new_node, goal) < step_size:
                goal.parent = new_node
                nodes.append(goal)
                return nodes, iterations

        iterations += 1

    return nodes, iterations

# %%
def retrace_path(goal):
    path = []
    current = goal

    while current:
        path.append((current.x, current.y))
        current = current.parent

    return path[::-1]


# %%
def calculate_path_distance(path):
    distance_sum = 0
    for i in range(1, len(path)):
        distance_sum += distance(Node(path[i-1][0], path[i-1][1]), Node(path[i][0], path[i][1]))
    return distance_sum

# %%
def plot_rrt(nodes, start, goal, obstacles, retrace=False):
    plt.scatter([node.x for node in nodes], [node.y for node in nodes], color='gray')
    plt.scatter(start.x, start.y, color='green', marker='o', label='Start')
    plt.scatter(goal.x, goal.y, color='red', marker='o', label='Goal')

    for node in nodes:
        if node.parent:
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], color='black')

    # Plot obstacles
    for obstacle in obstacles:
        plt.gca().add_patch(plt.Rectangle(
            (obstacle['x_min'], obstacle['y_min']),
            obstacle['x_max'] - obstacle['x_min'],
            obstacle['y_max'] - obstacle['y_min'],
            color='black'
        ))

    if retrace:
        path = retrace_path(goal)
        plt.plot([point[0] for point in path], [point[1] for point in path], color='blue', linestyle='dashed', label='Retrace Path')

    plt.legend()
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Rapidly Exploring Random Trees (RRT) with Obstacles')
    plt.show()


# %%
if __name__ == "__main__":
    start_node = Node(0, 0)
    goal_node = Node(10, 10)

    num_nodes = 1000
    step_size = 1

    obstacles = [
        {'x_min': 3, 'x_max': 4, 'y_min': 3, 'y_max': 4},
        {'x_min': 7, 'x_max': 8, 'y_min': 2, 'y_max': 3},
    ]

    rrt_nodes, iterations = generate_rrt(start_node, goal_node, num_nodes, step_size, obstacles)
    plot_rrt(rrt_nodes, start_node, goal_node, obstacles, retrace=True)

    final_path = retrace_path(goal_node)
    path_distance = calculate_path_distance(final_path)

    print("Distance from start to goal through RRT path:", path_distance)
    print("Number of iterations required:", iterations)

# %%



