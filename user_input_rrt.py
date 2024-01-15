from functions import Node, generate_rrt, plot_rrt, retrace_path, calculate_path_distance

def get_user_input():
    start_x = float(input("Enter start point x-coordinate: "))
    start_y = float(input("Enter start point y-coordinate: "))
    start_node = Node(start_x, start_y)

    goal_x = float(input("Enter goal point x-coordinate: "))
    goal_y = float(input("Enter goal point y-coordinate: "))
    goal_node = Node(goal_x, goal_y)

    num_nodes = int(input("Enter the number of nodes: "))
    step_size = float(input("Enter the step size: "))

    obstacles = []
    num_obstacles = int(input("Enter the number of obstacles: "))

    for i in range(num_obstacles):
        x_min = float(input(f"Enter x_min for obstacle {i + 1}: "))
        x_max = float(input(f"Enter x_max for obstacle {i + 1}: "))
        y_min = float(input(f"Enter y_min for obstacle {i + 1}: "))
        y_max = float(input(f"Enter y_max for obstacle {i + 1}: "))

        obstacles.append({'x_min': x_min, 'x_max': x_max, 'y_min': y_min, 'y_max': y_max})

    return start_node, goal_node, num_nodes, step_size, obstacles

if __name__ == "__main__":
    start_node, goal_node, num_nodes, step_size, obstacles = get_user_input()

    rrt_nodes, iterations = generate_rrt(start_node, goal_node, num_nodes, step_size, obstacles)
    plot_rrt(rrt_nodes, start_node, goal_node, obstacles, retrace=True)

    final_path = retrace_path(goal_node)
    path_distance = calculate_path_distance(final_path)

    print("Distance from start to goal through RRT path:", path_distance)
    print("Number of iterations required:", iterations)
