# Rapidly Exploring Random Trees (RRT)
The Rapidly Exploring Random Trees (RRT) algorithm serves as a path planning tool in navigating from a starting point (𝑥<sub>𝑠𝑡𝑎𝑟𝑡</sub>) to a destination (𝑥<sub>𝑑𝑒𝑠𝑡</sub>) within a defined metric space (𝑋). This algorithm operates by searching through the free space (𝑋<sub>𝑓𝑟𝑒𝑒</sub>), avoiding obstacles (𝑋<sub>𝑜𝑏𝑠</sub>). The process begins by randomly sampling a point (𝑥<sub>𝑟𝑎𝑛𝑑</sub>) and finding the nearest neighboring point (𝑥<sub>𝑛𝑒𝑎𝑟</sub>) using Euclidean distance. A tree is then constructed, expanding from 𝑥<sub>𝑠𝑡𝑎𝑟𝑡</sub> to 𝑥<sub>𝑛𝑒𝑎𝑟</sub>.

The algorithm continues by connecting 𝑥<sub>𝑛𝑒𝑎𝑟</sub> to a newly generated point (𝑥<sub>𝑛𝑒𝑤</sub>) and checking for collisions with obstacles. If successful, 𝑥<sub>𝑛𝑒𝑤</sub> is added to the tree, and the process repeats until reaching the destination or reaching a specified iteration limit. However, due to the random nature of the algorithm, the path may contain redundant turning points, resulting in inefficiency.







## Overview of script

This Python script implements Rapidly Exploring Random Trees (RRT) for path planning in a 2D space with obstacles. The main file, 'rrt_path_planning.py', generates an RRT, visualizes the exploration process, and outputs the final path. The script is compatible with Ubuntu 20. For proper execution, ensure the required Python library is installed.

## Requirements

- Python 3
- Matplotlib

## Installation

Install the required libraries using the following command:

    sudo apt install python3

    pip install matplotlib


## Usage
After downloading the repository to your local machine, Navigate to the project directory:


     cd rrt-path-planning

### There is two method to running this script
#### Method 1 : User input
Run the script:
    
    python user_input_rrt.py

You have to give input following things:

    Enter start point x-coordinate: 
    Enter start point y-coordinate: 
    Enter goal point x-coordinate: 
    Enter goal point y-coordinate: 
    Enter the number of nodes: 
    Enter the step size: 
    Enter the number of obstacles: 
    Enter x_min for obstacle 1: 
    Enter x_max for obstacle 1: 
    Enter y_min for obstacle 1: 
    Enter y_max for obstacle 1: 

You will be prompted to provide input for the start and goal points, number of nodes, step size, and obstacles. The script will output the distance from start to goal through the RRT-generated path and the number of iterations required.

#### Method 2 : Predefined Values
This script uses predefined input values (which you can modify within the script) and outputs the distance from start to goal through the RRT-generated path and the number of iterations required.

Run the script:

    python functions.py


# License

MIT License






