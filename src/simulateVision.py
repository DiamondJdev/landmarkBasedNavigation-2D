from main import *
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from random import randint

# Local Map Visibility: View the area around the drone
def get_local_map(grid, drone_pos, view_radius):
    x, y = drone_pos
    local_map = []

    # Generate local map based on view_radius
    for i in range(x - view_radius, x + view_radius + 1):
        row = []
        for j in range(y - view_radius, y + view_radius + 1):
            if 0 <= i < len(grid) and 0 <= j < len(grid[0]):  # Ensure within bounds
                row.append(grid[i][j])
            else:
                row.append('#')  # Treat out-of-bound as obstacles
        local_map.append(row)

    return local_map


# Simple Pathfinding Algorithm: A* Algorithm (Modified to work with local map)
def a_star_local(grid, start, goal, view_radius):
    open_list = deque([start])  # Using deque as a queue
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_list:
        current = open_list.popleft()
        if current == goal:
            return reconstruct_path(came_from, current)

        x, y = current
        local_map = get_local_map(grid, current, view_radius)

        neighbors = get_neighbors(x, y, local_map)
        for neighbor in neighbors:
            nx, ny = neighbor
            tentative_g_score = g_score.get((x, y), float('inf')) + 1  # Assuming cost of moving is 1
            if tentative_g_score < g_score.get((nx, ny), float('inf')):
                came_from[(nx, ny)] = (x, y)
                g_score[(nx, ny)] = tentative_g_score
                f_score[(nx, ny)] = tentative_g_score + heuristic(neighbor, goal)
                if neighbor not in open_list:
                    open_list.append(neighbor)

    return None  # No path found


# Heuristic for A* (Manhattan Distance)
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# Reconstruct path after finding a path
def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


# Get valid neighbors from the local map
def get_neighbors(x, y, local_map):
    neighbors = []
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right
    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if 0 <= nx < len(local_map) and 0 <= ny < len(local_map[0]) and local_map[nx][ny] != '#':
            neighbors.append((nx, ny))
    return neighbors


# Visualization of simulation
def visualize_simulation(grid, drone_pos, path, revealed_map):
    grid_array = np.array([[1 if cell == '#' else 0 for cell in row] for row in grid])
    revealed_array = np.array([[1 if cell == 'R' else 0 for cell in row] for row in revealed_map])

    plt.imshow(grid_array, cmap='Greys', origin='upper')

    # Plot the revealed areas (as R)
    plt.imshow(revealed_array, cmap='Blues', alpha=0.3, origin='upper')

    # Plot the path if available
    if path:
        x_coords, y_coords = zip(*path)
        plt.plot(y_coords, x_coords, color='red', linewidth=2, label='Path')

    # Plot the drone's current position
    plt.scatter(drone_pos[1], drone_pos[0], color='green', s=100, label='Drone')

    plt.legend()
    plt.pause(0.5)
    plt.clf()


# Update the revealed map
def update_revealed_map(revealed_map, local_map, drone_pos, view_radius):
    x, y = drone_pos
    for i in range(x - view_radius, x + view_radius + 1):
        for j in range(y - view_radius, y + view_radius + 1):
            if 0 <= i < len(revealed_map) and 0 <= j < len(revealed_map[0]):
                if local_map[i - (x - view_radius)][j - (y - view_radius)] != '#':
                    revealed_map[i][j] = 'R'  # Mark as revealed

# Main execution code
if __name__ == "__main__":
    # Pull map
    choice = input("Run Complex testing (y/n): ")
    if choice == 'y':
        mapChoice = randint(0, 1)
        if mapChoice == 0:
            map_file = "complexMaze.txt"
            start = (1, 1)
            goal = (25, 49)
        else:
            map_file = "anotherComplex.txt"
            start = (1, 1)
            goal = (25, 49)
    else:
        map_file = "map.txt"
        start = (1, 1)  # Starting position
        goal = (2, 3)  # Goal position (H)

    print("Map:", map_file)
    #   (0_0)
    # \___|___/
    #     |
    #    / \
    grid = load_map(map_file)  # Load map file
    view_radius = 3  # View radius (size of local area around the drone)

    # Get path from the local map
    path = a_star_local(grid, start, goal, view_radius)

    # Visualize the simulation
    drone_pos = start
    if not path:
        print("No path found.")
        exit(0)
    for position in path:
        visualize_simulation(grid, drone_pos, path)
        drone_pos = position