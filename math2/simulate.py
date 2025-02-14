from random import randint
from main import *
import matplotlib.pyplot as plt
import numpy as np

def simulate_vision_system(grid, drone_pos, vision_range=2):
    """Simulate the vision system detecting the environment."""
    x, y = drone_pos
    visible_map = [['?' for _ in row] for row in grid]  # Start with unknown map

    for i in range(-vision_range, vision_range + 1):
        for j in range(-vision_range, vision_range + 1):
            nx, ny = x + i, y + j
            if 0 <= ny < len(grid) and 0 <= nx < len(grid[0]):
                visible_map[ny][nx] = grid[ny][nx]

    return visible_map

def update_drone_position(current_pos, next_pos):
    """Move the drone to the next position."""
    return next_pos

def apply_vision_to_grid(grid, visible_map):
    """Apply vision updates to the grid."""
    updated_grid = [row[:] for row in grid]  # Create a copy of the grid
    for y, row in enumerate(visible_map):
        for x, cell in enumerate(row):
            if cell != '?':  # If the cell is visible, update the grid
                updated_grid[y][x] = cell
    return updated_grid

def visualize_simulation(grid, drone_pos, path):
    """Visualize the real-time simulation."""
    grid_array = np.array([[1 if cell == '#' else 0 for cell in row] for row in grid])
    plt.imshow(grid_array, cmap='Greys', origin='upper')

    # Plot the path
    if path:
        x_coords, y_coords = zip(*path)
        plt.plot(x_coords, y_coords, color='red', linewidth=2, label='Path')

    # Plot the drone
    plt.scatter(*drone_pos, color='green', s=100, label='Drone')

    plt.legend()
    plt.pause(0.5)  # Pause for a real-time effect
    plt.clf()

# Main simulation
if __name__ == "__main__":
    # Load map and initialize positions
    choice = input("Run Complex testing (y/n): ")
    if choice == 'y':
        mapChoice = randint(0, 1)
        if mapChoice == 0:
            map_file = "complexMaze.txt"
        else:
            map_file = "anotherComplex.txt"
    else:
        map_file = "map.txt"
    print("Map:", map_file)
    grid = load_map(map_file)

    original_grid = [row[:] for row in grid]  # Keep the original map for reference
    positions = find_positions(grid, targets="SHABCDEFGHIJKLMNOPQRSTUVWXYZ")
    start = positions['S']
    goal = positions['H']
    landmarks = {key: val for key, val in positions.items() if key not in ('S', 'H')}

    if not start or not goal:
        print("Error: Start or goal position not found.")
    else:
        # Initialize simulation variables
        drone_pos = start
        vision_range = 90
        visible_map = simulate_vision_system(original_grid, drone_pos, vision_range)

        # Simulate movement along the path
        plt.figure()
        while drone_pos != goal:
            # Update the visible map
            visible_map = simulate_vision_system(original_grid, drone_pos, vision_range)

            # Apply vision to the grid
            grid = apply_vision_to_grid(grid, visible_map)

            # Recalculate the path based on the updated grid
            path = a_star_with_landmarks(grid, drone_pos, goal, landmarks)

            if not path:
                print("No valid path to goal.")
                break

            # Move the drone to the next step in the path
            drone_pos = update_drone_position(drone_pos, path[1])

            # Visualize the simulation
            visualize_simulation(grid, drone_pos, path)

        plt.show()
        print("Simulation complete")