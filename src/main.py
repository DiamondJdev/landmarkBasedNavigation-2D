import heapq
import matplotlib.pyplot as plt
import numpy as np

def load_map(file_path):
    """Load map from a txt file and return as a 2D list."""
    with open(file_path, 'r') as file:
        return [list(line.strip()) for line in file.readlines()]

def find_positions(grid, targets):
    """Find positions of multiple targets ('S', 'H', landmarks) in the grid."""
    positions = {}
    for y, row in enumerate(grid):
        for x, cell in enumerate(row):
            if cell in targets:
                positions[cell] = (x, y)
    return positions

def is_within_bounds(pos, grid):
    """Check if a position is within grid boundaries."""
    x, y = pos
    return 0 <= x < len(grid[0]) and 0 <= y < len(grid)

def is_walkable(pos, grid):
    """Check if a position is walkable (not an obstacle)."""
    x, y = pos
    return grid[y][x] in ('.', 'H') or grid[y][x].isalpha()

def get_neighbors(pos):
    """Return neighboring positions (up, down, left, right)."""
    x, y = pos
    return [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]

def heuristic(a, b):
    """Heuristic function for A* (Manhattan distance)."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_with_landmarks(grid, start, goal, landmarks):
    """A* algorithm with opportunistic use of landmarks."""
    open_set = []
    heapq.heappush(open_set, (0, start))  # (priority, position)
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)

        # If we reach the goal, reconstruct the path
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)  # Add the start position
            return path[::-1]  # Return reversed path

        for neighbor in get_neighbors(current):
            if not is_within_bounds(neighbor, grid) or not is_walkable(neighbor, grid):
                continue

            # Tentative g_score
            tentative_g_score = g_score[current] + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                if neighbor not in [i[1] for i in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # No path found

def plot_grid(grid, path):
    """Visualize the grid and the path."""
    grid_array = np.array([[1 if cell == '#' else 0 for cell in row] for row in grid])
    plt.imshow(grid_array, cmap='Greys', origin='upper')

    if path:
        x_coords, y_coords = zip(*path)
        plt.plot(x_coords, y_coords, color='red', linewidth=2, label='Path')

    # Highlight landmarks
    for y, row in enumerate(grid):
        for x, cell in enumerate(row):
            if cell.isalpha() and cell not in ('S', 'H'):
                plt.text(x, y, cell, color='blue', fontsize=12, ha='center', va='center')

    plt.legend()
    plt.show()

# Main execution
def calcPathing(x, y):
    # Load the map
    map_file = "map.txt"
    grid = load_map(map_file)

    # Find start, goal, and landmark positions
    positions = find_positions(grid, targets="SHABCDEFGHIJKLMNOPQRSTUVWXYZ")
    start = x, y
    goal = positions.get('H')
    landmarks = {key: val for key, val in positions.items() if key not in ('S', 'H')}

    if not goal:
        print("Error: Start or goal position not found in the map.")
    else:
        # Run pathfinding
        path = a_star_with_landmarks(grid, start, goal, landmarks)

        # Print and visualize the result
        if path:
            print("Path found:", path)
            plot_grid(grid, path)
        else:
            print("No path found.")

if __name__ == "__main__":
    calcPathing(2,2)