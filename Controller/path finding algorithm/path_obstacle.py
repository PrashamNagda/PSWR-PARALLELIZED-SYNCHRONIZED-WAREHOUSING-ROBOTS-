import tkinter as tk
from heapq import heappop, heappush

def create_grid(canvas, rows, cols, cell_size):
    """Creates a grid on the given canvas."""
    for i in range(rows + 1):
        canvas.create_line(0, i * cell_size, cols * cell_size, i * cell_size)
    for j in range(cols + 1):
        canvas.create_line(j * cell_size, 0, j * cell_size, rows * cell_size)

def get_cell(event, cell_size):
    """Converts a mouse click to grid cell coordinates."""
    x, y = event.x, event.y
    return y // cell_size, x // cell_size

def draw_cell(canvas, row, col, color, cell_size):
    """Draws a cell with a specific color."""
    x1, y1 = col * cell_size, row * cell_size
    x2, y2 = x1 + cell_size, y1 + cell_size
    canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="")

def heuristic(a, b):
    """Calculates the heuristic (Euclidean distance)."""
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

def a_star(start, goal, rows, cols):
    """Finds the shortest path using A* algorithm."""
    directions = [
        (-1, 0), (1, 0), (0, -1), (0, 1),  # Cardinal directions
        (-1, -1), (-1, 1), (1, -1), (1, 1)  # Diagonal directions
    ]
    
    open_set = []
    heappush(open_set, (0, start))  # (priority, position)
    g_cost = {start: 0}
    parent = {start: None}

    while open_set:
        _, current = heappop(open_set)

        if current == goal:
            path = []
            while current:
                path.append(current)
                current = parent[current]
            return path[::-1]  # Reverse the path

        for dr, dc in directions:
            neighbor = (current[0] + dr, current[1] + dc)
            if (
                0 <= neighbor[0] < rows and 
                0 <= neighbor[1] < cols and 
                neighbor not in obstacles
            ):
                tentative_g_cost = g_cost[current] + heuristic(current, neighbor)
                if neighbor not in g_cost or tentative_g_cost < g_cost[neighbor]:
                    g_cost[neighbor] = tentative_g_cost
                    f_cost = tentative_g_cost + heuristic(neighbor, goal)
                    heappush(open_set, (f_cost, neighbor))
                    parent[neighbor] = current

    return None  # No path found

def on_left_click(event):
    """Handles left click to set start and goal."""
    global start, goal, path
    cell = get_cell(event, cell_size)

    if not start:
        start = cell
        draw_cell(canvas, cell[0], cell[1], "green", cell_size)
    elif not goal and cell != start:
        goal = cell
        draw_cell(canvas, cell[0], cell[1], "red", cell_size)
        path = a_star(start, goal, rows, cols)

        if path:
            for step in path[1:-1]:  # Exclude start and goal
                draw_cell(canvas, step[0], step[1], "yellow", cell_size)
        else:
            print("No path found!")

def on_right_click(event):
    """Handles right click to add obstacles."""
    cell = get_cell(event, cell_size)
    if cell[0] >= rows or cell[1] >= cols:
        print(f"Invalid cell: {cell}. Click within grid bounds.")
        return

    if cell != start and cell != goal and cell not in obstacles:
        obstacles.add(cell)
        draw_cell(canvas, cell[0], cell[1], "black", cell_size)
    elif cell in obstacles:
        print(f"Obstacle already exists at: {cell}.")
    elif cell == start:
        print("Cannot place obstacle on the start point.")
    elif cell == goal:
        print("Cannot place obstacle on the goal point.")
# Window dimensions
rows, cols = 10, 10
cell_size = 40  # Size of each cell in pixels
start, goal, path = None, None, None  # Start and goal points
obstacles = set()  # Set to store obstacle positions

# Create the main window
root = tk.Tk()
root.title("10x10 Grid Pathfinding with A* Algorithm and Obstacles")

# Create a canvas to draw the grid
canvas_width = cols * cell_size
canvas_height = rows * cell_size
canvas = tk.Canvas(root, width=canvas_width, height=canvas_height, bg="white")
canvas.pack()

# Draw the grid
create_grid(canvas, rows, cols, cell_size)

# Bind click events
canvas.bind("<Button-1>", on_left_click)  # Left click for start/goal
canvas.bind("<Button-3>", on_right_click)  # Right click for obstacles

# Run the application
root.mainloop()
