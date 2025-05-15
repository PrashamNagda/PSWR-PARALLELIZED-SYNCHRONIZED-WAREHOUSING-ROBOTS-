import cv2
import cv2.aruco as aruco
import numpy as np
from heapq import heappop, heappush

# Define ArUco marker IDs
START_POINT_MARKER_ID = 23
END_POINT_MARKER_ID = 24

# Grid settings
rows, cols = 10, 10
cell_size = 50  # Adjust cell size for visibility
start, goal, path = None, None, None

# Initialize ArUco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

def heuristic(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

def a_star(start, goal):
    """Finds the shortest path using A* algorithm."""
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    open_set = []
    heappush(open_set, (0, start))
    g_cost = {start: 0}
    parent = {start: None}
    while open_set:
        _, current = heappop(open_set)
        if current == goal:
            path = []
            while current:
                path.append(current)
                current = parent[current]
            return path[::-1]
        for dr, dc in directions:
            neighbor = (current[0] + dr, current[1] + dc)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                tentative_g_cost = g_cost[current] + heuristic(current, neighbor)
                if neighbor not in g_cost or tentative_g_cost < g_cost[neighbor]:
                    g_cost[neighbor] = tentative_g_cost
                    heappush(open_set, (tentative_g_cost + heuristic(neighbor, goal), neighbor))
                    parent[neighbor] = current
    return None

def draw_grid(frame):
    """Draws a grid overlay on the camera feed with thicker lines."""
    for i in range(rows + 1):
        cv2.line(frame, (0, i * cell_size), (cols * cell_size, i * cell_size), (200, 200, 200), 2)
    for j in range(cols + 1):
        cv2.line(frame, (j * cell_size, 0), (j * cell_size, rows * cell_size), (200, 200, 200), 2)

def draw_path(frame, path):
    """Draws the A* path on the camera feed."""
    if path:
        for step in path:
            x1, y1 = step[1] * cell_size, step[0] * cell_size
            x2, y2 = x1 + cell_size, y1 + cell_size
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), -1)

def detect_aruco():
    """Detects ArUco markers and maps them to the grid."""
    global start, goal, path
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture image")
            break
        frame = cv2.resize(frame, (cols * cell_size, rows * cell_size))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:
            for i in range(len(ids)):
                marker_id = ids[i][0]
                marker_corners = corners[i][0]
                cx, cy = int(marker_corners[:, 0].mean()), int(marker_corners[:, 1].mean())
                row, col = cy // cell_size, cx // cell_size
                if marker_id == START_POINT_MARKER_ID:
                    start = (row, col)
                    print(f"Start Point Detected at Grid: {start}")
                elif marker_id == END_POINT_MARKER_ID:
                    goal = (row, col)
                    print(f"End Point Detected at Grid: {goal}")
        if start and goal:
            path = a_star(start, goal)
        draw_grid(frame)
        if path:
            draw_path(frame, path)
        cv2.imshow("Aruco Grid Pathfinding", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

detect_aruco()
