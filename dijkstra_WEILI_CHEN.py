import numpy as np
import heapq
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time 
import math

clearance = 5
map_width = 1200
map_height = 500

# map with obstacles
obstacles = [
    [(100, 100), (100, 500), (175, 500), (175, 100)],

    [(275, 0), (275, 400), (350, 400), (350, 0)],

    # [(650-150*np.cos(np.pi/6), 400-150-150*np.sin(np.pi/6)),
    #  (650-150*np.cos(np.pi/6), 400-150*np.sin(np.pi/6)),
    #  (650, 400),
    #  (650+150*np.cos(np.pi/6), 400-150*np.sin(np.pi/6)),
    #  (650+150*np.cos(np.pi/6), 400-150-150*np.sin(np.pi/6)),
    #  (650, 100)],

    [(650-150*np.cos(np.pi/6), 400-150-150*0.5),
     (650-150*np.cos(np.pi/6), 400-150*0.5),
     (650, 400),
     (650+150*np.cos(np.pi/6), 400-150*0.5),
     (650+150*np.cos(np.pi/6), 400-150-150*0.5),
     (650, 100)],

    [(900, 450), (1100, 450), (1100, 50), (900, 50), (900, 125), (1020, 125), (1020, 375), (900, 375)]
]

def within_clearance(x, y, polygon, clearance):
    for i in range(len(polygon)):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % len(polygon)]
        if point_line_dist(x, y, p1[0], p1[1], p2[0], p2[1]) < clearance:
            return True
    return False

# def point_line_dist(px, py, x1, y1, x2, y2):
#     liner = np.array([x2 - x1, y2 - y1])
#     ptr = np.array([px - x1, py - y1])
#     line_len = np.linalg.norm(liner)
#     line_unitvec = liner / line_len if line_len != 0 else liner
#     ptr_scaled = ptr / line_len
#     t = np.dot(line_unitvec, ptr_scaled)
#     t = max(0.0, min(1.0, t))
#     nearest = liner * t
#     dist = np.linalg.norm(ptr - nearest)
#     return dist


def point_line_dist(px, py, x1, y1, x2, y2):
    # line_vec = [(x2 - x1), (y2 - y1)]
    # pnt_vec = [(px - x1), (py - y1)]
    line_len = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
    # if line_len == 0:
    #     return (pnt_vec[0] ** 2 + pnt_vec[1] ** 2) ** 0.5

    line_unitvec = [(x2 - x1) / line_len, (y2 - y1) / line_len]
    proj_length = (px - x1) * line_unitvec[0] + (py - y1) * line_unitvec[1]
    proj_length = max(0.0, min(line_len, proj_length))
    nearest = [x1 + line_unitvec[0] * proj_length, y1 + line_unitvec[1] * proj_length]
    dist = ((px - nearest[0]) ** 2 + (py - nearest[1]) ** 2) ** 0.5
    return dist

def is_in_obstacle_space(x, y):
    if x < clearance or y < clearance or x > map_width - clearance or y > map_height - clearance:
        return True
    for polygon in obstacles:
        if within_clearance(x, y, polygon, clearance):
            return True
    return False

def dijkstra(start, goal):
    actions = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.4), (-1, -1, 1.4), (1, -1, 1.4), (-1, 1, 1.4)]
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    cost_so_far = {start: 0}
    visited = []

    while open_list:
        current_cost, current = heapq.heappop(open_list)
        visited.append(current)

        if current == goal:
            break

        for dx, dy, move_cost in actions:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] <= map_width and 0 <= neighbor[1] <= map_height:
                new_cost = cost_so_far[current] + move_cost
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    if not is_in_obstacle_space(neighbor[0], neighbor[1]):
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost
                        heapq.heappush(open_list, (priority, neighbor))
                        came_from[neighbor] = current

    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from.get(current, start)
    path.append(start)
    path.reverse()

    return path, visited

def animate_search(visited):
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.set_xlim(0, map_width)
    ax.set_ylim(0, map_height)

    for polygon in obstacles:
        poly = plt.Polygon(polygon, facecolor="gray", edgecolor='black')
        ax.add_patch(poly)

    points = ax.scatter([], [], s=1, color='blue') 

    def init():
        points.set_offsets(np.empty((0, 2))) 
        return points,

    def update(frame):
        skip = 1500
        frame *= skip
        visited_points = np.array(visited[:frame+1])
        points.set_offsets(visited_points)
        return points,

    ani = FuncAnimation(fig, update, frames=len(visited), init_func=init, blit=True, interval=1)
    plt.show()

def animate_path(path):
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.set_xlim(0, map_width)
    ax.set_ylim(0, map_height)

    for polygon in obstacles:
        poly = plt.Polygon(polygon, facecolor="gray", edgecolor='black')
        ax.add_patch(poly)

    line, = ax.plot([], [], 'b-', lw=2)  # Path line

    def init():
        line.set_data([], [])
        return line,

    def update(frame):
        skip = 20
        frame *= skip
        x, y = zip(*path[:frame+1])
        line.set_data(x, y)
        return line,

    ani = FuncAnimation(fig, update, frames=len(path), init_func=init, blit=True, interval=50)
    plt.show()

start_time = time.time()
start = (6, 6)
goal = (1194, 162)
path, visited = dijkstra(start, goal)
time_finish = time.time() - start_time

print(f"Goal found in {math.floor(time_finish/60)} minutes and {(time_finish % 60):.2f} seconds")

animate_search(visited)
animate_path(path)