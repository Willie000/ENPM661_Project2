import numpy as np
import heapq
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
import time

clearance = 5
map_width = 1200
map_height = 500

# obstacles
obstacles = [
    [(100, 100), (100, 500), (175, 500), (175, 100)],

    [(275, 0), (275, 400), (350, 400), (350, 0)],

    [(650-150*np.cos(np.pi/6), 400-150-150*0.5),
     (650-150*np.cos(np.pi/6), 400-150*0.5),
     (650, 400),
     (650+150*np.cos(np.pi/6), 400-150*0.5),
     (650+150*np.cos(np.pi/6), 400-150-150*0.5),
     (650, 100)],

    [(900, 450), (1100, 450), (1100, 50), (900, 50), (900, 125), (1020, 125), (1020, 375), (900, 375)]
]

#check whether within clearance distance
def within_clearance(x, y, polygon, clearance):
    for i in range(len(polygon)):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % len(polygon)]
        if point_line_dist(x, y, p1[0], p1[1], p2[0], p2[1]) < clearance:
            return True
    return False

#calcualte shortest distance to line
def point_line_dist(px, py, x1, y1, x2, y2):
    line_len = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
    line_unitvec = [(x2 - x1) / line_len, (y2 - y1) / line_len]
    proj_length = (px - x1) * line_unitvec[0] + (py - y1) * line_unitvec[1]
    proj_length = max(0.0, min(line_len, proj_length))
    nearest = [x1 + line_unitvec[0] * proj_length, y1 + line_unitvec[1] * proj_length]
    dist = ((px - nearest[0]) ** 2 + (py - nearest[1]) ** 2) ** 0.5
    return dist

#make sure search within maps
def is_in_obstacle_space(x, y):
    if x < clearance or y < clearance or x > map_width - clearance or y > map_height - clearance:
        return True
    for polygon in obstacles:
        if within_clearance(x, y, polygon, clearance):
            return True
    return False

#use dijkstra to find solution path
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

#show search process
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
        skip = 3000
        frame *= skip
        visited_points = np.array(visited[:frame+1])
        points.set_offsets(visited_points)
        return points,

    ani = FuncAnimation(fig, update, frames=len(visited), init_func=init, blit=True, interval=1)
    plt.show()

#show solution path
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

#start = (6, 6) #start point 
#goal = (1194, 162) # Goal point
start_x = int(input("start x "))
start_y = int(input("start y "))
goal_x = int(input("goal x "))
goal_y = int(input("goal y "))
start = (start_x, start_y) #start point 
goal = (goal_x, goal_y) # Goal point
start_time = time.time()
path, visited = dijkstra(start, goal) #Use dijkstra to find solution
time_finish = time.time() - start_time
print(f"Goal found in {math.floor(time_finish/60)} minutes and {(time_finish % 60):.2f} seconds")

animate_search(visited) #show search process
animate_path(path) #show solution path