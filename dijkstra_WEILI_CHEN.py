# GitHub Repository Link:
# https://github.com/Willie000/ENPM661_Project2.git

import numpy as np
import heapq
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
import time

# map setups, mapsize and clearance
clearance = 5
map_width = 1200
map_height = 500

# map setups, obstacles
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

#Get vaild start and goal point
def get_input():
    while True:
        print("Please enter vaild Start and Goal point")
        start_x = int(input("start x "))
        start_y = int(input("start y "))
        goal_x = int(input("goal x "))
        goal_y = int(input("goal y "))

        if is_in_obstacle_space(start_x, start_y) or is_in_obstacle_space(goal_x, goal_y):
            print("srat or goal point is within obstacle or within clearance or outside the map")
        else:
            return (start_x, start_y), (goal_x, goal_y)


#check whether within clearance distance
def within_clearance(x, y, polygon, clearance):
    for i in range(len(polygon)): #every edge
        p1 = polygon[i] #edge point1
        p2 = polygon[(i + 1) % len(polygon)]#edge point2
        if point_line_dist(x, y, p1[0], p1[1], p2[0], p2[1]) < clearance:
            return True #robot to edge < clearance
    return False #robot to edge > clearance

#calcualte robot distance to nearest line
def point_line_dist(px, py, x1, y1, x2, y2):
    line_len = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5 #calculate edge length
    line_unitvec = [(x2 - x1) / line_len, (y2 - y1) / line_len] #calculate unit vector
    proj_length = (px - x1) * line_unitvec[0] + (py - y1) * line_unitvec[1] #project length
    proj_length = max(0.0, min(line_len, proj_length)) #project point within line
    nearest = [x1 + line_unitvec[0] * proj_length, y1 + line_unitvec[1] * proj_length] #project point
    dist = ((px - nearest[0]) ** 2 + (py - nearest[1]) ** 2) ** 0.5 #robot to project point distance
    return dist

#make sure search within maps, or in clearance
def is_in_obstacle_space(x, y):
    #check map edges
    if x < clearance or y < clearance or x > map_width - clearance or y > map_height - clearance:
        return True
    #check obstacles edges
    for polygon in obstacles:
        if within_clearance(x, y, polygon, clearance):
            return True
    return False

#use dijkstra to find solution path
def dijkstra(start, goal):
    actions = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.4), (-1, -1, 1.4), (1, -1, 1.4), (-1, 1, 1.4)]
    open_list = []
    heapq.heappush(open_list, (0, start)) #+start
    came_from = {} #track path
    cost_so_far = {start: 0} #record cost
    visited = [] #close

    while open_list:
        current_cost, current = heapq.heappop(open_list) #min cost node
        visited.append(current) #add to close 
        
        #if reached goal, finish dijkstra search
        if current == goal: 
            break
        
        #search for all actins
        for dx, dy, move_cost in actions:
            neighbor = (current[0] + dx, current[1] + dy) #calculate sourding nodes
            if 0 <= neighbor[0] <= map_width and 0 <= neighbor[1] <= map_height:
                new_cost = cost_so_far[current] + move_cost
                #in case new nodes or lower cost
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    if not is_in_obstacle_space(neighbor[0], neighbor[1]):
                        cost_so_far[neighbor] = new_cost #renew cost to near nodes
                        priority = new_cost #set as new cost
                        heapq.heappush(open_list, (priority, neighbor)) #add to open list
                        came_from[neighbor] = current #for track path

    #track back
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
    fig, ax = plt.subplots(figsize=(12, 5)) #set animate to 12:5 match map shape
    ax.set_xlim(0, map_width) #set animate x axis
    ax.set_ylim(0, map_height) #set animate y axis

    #show obstacles
    for polygon in obstacles:
        poly = plt.Polygon(polygon, facecolor="gray", edgecolor='black')
        ax.add_patch(poly)

    points = ax.scatter([], [], s=1, color='blue') 

    def init():
        points.set_offsets(np.empty((0, 2))) 
        return points,

    def update(frame):
        skip = 3000 #set flames skip
        frame *= skip 
        visited_points = np.array(visited[:frame+1]) #get visited
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
        skip = 20 #set flames skip
        frame *= skip
        x, y = zip(*path[:frame+1]) #get path
        line.set_data(x, y)
        return line,

    ani = FuncAnimation(fig, update, frames=len(path), init_func=init, blit=True, interval=50)
    plt.show()

start ,goal = get_input() # Get Start and Goal point
start_time = time.time()
path, visited = dijkstra(start, goal) #Use dijkstra to find solution
time_finish = time.time() - start_time
print(f"Goal found in {math.floor(time_finish/60)} minutes and {(time_finish % 60):.2f} seconds")

animate_search(visited) #show search process
animate_path(path) #show solution path