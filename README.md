# ENPM661_Project2
 Implementation of the Dijkstra Algorithm for a Point Robot
README file (.md or .txt)
○ Must describe how to run the code and give inputs (start and goal coordinates)


Description:
Implementing Dijkstra's algorithm to navigate a point robot from a specified starting position to an ending position in an environment containing obstacles. The goal is to ensure that the robot avoids contact with obstacles while achieving successful navigation using Dijkstra's algorithm.

Prerequisites:
Python 3.8+

Libraries:
NumPy
heapq
matplotlib.pyplot 
matplotlib.animation.FuncAnimation
math
time

Features:
1. Find optimal path from start to goal points, avoids contact with obstacles with clearance distance. 
2. Visualization of exploration process.
3. Visualization of optimal path.

Running the Code:
1. Input start x, start y, goal x, goal y, coordinates.
* do not input start and goal points excess map (1200,500), or within obstacles.
2. After solution found, will automatically play exploration process.
3. close exploration process, will automatically play optimal path from start to goal points.

Contact:
WEI-LI, CHEN (Willie) wc2023@umd.edu