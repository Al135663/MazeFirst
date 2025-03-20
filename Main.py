from pyamaze import maze, agent, COLOR
from collections import deque
from queue import PriorityQueue
import math

start_point = (8, 10)  # (row, col) - Change this to any valid cell
goal_point = (3, 3)  # (row, col) - Change this to any valid cell

def h(cell1, cell2):
    """ Heuristic function (Manhattan Distance) """
    x1, y1 = cell1
    x2, y2 = cell2
    return abs(x1 - x2) + abs(y1 - y2)

####################################

""" Heuristic function (Euclidean Distance) """
def e(cell1, cell2):
    x1, y1 = cell1
    x2, y2 = cell2
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

####################################

R=e   # Change this to e for Euclidean Distance or h for Manhattan Distance.

####################################  
def BFS(m):
    """ Breadth-First Search (FIFO) """
    start = start_point  
    frontier = [start]
    explored = [start]
    bfsPath = {}

    while frontier:
        currCell = frontier.pop(0)  
        if currCell == goal_point:
            break
        for d in 'ESNW':
            if m.maze_map[currCell][d]:
                childCell = (currCell[0] + (d == 'S') - (d == 'N'), 
                             currCell[1] + (d == 'E') - (d == 'W'))
                
                if childCell in explored:
                    continue
                
                explored.append(childCell)
                frontier.append(childCell)
                bfsPath[childCell] = currCell

    fwdPath = {}
    cell = goal_point
    steps = 0
    while cell != start:
        fwdPath[bfsPath[cell]] = cell
        cell = bfsPath[cell]
        steps += 1
    return fwdPath, steps

####################################  
def DFS(m):
    """ Depth-First Search (LIFO) """
    start = start_point  
    explored = [start]
    frontier = [start]
    dfsPath = {}

    while frontier:
        currCell = frontier.pop()  
        if currCell == goal_point:
            break
        for d in 'ESNW':
            if m.maze_map[currCell][d]:
                childCell = (currCell[0] + (d == 'S') - (d == 'N'), 
                             currCell[1] + (d == 'E') - (d == 'W'))
                
                if childCell in explored:
                    continue
                
                explored.append(childCell)
                frontier.append(childCell)    
                dfsPath[childCell] = currCell

    fwdPath = {}
    cell = goal_point
    steps = 0
    while cell != start:
        fwdPath[dfsPath[cell]] = cell
        cell = dfsPath[cell]
        steps += 1
    return fwdPath, steps

####################################  
def aStar(m):
    """ A* Search Algorithm """
    start = start_point  
    g_score = {cell: float('inf') for cell in m.grid}
    g_score[start] = 0
    f_score = {cell: float('inf') for cell in m.grid}
    f_score[start] = R(start, goal_point)

    open_set = PriorityQueue()
    open_set.put((f_score[start], start))
    
    aPath = {}

    while not open_set.empty():
        currCell = open_set.get()[1]
        if currCell == goal_point:
            break
        for d in 'ESNW':
            if m.maze_map[currCell][d]:
                childCell = (currCell[0] + (d == 'S') - (d == 'N'), 
                             currCell[1] + (d == 'E') - (d == 'W'))
                
                temp_g_score = g_score[currCell] + 1
                temp_f_score = temp_g_score + R(childCell, goal_point)

                if temp_f_score < f_score[childCell]:
                    g_score[childCell] = temp_g_score
                    f_score[childCell] = temp_f_score
                    open_set.put((temp_f_score, childCell))
                    aPath[childCell] = currCell

    fwdPath = {}
    cell = goal_point
    steps = 0
    while cell != start:
        fwdPath[aPath[cell]] = cell
        cell = aPath[cell]
        steps += 1
    return fwdPath, steps

####################################  
if __name__ == "__main__":
    m = maze(10, 15)
    m.CreateMaze(goal_point[0], goal_point[1], loopPercent=50, theme=COLOR.dark)  

    # Generate paths and count steps for each algorithm
    path_BFS, steps_BFS = BFS(m)
    path_DFS, steps_DFS = DFS(m)
    path_AStar, steps_AStar = aStar(m)

    # Print steps taken by each agent
    print(f"BFS Steps: {steps_BFS}")
    print(f"DFS Steps: {steps_DFS}")
    print(f"A* Steps: {steps_AStar}")

    # Create three agents with different colors
    agent_BFS = agent(m, start_point[0], start_point[1], footprints=True, color=COLOR.red)      
    agent_DFS = agent(m, start_point[0], start_point[1], footprints=True, color=COLOR.blue)     
    agent_AStar = agent(m, start_point[0], start_point[1], footprints=True, color=COLOR.green)  

    # Trace the paths for each agent
    m.tracePath({agent_BFS: path_BFS})
    m.tracePath({agent_DFS: path_DFS})
    m.tracePath({agent_AStar: path_AStar})

    m.run()