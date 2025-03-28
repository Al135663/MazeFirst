from pyamaze import maze, agent, COLOR
from queue import PriorityQueue
import math
import time

start_point = (13, 17)  # (row, col)
goal_point = (3, 2)     # (row, col)

def H(cell1, cell2):
    """ Heuristic function (Manhattan Distance) """
    x1, y1 = cell1
    x2, y2 = cell2
    return abs(x1 - x2) + abs(y1 - y2)

def E(cell1, cell2):
    """ Heuristic function (Euclidean Distance) """
    x1, y1 = cell1
    x2, y2 = cell2
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

R = E  # Change to H for Manhattan, E for Euclidean Distance

def BFS(m):
    """ Breadth-First Search (FIFO) """
    start = start_point  
    frontier = [start]
    explored = {start}
    bfsPath = {}

    a = agent(m, start[0], start[1], footprints=True, color=COLOR.red, shape='square')
    
    while frontier:
        currCell = frontier.pop(0)
        m.tracePath({a: [currCell]}, delay=100)  # Update agent's position
        time.sleep(0.1)  # Visualization delay
        if currCell == goal_point:
            break
        
        for d in 'ESNW':
            if m.maze_map[currCell][d]:
                if d == 'E':
                    childCell = (currCell[0], currCell[1] + 1)
                elif d == 'W':
                    childCell = (currCell[0], currCell[1] - 1)
                elif d == 'S':
                    childCell = (currCell[0] + 1, currCell[1])
                else:
                    childCell = (currCell[0] - 1, currCell[1])
                
                if childCell not in explored:
                    explored.add(childCell)
                    frontier.append(childCell)
                    bfsPath[childCell] = currCell

    # Reconstruct path
    path = []
    if currCell == goal_point:
        cell = goal_point
        while cell != start:
            path.append(cell)
            cell = bfsPath[cell]
        path.reverse()
    return path, len(path)


####################################################


def DFS(m):
    """ Depth-First Search (LIFO) """
    start = start_point  
    explored = [start]
    frontier = [start]
    dfsPath = {}
    
    a = agent(m, start[0], start[1], footprints=True, color=COLOR.blue, shape='square')

    while frontier:
        currCell = frontier.pop()
        m.tracePath({a: [currCell]}, delay=100)  # Update agent's position
        time.sleep(0.1)  # Visualization delay
        if currCell == goal_point:
            break
        
        for d in 'ESNW':
            if m.maze_map[currCell][d]:
                if d == 'E':
                    childCell = (currCell[0], currCell[1] + 1)
                elif d == 'W':
                    childCell = (currCell[0], currCell[1] - 1)
                elif d == 'S':
                    childCell = (currCell[0] + 1, currCell[1])
                else:
                    childCell = (currCell[0] - 1, currCell[1])
                
                if childCell not in explored:
                    explored.append(childCell)
                    frontier.append(childCell)    
                    dfsPath[childCell] = currCell

    # Reconstruct path
    path = []
    if currCell == goal_point:
        cell = goal_point
        while cell != start:
            path.append(cell)
            cell = dfsPath[cell]
        path.reverse()
    return path, len(path)

####################################################

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
    
    a = agent(m, start[0], start[1], footprints=True, color=COLOR.green, shape='square')

    while not open_set.empty():
        currCell = open_set.get()[1]
        m.tracePath({a: [currCell]}, delay=100)  # Update agent's position
        time.sleep(0.1)  # Visualization delay
        if currCell == goal_point:
            break
        
        for d in 'ESNW':
            if m.maze_map[currCell][d]:
                if d == 'E':
                    childCell = (currCell[0], currCell[1] + 1)
                elif d == 'W':
                    childCell = (currCell[0], currCell[1] - 1)
                elif d == 'S':
                    childCell = (currCell[0] + 1, currCell[1])
                else:
                    childCell = (currCell[0] - 1, currCell[1])
                
                temp_g_score = g_score[currCell] + 1
                temp_f_score = temp_g_score + R(childCell, goal_point)

                if temp_f_score < f_score[childCell]:
                    g_score[childCell] = temp_g_score
                    f_score[childCell] = temp_f_score
                    open_set.put((temp_f_score, childCell))
                    aPath[childCell] = currCell

    # Reconstruct path
    path = []
    if currCell == goal_point:
        cell = goal_point
        while cell != start:
            path.append(cell)
            cell = aPath[cell]
        path.reverse()
    return path, len(path)

def traceFinalPath(m, agent, path):
    """ Trace the final path from start to goal """
    if not path:
        return
    # Move agent to the start position first
    m.tracePath({agent: [path[0]]}, delay=100, kill=False)
    # Trace each step of the path
    for i in range(1, len(path)):
        m.tracePath({agent: [path[i]]}, delay=100, kill=False)
        time.sleep(0.1)

#####################################################

if __name__ == "__main__":
    m = maze(15, 20)
    m.CreateMaze(goal_point[0], goal_point[1], loopPercent=100, theme=COLOR.dark)

    # Generate paths
    path_BFS, steps_BFS = BFS(m)
    path_DFS, steps_DFS = DFS(m)
    path_AStar, steps_AStar = aStar(m)

    print(f"BFS Path Length: {steps_BFS}")
    print(f"DFS Path Length: {steps_DFS}")
    print(f"A* Path Length: {steps_AStar}")

    # Create agents for final paths with custom shapes
    agent_BFS = agent(m, start_point[0], start_point[1], footprints=True, color=COLOR.yellow, shape='arrow')
    agent_DFS = agent(m, start_point[0], start_point[1], footprints=True, color=COLOR.black, shape='triangle')
    agent_AStar = agent(m, start_point[0], start_point[1], footprints=True, color=COLOR.cyan, shape='classic')

    # Trace final paths
    traceFinalPath(m, agent_BFS, path_BFS)
    traceFinalPath(m, agent_DFS, path_DFS)
    traceFinalPath(m, agent_AStar, path_AStar)

    m.run()