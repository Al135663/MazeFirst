from pyamaze import maze, agent, COLOR
from queue import PriorityQueue
import math
import os
import time

# Global variables
start_point = (15, 17)  # (row, col)
goal_point = (3, 2)     # (row, col)


####################################################

def load_or_create_maze():
    m = maze(15, 20)
    if os.path.exists('maze.csv'):
        print("Loading saved maze from CSV...")
        
        m.CreateMaze(3, 2, loadMaze="maze.csv", theme=COLOR.dark) 
    else:
        print("Creating new maze and saving to CSV...")
        m.CreateMaze(3, 2, loopPercent=100, theme=COLOR.dark, saveMaze='maze.csv') 
    return m

# DFS Search Algorithm
def DFS(m):
    """ Depth-First Search (LIFO) """
    start = start_point  
    explored = [start]
    frontier = [start]
    dfsPath = {}
    
    a = agent(m, start[0], start[1], footprints=True, color=COLOR.blue, shape='square')

    while frontier:
        currCell = frontier.pop()  # pop from the end
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
    return path, len(path), len(explored), explored



#####################################################

if __name__ == "__main__":
    m = load_or_create_maze()
    
    # Generate paths   
    path_DFS, steps_DFS, _,explored = DFS(m)
    
    print(f"DFS Path Length: {steps_DFS}")
    print(f"DFS Visited Cells: {len(explored)}")

    # Create agents for final paths with custom shapes    
    agent_DFS = agent(m, start_point[0], start_point[1], footprints=True, color=COLOR.black, shape='arrow')
   
    # Trace final paths    
    m.tracePath({agent_DFS: path_DFS}, delay=100)
    
    m.run()