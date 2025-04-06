from pyamaze import maze, agent, COLOR
from queue import PriorityQueue
import math
import os
import time

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




   # """ Breadth-First Search (FIFO) """
def BFS(m):
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
    return path, len(path), len(explored), explored # Return path, path length, number of visited cells, and explored set


#####################################################

if __name__ == "__main__":
    m = load_or_create_maze()
    

    # Generate paths
    path_BFS, steps_BFS, _, explored = BFS(m)
    
    print(f"BFS Path Length: {steps_BFS}")
    print(f"BFS Visited Cells: {len(explored)}")
    
    # Create agents for final paths with custom shapes
    agent_BFS = agent(m, start_point[0], start_point[1], footprints=True, color=COLOR.yellow, shape='arrow')
    
    # Trace final paths
    m.tracePath({agent_BFS: path_BFS}, delay=100)
    

    m.run()