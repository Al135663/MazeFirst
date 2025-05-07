from pyamaze import maze, agent, COLOR
from queue import PriorityQueue
import math
import os
import time

start_point = (15, 17)  # (row, col)
goal_point = (3, 2)     # (row, col)



####################################################

def load_or_create_maze():
    ## Loads a maze from 'maze.csv' if it exists. Otherwise, it creates a new maze, saves it as 'maze.csv', and returns it.
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
    ## Breadth-First Search (BFS) algorithm for finding the shortest path. Visualises the search in real time with an agent moving step by step.
    start = start_point  
    frontier = [start] # Queue to hold cells to explore next (FIFO)
    explored = {start} # Set to keep track of visited cells
    bfsPath = {}       # Dictionary to keep track of how each cell was reached

    # Set up the agent that moves through the maze for visualization
    a = agent(m, start[0], start[1], footprints=True, color=COLOR.red, shape='square')
    
    while frontier:
        currCell = frontier.pop(0) # Get the next cell from the front of the queue (FIFO)
        m.tracePath({a: [currCell]}, delay=100)  # Show agent moving to the current cell
        time.sleep(0.1)  # Add a small delay for better visualisation
        if currCell == goal_point:
            break  # Stop if the goal is reached
        

         # Check all 4 possible directions (East, South, North, West)
        for d in 'ESNW':
            if m.maze_map[currCell][d]:  # Check if movement is possible in direction 'd'
                if d == 'E':
                    childCell = (currCell[0], currCell[1] + 1)
                elif d == 'W':
                    childCell = (currCell[0], currCell[1] - 1)
                elif d == 'S':
                    childCell = (currCell[0] + 1, currCell[1])
                else:
                    childCell = (currCell[0] - 1, currCell[1])
                

                ## If the neighbouring cell has not been explored yet, add it to the frontier
                if childCell not in explored:
                    explored.add(childCell)
                    frontier.append(childCell)
                    bfsPath[childCell] = currCell  # Record how this cell was reached


    # Reconstruct path from goal to start
    path = []
    if currCell == goal_point:
        cell = goal_point
        while cell != start:
            path.append(cell)
            cell = bfsPath[cell]
        path.reverse() # Reverse to get the path from start to goal

    return path, len(path), len(explored), explored # Return path, path length, number of visited cells, and explored set


#####################################################
# Main code execution

if __name__ == "__main__":
    m = load_or_create_maze() # Load or create the maze
    

    # Generate paths
    path_BFS, steps_BFS, _, explored = BFS(m)
    
    # Print out BFS path length and number of visited cells
    print(f"BFS Path Length: {steps_BFS}")
    print(f"BFS Visited Cells: {len(explored)}")
    
    # Set up a new agent to draw the final BFS path in a different colour and shape
    agent_BFS = agent(m, start_point[0], start_point[1], footprints=True, color=COLOR.yellow, shape='arrow')
    
    # Visualise the final shortest path found by BFS
    m.tracePath({agent_BFS: path_BFS}, delay=100)
    
    # Run the maze window to display everything
    m.run()