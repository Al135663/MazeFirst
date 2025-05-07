from pyamaze import maze, agent, COLOR
from queue import PriorityQueue
import math
import os
import time

# Global variables
start_point = (15, 17)  # (row, col)
goal_point = (3, 2)     # (row, col)



####################################################
# Function to load an existing maze from CSV or create a new one
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



################################################################
# DFS Search Algorithm
def DFS(m):
    """ Depth-First Search (LIFO) """
    start = start_point  
    explored = [start]  # List to keep track of visited cells
    frontier = [start]  # Stack (LIFO) for cells to visit next
    dfsPath = {}        # Dictionary to keep track of how each cell was reached
    

    # Set up the agent that moves through the maze to visualise DFS
    a = agent(m, start[0], start[1], footprints=True, color=COLOR.blue, shape='square')

    while frontier:
        currCell = frontier.pop()  # pop from the end
        m.tracePath({a: [currCell]}, delay=100)  # Update agent's position
        time.sleep(0.1)  # Visualization delay
        if currCell == goal_point:
            break # Stop if the goal is reached
        

        # Check all 4 directions (East, South, North, West)
        for d in 'ESNW':
            if m.maze_map[currCell][d]:   # Check if movement is possible in direction 'd'
                if d == 'E':
                    childCell = (currCell[0], currCell[1] + 1)
                elif d == 'W':
                    childCell = (currCell[0], currCell[1] - 1)
                elif d == 'S':
                    childCell = (currCell[0] + 1, currCell[1])
                else:
                    childCell = (currCell[0] - 1, currCell[1])
                

                 # If the neighbour has not been visited yet, add it to the stack
                if childCell not in explored:
                    explored.append(childCell) # Mark as visited
                    frontier.append(childCell) # Push onto the stack   
                    dfsPath[childCell] = currCell ## Record how we got to childCell

    # Reconstruct path from goal to start
    path = []
    if currCell == goal_point:
        cell = goal_point
        while cell != start:
            path.append(cell)
            cell = dfsPath[cell]
        path.reverse()  # Get the path from start to goal
    return path, len(path), len(explored), explored



#####################################################
# Main code execution

if __name__ == "__main__":
    m = load_or_create_maze() # Load or create the maze
    
    ## Run DFS and get results  
    path_DFS, steps_DFS, _,explored = DFS(m)
    
    # Print the path length and number of visited cells
    print(f"DFS Path Length: {steps_DFS}")
    print(f"DFS Visited Cells: {len(explored)}")

    # Set up a new agent to draw the final DFS path    
    agent_DFS = agent(m, start_point[0], start_point[1], footprints=True, color=COLOR.black, shape='arrow')
   
    # Visualise the final DFS path after the search
    m.tracePath({agent_DFS: path_DFS}, delay=100)
    
    # Run the maze GUI to display everything
    m.run()