from pyamaze import maze, agent, COLOR
from queue import PriorityQueue
import math
import os
import time

# Global variable for start and goal points
start_point = (15, 17)  # (row, col)
goal_point = (3, 2)     # (row, col)

# """ Heuristic function """
def M(cell1, cell2):
    """ Heuristic function (Manhattan Distance) """
    x1, y1 = cell1
    x2, y2 = cell2
    return abs(x1 - x2) + abs(y1 - y2) 

def E(cell1, cell2):
    """ Heuristic function (Euclidean Distance) """
    x1, y1 = cell1
    x2, y2 = cell2
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

R = M  # Change to M for Manhattan, E for Euclidean Distance


# ==== Load or Create Maze ====
def load_or_create_maze():
    # Loads a maze from CSV if available, or creates a new maze and saves it. 
    m = maze(15, 20)
    if os.path.exists('maze.csv'):
        print("Loading saved maze from CSV...")
        m.CreateMaze(3, 2, loadMaze="maze.csv", theme=COLOR.dark) 
    else:
        print("Creating new maze and saving to CSV...")
        m.CreateMaze(3, 2, loopPercent=100, theme=COLOR.dark, saveMaze='maze.csv') 
    return m


# A* Search Algorithms

def aStar(m):
    #  A* Search Algorithm to find the shortest path from start to goal. Visualises progress by moving the agent step by step.
    # The algorithm uses a priority queue to explore the most promising nodes first based on their f_score values.
    start = start_point 
    # Initialise g_score and f_score for all cells in the grid
    g_score = {cell: float('inf') for cell in m.grid}
    g_score[start] = 0
    f_score = {cell: float('inf') for cell in m.grid}
    f_score[start] = R(start, goal_point) # Initial heuristic estimate

    open_set = PriorityQueue() # Creates a priority queue called open_set, which is used to store cells to be explored based on their f_score values.
    open_set.put((f_score[start], start))
    aPath = {}
    visited_cells = 0
    

    # Set up the agent to move inside the maze
    a = agent(m, start[0], start[1], footprints=True, color=COLOR.green, shape='square')

    while not open_set.empty():  # Priority queue is not empty
        currCell = open_set.get()[1] # Get the cell with the lowest f_score
        visited_cells += 1  # Increment visited cells count

        m.tracePath({a: [currCell]}, delay=100)  # Update agent's position
        time.sleep(0.1)  # Visualization delay to slow down the animation


        if currCell == goal_point:
            break  # Stop when the goal is reached
        

        # Check all 4 possible directions from the current cell
        for d in 'ESNW':  # E=East, S=South, N=North, W=West
            if m.maze_map[currCell][d]: # Check if the direction is valid (not a wall)
                if d == 'E':
                    childCell = (currCell[0], currCell[1] + 1)
                elif d == 'W':
                    childCell = (currCell[0], currCell[1] - 1)
                elif d == 'S':
                    childCell = (currCell[0] + 1, currCell[1])
                else:
                    childCell = (currCell[0] - 1, currCell[1])
                

                # Calculate the new scores for the neighbour
                temp_g_score = g_score[currCell] + 1
                temp_f_score = temp_g_score + R(childCell, goal_point) # Heuristic estimate



                 # If this path is better, update the scores and add to the queue
                if temp_f_score < f_score[childCell]:
                    g_score[childCell] = temp_g_score
                    f_score[childCell] = temp_f_score
                    open_set.put((temp_f_score, childCell))
                    aPath[childCell] = currCell   # Remember how we got to childCell


    # Reconstruct path from the goal to start
    path = []
    if currCell == goal_point:
        cell = goal_point
        while cell != start:
            path.append(cell)
            cell = aPath[cell]
        path.reverse()  # Reverse the path to get it from start to goal
    return path, len(path), visited_cells



# ==============================
# Trace Final Path
# ==============================
def traceFinalPath(m, agent, path):
    # Trace the full path from start to goal with an agent after search is done.
    if not path:
        return # No path found
    
    # Move agent to the start position first
    m.tracePath({agent: [path[0]]}, delay=100, kill=False)

    # Trace each step of the path
    for i in range(1, len(path)):
        m.tracePath({agent: [path[i]]}, delay=100, kill=False)
        time.sleep(0.1)





# ==============================
# Main Function
# ==============================

if __name__ == "__main__":
    m = load_or_create_maze()  # Load or create the maze
    
    # Run A* Search and get the results
    path_AStar, steps_AStar, visited_cells = aStar(m)

    print(f"A* Path Length: {steps_AStar}")
    print(f"Visited Cells: {visited_cells}")

    # Set up a new agent to draw the final path
    agent_AStar = agent(m, start_point[0], start_point[1], footprints=True, color=COLOR.cyan, shape='arrow')

    # Show the full final path
    traceFinalPath(m, agent_AStar, path_AStar)

    # Run the maze GUI
    m.run()