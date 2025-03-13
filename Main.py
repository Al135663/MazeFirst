from pyamaze import maze, agent, COLOR

def DFS(m):
    start = (m.rows, m.cols)  # Start point is the bottom-right corner
    explored = [start]
    frontier = [start]

    dfsPath = {}

    while len(frontier) > 0:
        currCell = frontier.pop()
        if currCell == (1, 1):  # Defining the goal cell
            break
        for d in 'NWSE': # Order of possible direction
            if m.maze_map[currCell][d] == True:
                if d == 'E':
                    childCell = (currCell[0], currCell[1] + 1)
                elif d == 'W':
                    childCell = (currCell[0], currCell[1] - 1)
                elif d == 'S':
                    childCell = (currCell[0] + 1, currCell[1])  
                elif d == 'N':
                    childCell = (currCell[0] - 1, currCell[1]) 
                if childCell in explored:
                    continue
                explored.append(childCell)
                frontier.append(childCell)    
                dfsPath[childCell] = currCell

    fwdPath = {}
    cell = (1, 1)  # Start with the goal cell top-left
    while cell != start:
        fwdPath[dfsPath[cell]] = cell 
        cell = dfsPath[cell]  # Move backwards in the path

    return fwdPath

m = maze(10, 17)
m.CreateMaze()
m.agent=COLOR.red
path = DFS(m)
a = agent(m, footprints=True, color=COLOR.red)
m.tracePath({a: path})

m.run()