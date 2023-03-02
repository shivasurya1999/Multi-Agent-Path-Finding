import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import csv
import time

from CBS_high_level import HighLevelCBS


# Load map, start and goal point.
def load_map(file_path):
    grid = []
    start1 = [0, 0]
    goal1 = [0, 0]
    start2 = [0, 0]
    goal2 = [0, 0]
    # Load from the file
    with open(file_path, 'r') as map_file:
        reader = csv.reader(map_file)
        for i, row in enumerate(reader):
            # load start and goal point
            if i == 0:
                start1[0] = int(row[1])
                start1[1] = int(row[2])
            elif i == 1:
                start2[0] = int(row[1])
                start2[1] = int(row[2])
            elif i == 2:
                goal1[0] = int(row[1])
                goal1[1] = int(row[2])
            elif i == 3:
                goal2[0] = int(row[1])
                goal2[1] = int(row[2])
            # load the map
            else:
                int_row = [int(col) for col in row]
                grid.append(int_row)
    return grid, start1, start2, goal1, goal2



def draw_path(grid, paths, title="Path"):
    # Visualization of the found paths using matplotlib
    fig, ax = plt.subplots(1)
    ax.margins()
    
    # Draw map
    row = len(grid)
    col = len(grid[0])
    for i in range(row):
        for j in range(col):
            if grid[i][j]:
                ax.add_patch(Rectangle((j-0.5, i-0.5),1,1,edgecolor='k',facecolor='k'))  # obstacle
            else:
                ax.add_patch(Rectangle((j-0.5, i-0.5),1,1,edgecolor='k',facecolor='w'))  # free space
    
    # Draw paths up to each timestep
    for t in range(max(len(path) for path in paths.values())):
        for agent, path in paths.items():
            if t < len(path):
                x, y = path[t]
                ax.add_patch(Rectangle((y-0.5, x-0.5),1,1,edgecolor='k',facecolor=f'C{agent}'))
    
    # Draw start and goal points
    for agent, path in paths.items():
        start = path[0]
        goal = path[-1]
        ax.add_patch(Rectangle((start[1]-0.5, start[0]-0.5),1,1,edgecolor='k',facecolor=f'C{agent}'))
        ax.add_patch(Rectangle((goal[1]-0.5, goal[0]-0.5),1,1,edgecolor='k',facecolor=f'C{agent}'))

    # Graph settings
    plt.title(title)
    plt.axis('scaled')
    plt.gca().invert_yaxis()
    plt.show()




if __name__ == "__main__":
    # Load the map
    grid, start1, start2, goal1, goal2 = load_map('test_map.csv')
    agent_dict = {1:[start1,goal1],2:[start2,goal2]}
    # Search
    cbs_solution,cbs_cost = HighLevelCBS(grid,agent_dict)
    print("solution:")
    print(cbs_solution)
    print("total cost:")
    print(cbs_cost)
    # Show the progress of each path at every timestep
    for t in range(max(len(paths) for paths in cbs_solution.values())):
        draw_path(grid, {agent: paths[:t+1] for agent, paths in cbs_solution.items()})
        time.sleep(1) # Add a delay to slow down the animation

    # Show the final solution
    draw_path(grid, cbs_solution)


