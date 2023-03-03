from pygame.locals import *
import pygame
from grid import *
from pggrid import *

grid = Grid(4,4)
agent1 = AgentCell(1)
agent1goal = AgentGoal(agent1.getColor(),agent1.getAgentID())
agent2 = AgentCell(2)
agent2goal = AgentGoal(agent2.getColor(),agent2.getAgentID())

grid.setCell((1,2),Obstacle())
grid.setCell((1,0), agent1)
grid.setCell((1,3), agent1goal)
grid.setCell((0,0), agent2)
grid.setCell((2,3), agent2goal)

print(grid.getMap().tolist())
print(grid.getAgentMap().tolist())
print(grid.getAgentGoalMap().tolist())
start_list = grid.getAgentMap().tolist()
goal_list = grid.getAgentGoalMap().tolist()

# i = 0
# for row in grid.getAgentMap().tolist():
#     for elem in row:
#         print(i,elem)
#         i += 1

agent_dict = {}
i = 0
for row_s in start_list:
    j = 0
    for elem_s in row_s:
        if(elem_s>0):
            agent_dict[elem_s] = [[i,j]]
        j += 1
    i += 1

k = 0
for row_g in goal_list:
    l = 0
    for elem_g in row_g:
        if(elem_g>0):
            agent_dict[elem_g].append([k,l])
        l += 1
    k += 1

agent_dict = {k: agent_dict[k] for k in sorted(agent_dict, key=lambda x: x)}
print(agent_dict)

# Set the HEIGHT and WIDTH of the screen
pgGrid = PGGrid(grid)
 
# Set title of screen
pygame.display.set_caption("MAFP Simulator")
 
# Loop until the user clicks the close button.
running = True

while running:
    running = pgGrid.tick(120) #120 frames per second
 
# Be IDLE friendly. If you forget this line, the program will 'hang'
# on exit.
pygame.quit()