from pygame.locals import *
import pygame
from grid import *
from pggrid import *

grid = Grid(4,5)

# Set the HEIGHT and WIDTH of the screen
pgGrid = PGGrid(grid)
 
# Set title of screen
pygame.display.set_caption("MAPF Simulator")
 
# Loop until the user clicks the close button.
running = True

while running:
    running = pgGrid.tick(120) #120 frames per second
 
# Be IDLE friendly. If you forget this line, the program will 'hang'
# on exit.
pygame.quit()