from grid import *
from colorsys import rgb_to_hsv, hsv_to_rgb
import pygame

def complementary(color):
   """returns RGB components of complementary color"""
   r,g,b = color
   hsv = rgb_to_hsv(r, g, b)
   return hsv_to_rgb((hsv[0] + 0.5) % 1, hsv[1], hsv[2])

class GridDisplay():
    def __init__(self, grid, windowSize):
        self.screen = pygame.display.set_mode(windowSize, pygame.RESIZABLE)
        self.grid = grid
        self.cellSize = 20
        self.marginSize = 5
        self.agentFont = pygame.font.SysFont('Arial', int(self.cellSize*0.6), bold=True)
        self.agentGoalFont = pygame.font.SysFont('Arial', int(self.cellSize*0.6), bold=False, italic=True)
        self.fitCellSizeAndMargin()

    def fitCellSizeAndMargin(self):
        gridMap = self.grid.getMap()
        girdRowSize = len(gridMap)
        gridColSize = len(gridMap[0])
        displaySize = min(self.screen.get_width(), self.screen.get_height()) - self.marginSize
        cellPlusMargin = displaySize//min(girdRowSize, gridColSize)
        self.cellSize = cellPlusMargin - self.marginSize
        self.agentFont = pygame.font.SysFont('Arial', int(self.cellSize*0.6), bold=True)
        self.agentGoalFont = pygame.font.SysFont('Arial', int(self.cellSize*0.6), bold=False, italic=True)

    def updateGrid(self, newGrid):
        self.grid = newGrid

    def xy2GridCoords(self, pos):
        row = pos[1]//(self.cellSize + self.marginSize)
        col = pos[0]//(self.cellSize + self.marginSize)

        return (row,col)

    def renderGridDisplay(self):
        self.fitCellSizeAndMargin()
        gridColors = self.grid.getGridColors()
        cellsTypeMap = self.grid.getCellsType()
        agentsMap = self.grid.getAgentMap()
        agentsGoalMap = self.grid.getAgentGoalMap()

        self.screen.fill((0,0,0))
        for row in range(len(gridColors)):
            for col in range(len(gridColors[0])):
                pygame.draw.rect(self.screen,
                             gridColors[row][col],
                             [(self.marginSize + self.cellSize) * col + self.marginSize,
                              (self.marginSize + self.cellSize) * row + self.marginSize,
                              self.cellSize,
                              self.cellSize])
                
                if cellsTypeMap[row][col] == AGENT_CELL:
                    BG_color = gridColors[row][col]
                    text_color = complementary(BG_color)
                    fontImg = self.agentFont.render(str(agentsMap[row][col]), True,
                                                            pygame.Color(text_color),
                                                            pygame.Color(BG_color))
                    self.screen.blit(fontImg, ((self.marginSize + self.cellSize) * col + self.marginSize,
                                        (self.marginSize + self.cellSize) * row + self.marginSize))
                
                if cellsTypeMap[row][col] == AGENT_GOAL:
                    BG_color = gridColors[row][col]
                    text_color = complementary(BG_color)
                    fontImg = self.agentGoalFont.render(str(agentsGoalMap[row][col]), True,
                                                            pygame.Color(text_color),
                                                            pygame.Color(BG_color))
                    self.screen.blit(fontImg, ((self.marginSize + self.cellSize) * col + self.marginSize,
                                        (self.marginSize + self.cellSize) * row + self.marginSize))
        
        return self.screen