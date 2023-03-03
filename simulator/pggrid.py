import pygame
from grid import *
from grid_display import *
# import sys
# import os
# includePath = os.getcwd() + "/../CBS"
# print(includePath)
# sys.path.insert(1, includePath)
from CBS_high_level import *


IDLE = 0
ADD_OBSTACLE = 1
REMOVE_OBSTACLE = 2
ADD_AGENT = 3
REMOVE_AGENT = 4
ADD_GOAL = 5
REMOVE_GOAL = 6
CALCULATE_PATH = 7
ANIMATE_PATH = 8

class PGGrid():
    def __init__(self, grid):
        self.grid = grid
        pygame.init()
        WINDOW_SIZE = [1000, 1000]
        self.gridDisplay = GridDisplay(self.grid,WINDOW_SIZE)
        self.state = IDLE
        self._onClick = None
        self._onKeyPress = None
        self.currentAgentID = None
        self.calcThread = None

        self.clock = pygame.time.Clock()
    
    def clearGrid(self):
        gridMap = self.grid.getMap()
        rowSize = len(gridMap)
        colSize = len(gridMap[0])
        self.grid = Grid(rowSize,colSize)
        self.gridDisplay.updateGrid(self.grid)

    def onClick(self)->None :
        xyPos = pygame.mouse.get_pos()
        gridPos = self.gridDisplay.xy2GridCoords(xyPos)
        if self.state == IDLE:
            self.state = ADD_OBSTACLE

        cellsTypeMap = self.grid.getCellsType()
        cellType = cellsTypeMap[gridPos[0]][gridPos[1]]
        print("Current State = " + str(self.state))
        if self.state == ADD_OBSTACLE:
            if cellType == EMPTY_CELL:
                self.grid.setCell(gridPos, Obstacle())
        elif self.state == ADD_AGENT:
            if cellType == EMPTY_CELL:
                agentMap = self.grid.getAgentMap()
                agentList = agentMap[np.nonzero(agentMap)]
                if agentList.size > 0:
                    maxAgentID = np.max(agentList)
                else:
                    maxAgentID = 0
                agentID = maxAgentID + 1
                agent = AgentCell(agentID)
                self.grid.setCell(gridPos, agent)
                self.state = ADD_GOAL
                self.currentAgentID = agentID
                self.currentAgentColor = agent.getColor()
        elif self.state == ADD_GOAL:
            if cellType == EMPTY_CELL:
                if self.currentAgentID:
                    self.grid.setCell(gridPos, AgentGoal(self.currentAgentColor, self.currentAgentID))
                    self.currentAgentID = None
                    self.currentAgentColor = None
                    self.state = ADD_AGENT
        elif self.state == REMOVE_OBSTACLE:
            if cellType == OBSTACLE:
                print("Remove obstacle")
                self.grid.setCell(gridPos,Cell())
            else:
                print("Cannpt remove obstacle because current cell is not an obstacle")
        elif self.state == REMOVE_AGENT:
            agentCellMap = self.grid.getAgentMap()
            agentGoalMap = self.grid.getAgentGoalMap()
            if cellType == AGENT_CELL:
                agentID = agentCellMap[gridPos[0]][gridPos[1]]
                x,y = np.where(agentGoalMap == agentID)
                agentGoalCoord = (x[0],y[0])
                print(agentGoalCoord)
                self.grid.setCell(gridPos,Cell())
                self.grid.setCell(agentGoalCoord,Cell())
            elif cellType == AGENT_GOAL:
                agentID = agentGoalMap[gridPos[0]][gridPos[1]]
                x,y = np.where(agentCellMap == agentID)
                agentCellCoord = (x[0],y[0])
                self.grid.setCell(agentCellCoord,Cell())
                self.grid.setCell(gridPos,Cell())
        
        self.gridDisplay.updateGrid(self.grid)



    def onKeyPress(self,key)->None :
        print(key)
        if key == pygame.K_ESCAPE:
            self.clearGrid()
            self.state = IDLE
        if key == pygame.K_o:
            self.state = ADD_OBSTACLE
        if key == pygame.K_a:
            self.state = ADD_AGENT
        if key == pygame.K_r:
            if self.state == ADD_OBSTACLE:
                self.state = REMOVE_OBSTACLE
            elif self.state == ADD_AGENT:
                self.state = REMOVE_AGENT
        if key == pygame.K_c:
            self.state = CALCULATE_PATH
            print("Calculating Path")
            self.calculatePath()
            print("Finished Calculating Path")
        if key == pygame.K_s:
            if self.foundPath:
                self.state = ANIMATE_PATH
                print("Animating Path")
                self.animatePath()
                print("Finished animating Path")
            else:
                print("No Path Found, Cannot Animate")
            
    def calculatePath(self):
        self.foundPath, self.pathDict, self.pathCost = HighLevelCBS(self.grid)

    def animatePath(self):
        longestPathSize = 0
        print(self.pathDict)
        for pathItem in self.pathDict:
            print(pathItem)
            if longestPathSize < len(self.pathDict[pathItem]):
                longestPathSize = len(self.pathDict[pathItem])
        
        agentCellsDict = {}
        gridObstacleMap = self.grid.getMap()
        agentCellMap = self.grid.getAgentMap()
        agentGoalMap = self.grid.getAgentGoalMap()

        agentStartXs, agentStartYs = np.where(agentCellMap != 0)
        for x in agentStartXs:
            for y in agentStartYs:
                agentCell = self.grid.getCell((x,y))
                agentGoalx, agentGoaly = np.where(agentGoalMap == agentCell.getAgentID())
                agentGoal = self.grid.getCell((agentGoalx[0], agentGoaly[0]))
                agentCellsDict[agentCell.getAgentID()] = (agentCell, agentGoal, (agentGoalx[0], agentGoaly[0]))
        
        
        for i in range(longestPathSize):
            gridToVisualize = Grid(len(gridObstacleMap), len(gridObstacleMap[0]), gridObstacleMap)
            for pathItem in self.pathDict:
                agentID = pathItem
                path = self.pathDict[pathItem]
                agentCellInfos = agentCellsDict.get(agentID)
                agentCell = agentCellInfos[0]
                agentGoal = agentCellInfos[1]
                agentGoalCoord = agentCellInfos[2]

                if i > len(path) - 1:
                    gridToVisualize.setCell(agentGoalCoord, agentGoal)
                    gridToVisualize.setCell(path[-1], agentCell)
                else:
                    gridToVisualize.setCell(path[i], agentCell)
                    gridToVisualize.setCell(agentGoalCoord, agentGoal)

            self.gridDisplay.updateGrid(gridToVisualize)
            self.clock.tick(1)
            self.gridDisplay.renderGridDisplay()
            pygame.display.update()
        
    
    def tick(self, maxframes:int)->bool:
        for event in pygame.event.get():
            if event.type == pygame.QUIT : pygame.quit(); return False
            if event.type == pygame.KEYDOWN:
                self.onKeyPress(event.key)
            if event.type == pygame.MOUSEBUTTONDOWN:
                self.onClick()
        
        self.gridDisplay.renderGridDisplay()
        pygame.display.update()

        self.clock.tick(maxframes)
        return True