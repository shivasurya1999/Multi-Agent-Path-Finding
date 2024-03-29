import numpy as np

EMPTY_CELL = 0
OBSTACLE = 1
AGENT_CELL = 2
AGENT_GOAL = 3
class Cell:
    def __init__(self, color = (255,255,255)):
        self.color = color
    
    def getColor(self):
        return self.color
    
    def getType(self):
        return EMPTY_CELL

class Obstacle(Cell):
    def __init__(self):
        super().__init__((0,0,0))
    
    def getType(self):
        return OBSTACLE

class AgentCell(Cell):
    def __init__(self, agentID):
        color = tuple(np.random.choice(range(256), size=3))
        super().__init__(color)
        self.agentID = agentID
    
    def getType(self):
        return AGENT_CELL
    
    def getAgentID(self):
        return self.agentID
    
class AgentGoal(Cell):
    def __init__(self, color, agentID):
        super().__init__(color)
        self.agentID = agentID
    
    def getType(self):
        return AGENT_GOAL
    
    def getAgentID(self):
        return self.agentID

class Grid:
    def __init__(self, rowSize, colSize, obstacleMap = None):
        if obstacleMap is not None:
            self.grid = [[Cell() for j in range(colSize)] for i in range(rowSize)]
            for i in range(len(obstacleMap)):
                for j in range(len(obstacleMap[0])):
                    if obstacleMap[i][j] == OBSTACLE:
                        self.setCell((i,j), Obstacle())
        else:
            self.grid = [[Cell() for j in range(colSize)] for i in range(rowSize)]

    def getCell(self, pos):
        row,col = pos
        return self.grid[row][col]
    
    def setCell(self, pos, CellToSet):
        row,col = pos
        self.grid[row][col] = CellToSet

    def getGridColors(self):
        colors = np.zeros((len(self.grid),len(self.grid[0]),3),dtype=np.uint8)
        for row in range(len(self.grid)):
            for col in range(len(self.grid[0])):
                colors[row][col] = np.uint8(self.grid[row][col].getColor())

        return colors
    
    def getCellsType(self):
        cellsType = np.zeros((len(self.grid),len(self.grid[0])),dtype=np.uint16)
        for row in range(len(self.grid)):
            for col in range(len(self.grid[0])):
                cellsType[row][col] = np.uint16(self.grid[row][col].getType())

        return cellsType
    
    def getMap(self):
        gridMap = np.zeros((len(self.grid),len(self.grid[0])),dtype=np.uint8)
        for row in range(len(self.grid)):
            for col in range(len(self.grid[0])):
                if self.grid[row][col].getType() == EMPTY_CELL or self.grid[row][col].getType() == OBSTACLE:
                    gridMap[row][col] = np.uint8(self.grid[row][col].getType())

        return gridMap

    def getAgentMap(self):
        agentMap = np.zeros((len(self.grid),len(self.grid[0])),dtype=np.uint16)
        for row in range(len(self.grid)):
            for col in range(len(self.grid[0])):
                if self.grid[row][col].getType() == AGENT_CELL:
                    agentMap[row][col] = np.uint16(self.grid[row][col].getAgentID())

        return agentMap

    def getAgentGoalMap(self):
        agentMap = np.zeros((len(self.grid),len(self.grid[0])),dtype=np.uint16)
        for row in range(len(self.grid)):
            for col in range(len(self.grid[0])):
                if self.grid[row][col].getType() == AGENT_GOAL:
                    agentMap[row][col] = np.uint8(self.grid[row][col].getAgentID())

        return agentMap
    
    def getAgentsInfo(self):
        start_list = self.getAgentMap().tolist()
        goal_list = self.getAgentGoalMap().tolist()

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
        
        return agent_dict

