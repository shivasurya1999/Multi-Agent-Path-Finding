class Agent:
    def __init__(self,agentID,path):
        self.agentID = agentID
        self.start = path[0]
        self.goal = path[-1]
        self.path = path
        self.s_t = 0
        self.g_t = self.getPathLen()

    def getPathLen(self):
        return len(self.path)-1
    
    def getManhattenDist(self):
        return abs(self.start[0] - self.goal[0]) + abs(self.start[1] - self.goal[1])

def isRectangle(agent1: Agent, agent2: Agent)->bool:
    """
    Given paths of a pair of agents having a vertex conflict,
    this fucntion checks if the agent pair has a cardinal rectangle conflict or not

    i/p: paths

    o/p: True/False
    """

    #check1: is the path optimal, i.e is path length and manhatten distance from Start to Goal equal
    if agent1.getManhattenDist() == agent1.getPathLen() & agent2.getManhattenDist() == agent2.getPathLen():
        check1 = True
    else: check1 = False

    #check2: the distances from each location inside the rectangle to the locations of the two start nodes are equal
    if (agent1.start[0] - agent2.start[0])*(agent1.goal[0] - agent2.goal[0])<=0 & \
        (agent1.start[1] - agent2.start[1])*(agent1.goal[1] - agent2.goal[1])<=0:
        check2 = True
    else: check2 = False

    #check3: start and goal nodes have opposite relative locations in both dimensions
    if (agent1.start[0] - agent2.start[0])*(agent1.goal[0] - agent2.goal[0])<=0 & \
        (agent1.start[1] - agent2.start[1])*(agent1.goal[1] - agent2.goal[1])<=0:
        check3 = True
    else: check3 = False

    if check1 & check2 & check3:
        return True
    else: 
        return False

def getVertices(agent1: Agent,agent2: Agent):
    ### compute cardinal rectangle edge node co-ordinates
    ## calculate Rs
    # Rs_x
    if agent1.start[0] == agent1.goal[0]:
        Rs_x = agent1.start[0]
    elif agent1.start[0] < agent1.goal[0]:
        Rs_x = max(agent1.start[0], agent2.start[0])
    else:
        Rs_x = min(agent1.start[0], agent2.start[0])
    # Rs_y
    if agent1.start[1] == agent1.goal[1]:
        Rs_y = agent1.start[1]
    elif agent1.start[1] < agent1.goal[1]:
        Rs_y = max(agent1.start[1], agent2.start[1])
    else:
        Rs_y = min(agent1.start[1], agent2.start[1])
    # Rs_t
    Rs_t = agent1.s_t + abs(agent1.start[0] - Rs_x) + abs(agent1.start[1] - Rs_y)

    Rs = [Rs_x,Rs_y,Rs_t]

    ## calculate Rg
    # Rg_x
    if agent1.start[0] == agent1.goal[0]:
        Rg_x = agent1.goal[0]
    elif agent1.start[0] < agent1.goal[0]:
        Rg_x = min(agent1.goal[0],agent2.goal[0])
    else:
        Rg_x = max(agent1.goal[0],agent2.goal[0])
    # Rg_y
    if agent1.start[1] == agent1.goal[1]:
        Rg_y = agent1.goal[1]
    elif agent1.start[1] < agent1.goal[1]:
        Rg_y = min(agent1.goal[1],agent2.goal[1])
    else:
        Rg_y = max(agent1.goal[1],agent2.goal[1])
    # Rg_t
    Rg_t = agent1.s_t + abs(agent1.start[0] - Rg_x) + abs(agent1.start[1] - Rg_y)
    
    Rg = [Rg_x,Rg_y,Rg_t]
    
    ## calculate Ri, Rj
    if (agent1.start[0] - agent2.start[0])*(agent2.start[0] - Rg_x) >=0: 
        Ri_x = Rg_x
        Ri_y = agent1.start[1]
        Rj_x = agent2.start[0]
        Rj_y = Rg_y
    else:
        Ri_x = agent1.start[0]
        Ri_y = Rg_y
        Rj_x = Rg_x
        Rj_y = agent2.start[1]
    # Ri_t
    Ri_t = agent1.s_t + abs(agent1.start[0] - Ri_x) + abs(agent1.start[1] - Ri_y)
    # Rj_t
    Rj_t = agent1.s_t + abs(agent1.start[0] - Rj_x) + abs(agent1.start[1] - Rj_y)
    
    Ri = [Ri_x,Ri_y,Ri_t]
    Rj = [Rj_x,Rj_y,Rj_t]

    return Rs,Rg,Ri,Rj

def getBarrierConstraint(agentID: int, Rk: tuple, Rg: tuple)->list:
    B = []
    row = Rk[0]
    col = Rk[1]
    t = Rk[2]
    if Rk[0] == Rg[0]: # Rk and Rg on same row
        # Rk on the left of Rg
        if Rk[1] < Rg[1]: 
            for i in range(abs(Rk[1]-Rg[1]) + 1):
                B.append((agentID,[row,col + i],t + i))
        # Rk on the right of Rg
        else:             
            for i in range(abs(Rk[1]-Rg[1]) + 1):
                B.append((agentID,[row,col - i],t + i))

    if Rk[1] == Rg[1]: # Rk and Rg on same col
        # Rk above Rg
        if Rk[0] < Rg[0]:
            for i in range(abs(Rk[0]-Rg[0]) + 1):
                B.append((agentID,[row + i,col],t + i))
        # Rk below Rg
        else:
            for i in range(abs(Rk[0]-Rg[0]) + 1):
                B.append((agentID,[row - i,col],t + i))
    return B



# def SymmetryBreaking(vertex_conflict: tuple, path1: list, path2: list):
#     """
#     vertex_conflict -> (ai,aj,v,t)
#     """
#     agent1 = Agent(vertex_conflict[0],path1)
#     agent2 = Agent(vertex_conflict[1],path2)
#     constraints = []
#     if isRectangle(agent1,agent2):
#         Rs,Rg,Ri,Rj = getVertices(agent1,agent2)
#         constraints = getBarrierConstraint(agent1.agentID,Ri,Rg)
#     return constraints

# path1 = [(1,0,0),(2,0,1),(2,1,2),(2,2,3),(2,3,4),(2,4,5)]
# path2 = [(0,1,0),(0,2,1),(0,3,2),(1,3,3),(2,3,4),(3,3,5)]
# conflict = (1,2,(2,3),4)
# SymmetryBreaking(conflict,path1,path2)