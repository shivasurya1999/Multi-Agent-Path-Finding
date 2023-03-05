import numpy as np 
from scipy import spatial
import random 
import queue 
import heapq 
from CBS_low_level import LowLevelCBS

"""
We have 2 agents (n agents in general), each with a start position, and a goal position 

We need to create a constraint tree (CT) with set of paths for each agent independent of the other agent (invoke low level)
Append these paths to the solution attribute of the root node of the CT
Calculate the cost of root node by adding individual costs of the solution set obtained from previous step 
Root node has no constraints, so we initiate that to empty 
Insert root to priority queue

While priority queue is not empty 
    Pop least cost node from priority queue
    Check for conflicts by initiating one pointer for each agent and compare the locations at those pointers to check for conflicts 
    If no conflict, return the solution of the popped node with its cost 
    Incase of first conflict C = (ai,aj,v,t)

    for each agent in C
        Create a node A that inherits parent's constraints plus the new constraint of that agent wrt conflict 
        A.solution = Parent's solution
        Update A.solution invoking low level solution of agent 
        Calculate the cost of A from the new A.solution 
        If cost is not infinity, insert A into priority queue 
"""

def SIC(solution): #i/p: dict of key as agent and value as list of the path coordinates, returns sum of all agent costs 
    cost = 0
    for path in solution:
        cost += len(solution[path])
    return cost  


class Node:
    def __init__(self,constraints,solution):
        self.constraints = constraints
        self.solution = solution  #dict of all paths. a dict with key as agent number and value as the path of that agent 
        self.cost = SIC(self.solution) #sum of individual solution costs 
        self.index = None 

    def __lt__(self,other):
        if self.cost == other.cost:
            return self.index < other.index
        return self.cost<other.cost 


def GiveVertexConflict(solution): #checks for vertex conflicts and returns the first conflict it finds in the form (ai,aj,v,t)
    i = 0 
    max_len = max(len(lst) for lst in solution.values())
    while i < max_len:
        positions = []
        for agent in solution:
            if(len(solution[agent]) > i):
                positions.append(solution[agent][i]) #append position for all agents at index i 
            else:
                positions.append(solution[agent][-1]) #the agent that has reached its goal stays there indefinitely 
                # positions.append([10000*random.random(),10000*random.random()]) #append a large number so that the kdtree ignores this 

        #if we manage to find neighbours with r=0, it means that there is collision between agents at that time step (index i)
        r = 0
        kdtree = spatial.KDTree(positions)
        pairs = list(kdtree.query_pairs(r))

        if(len(pairs) > 0):
            conflict_agents = list(pairs[0])
            conflict_agents = [1+conflict_agents[0],1+conflict_agents[1]] #to get correct number of the agent 
            try:
                v = solution[conflict_agents[0]][i] #position where the conflict arises 
            except IndexError: #for agents with non-longest path 
                v = solution[conflict_agents[0]][-1] #position where the conflict arises 

            t = i  #time step of conflict 
            return (conflict_agents[0],conflict_agents[1],v,t) #return the conflict in the form (ai,aj,v,t)

        i += 1
    return None #if no conflict has been found return None 


def GiveEdgeConflict(solution):
    """
    Given a solution dictionary containing paths for multiple robots, this function finds the first edge conflict
    between two robots in the solution.

    Args:
    - solution (dict): A dictionary containing paths for multiple robots, where each key represents a robot and
                       the corresponding value is a list of (x, y) coordinates representing the path of the robot.

    Returns:
    - A tuple representing the first edge conflict found in the solution, consisting of:
      * The keys of the two robots involved in the conflict (as integers).
      * The (x, y) coordinates of the edge where the conflict occurred.
      * The time step (as an integer) when the conflict occurred.

    If no edge conflicts are found in the solution, the function returns None.

    """

    #print(solution)
    # Iterate through all pairs of robot paths in the solution
    for key in solution.keys():
        if(key+1>len(solution)): break
        for i in range(len(solution[key])-1):
            for j in range(len(solution)-key):
                # Check if the two paths share a common edge at the current time step
                if(len(solution[key+j+1])>i+1):
                    if((solution[key][i]==solution[key+j+1][i+1]) and
                    (solution[key+j+1][i]==solution[key][i+1])):
                        # If a conflict is found, return the conflict details
                        return (key,key+j+1,solution[key][i],solution[key][i+1],i)
    
    # If no conflicts are found, return None
    return None
 

def GiveConflict(solution):
    """
    Given a solution to a Multi-Agent Path Finding (MAPF) problem, this function returns the first conflict found in the solution,
    whether it is a vertex conflict or an edge conflict. Vertex conflicts occur when two or more agents occupy the same vertex
    at the same time, while edge conflicts occur when two or more agents cross the same edge at the same time.
    
    Args:
    - solution: a dictionary that represents the solution to the MAPF problem. The dictionary has agents' ids as keys and lists
                of (x,y,t) tuples as values, where (x,y) is the position of the agent at time t.
    
    Returns:
    - a tuple that represents the first conflict found in the solution. If the conflict is a vertex conflict, the tuple has the
      form (ai,aj,v,t), where ai and aj are the ids of the agents that conflict at vertex v at time t. If the conflict is an edge
      conflict, the tuple has the form (ai,aj,v1,v2,t), where ai and aj are the ids of the agents that conflict at the edge
      between vertices v1 and v2 at time t.
    """
    vertex_conflict = GiveVertexConflict(solution) #(ai,aj,v,t)
    edge_conflict = GiveEdgeConflict(solution) #(ai,aj,v1,v2,t)
    #print(vertex_conflict,edge_conflict)
    
    # Determine which conflict occurs first based on the time of the conflict
    if vertex_conflict is not None and edge_conflict is not None:
        if vertex_conflict[3] <= edge_conflict[4]:
            return vertex_conflict
        else:
            return edge_conflict
    elif vertex_conflict is not None:
        return vertex_conflict
    elif edge_conflict is not None:
        return edge_conflict
    else:
        return None



def HighLevelCBS(grid_pygame): #{agent number1:[start1,goal1],agent number2:[start2,goal2]}. Returns solution with a path per agent, otherwise returns None 
    grid = grid_pygame.getMap().tolist()
    start_list = grid_pygame.getAgentMap().tolist()
    goal_list = grid_pygame.getAgentGoalMap().tolist()

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

    path = {}
    constraints = [] #vertex constraint: (ai,v,t) => constraints = [(ai,v,t),...]
    for agent in agent_dict:
        path[agent] = LowLevelCBS(grid,agent_dict[agent][0],agent_dict[agent][1],constraints,agent)

    pathFound = False
    for pathKey in path:
        if len(path[pathKey]):
            pathFound = True
    if(not pathFound): return False,None,None 

    idx = 0 
    Root = Node(constraints,path)   
    Root.index = 0 

    pq = []
    heapq.heappush(pq,Root) 

    while pq:
        P = heapq.heappop(pq) #pop node from the priority queue and make it as parent 
        C = GiveConflict(P.solution) 
        if(C==None): 
            return True,P.solution,P.cost
    

        for i in range(2):
            A_constraints = P.constraints.copy()
            A_solution = P.solution.copy() #child node sol = parent node sol (will be updated)
            if(len(C)==5): #incase of edge conflict 
                A_constraints = A_constraints + [(C[i],C[i+2],C[4])] #imposing constraints (ai,v1,t),(aj,v2,t)
                #print(A_constraints)
            else:
                A_constraints = A_constraints + [(C[i], C[2], C[3])] #child node constraints = parent node constraints + constraints as per conflict
                #print(A_solution)

            A = Node(A_constraints,A_solution) #initiate child node 
            idx = idx + 1 
            A.index = idx #assign index for priority queue purpose 
            A.solution[C[i]] = LowLevelCBS(grid,agent_dict[C[i]][0],agent_dict[C[i]][1],A.constraints,C[i]) #update solution of A 
            if(A.solution != None):
                heapq.heappush(pq,A)

    return False,None,None 


#Implementation:
#agent_dict = {agent number1:[start1,goal1],agent number2:[start2,goal2]}
#solution = HighLevelCBS(agent_dict) #Returns solution with a path per agent, otherwise returns None
            


        