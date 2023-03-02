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


def GiveConflict(solution): #checks for vertex conflicts and returns the first conflict it finds in the form (ai,aj,v,t)
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
            v = solution[agent][i] #position where the conflict arises 
            t = i  #time step of conflict 
            return (conflict_agents[0],conflict_agents[1],v,t) #return the conflict in the form (ai,aj,v,t)

        i += 1
    return None #if no conflict has been found return None 


def HighLevelCBS(grid,agent_dict): #{agent number1:[start1,goal1],agent number2:[start2,goal2]}. Returns solution with a path per agent, otherwise returns None 
    path = {}
    constraints = [] #vertex constraint: (ai,v,t) => constraints = [(ai,v,t),...]
    for agent in agent_dict:
        path[agent] = LowLevelCBS(grid,agent_dict[agent][0],agent_dict[agent][1],constraints,agent)

    idx = 0 
    Root = Node(constraints,path)   
    Root.index = 0 

    pq = []
    heapq.heappush(pq,Root) 

    while pq:
        P = heapq.heappop(pq) #pop node from the priority queue and make it as parent 
        C = GiveConflict(P.solution) 
        if(C==None): 
            return P.solution,P.cost
    

        for i in range(2):
            A_constraints = P.constraints.copy()
            A_solution = P.solution.copy() #child node sol = parent node sol (will be updated)
            A_constraints = A_constraints + [(C[i], C[2], C[3])] #child node constraints = parent node constraints + constraints as per conflict
            A = Node(A_constraints,A_solution) #initiate child node 
            idx = idx + 1 
            A.index = idx #assign index for priority queue purpose 
            A.solution[C[i]] = LowLevelCBS(grid,agent_dict[C[i]][0],agent_dict[C[i]][1],A.constraints,C[i]) #update solution of A 
            if(A.solution != None):
                heapq.heappush(pq,A)

    return None 


#Implementation:
#agent_dict = {agent number1:[start1,goal1],agent number2:[start2,goal2]}
#solution = HighLevelCBS(agent_dict) #Returns solution with a path per agent, otherwise returns None
            


        