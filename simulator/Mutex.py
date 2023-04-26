#Mutex

import numpy as np 
from scipy import spatial
import random 
import queue 
import heapq 
from CBS_low_level import LowLevelCBS
from MDD import GetMDD
import networkx as nx

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


def GetMutexes(mdd_dict): 

    mdd1 = mdd_dict[1]
    mdd2 = mdd_dict[2] 
    # for node, attr in mdd1.nodes(data=True): 
    #     print(node,attr) 
    # print(mdd1.nodes(data=True)) 
    start = tuple([node for node in mdd1.nodes if mdd1.nodes.get(node, {}).get('level') == 0])

    for i, nodes1 in enumerate(nx.bfs_tree(mdd1, source=start)): #set initial mutexes 
        for node in nodes1:
            if node in mdd2.nodes and mdd2.nodes[node].get('level', -1) == i: 
                mdd1.nodes[node]['mutex'] = True
                mdd2.nodes[node]['mutex'] = True

    for G in mdd_dict.values(): #set propagated mutexes 
        # Iterate over the levels of the graph from the second level to the last level
        for i in range(1, max(nx.get_node_attributes(G, 'level').values()) + 1):
            level_nodes = [node for node in G.nodes if G.nodes[node].get('level', -1) == i]

            # Iterate over the nodes at the current level
            for node in level_nodes:
                parents_mutex = True
                parents = list(G.predecessors(node))

                # Check if all of the parent nodes are mutex
                for parent in parents:
                    if not G.nodes[parent].get('mutex', False):
                        parents_mutex = False
                        break

                # If all of the parent nodes are mutex, set the current node's mutex attribute to True
                if parents_mutex:
                    G.nodes[node]['mutex'] = True
    
    return mdd_dict



def CheckCardinalConflict(mdd_mutex_dict): #returns True if the goal nodes of the agents are mutex, which means they have cardinal conflict  
    last_nodes = []
    
    for G in mdd_mutex_dict.values(): 
        last_level_node = [node for node in G.nodes if G.nodes[node]['level'] == max(nx.get_node_attributes(G, 'level').values())][-1]
        if not G.nodes[last_level_node]['mutex']:
            return False 
    
    return True 


def GetConstraints(mdd_dict):
    """
    Iterate over all the agents in the mdd_dict, create a list A_constraints for each agent, and check all the nodes 
    from pre-final layer to the first layer of that agent's MDD. If a node from MDD of agent 1 is mutex with all the MDD 
    nodes from MDD2 of the same level, then add the constraint as a tuple (agent1, node1, node1_level) to A_constraints. 
    Once done with all the nodes of the MDD of agent 1,append the list A_constraints to another list All_A_constraints 
    which was created before iterating over all agents of mdd_dict. Once the constraints of all agents have been collected 
    in the mdd_dict, return the list All_A_constraints.
    """
    All_A_constraints = []
    agents = list(mdd_dict.keys())
    
    for i, agent1 in enumerate(agents):
        A_constraints = []
        
        for node1 in reversed(list(nx.topological_sort(mdd_dict[agent1]))):
            node1_level = mdd_dict[agent1].nodes[node1]['level']
            node1_mutex = True
            
            for agent2 in agents[:i] + agents[i+1:]:
                for node2 in mdd_dict[agent2]:
                    if mdd_dict[agent2].nodes[node2]['level'] == node1_level:
                        if not mdd_dict[agent1].nodes[node1].get('mutex', False) or not mdd_dict[agent2].nodes[node2].get('mutex', False):
                            node1_mutex = False
                            break
                if not node1_mutex:
                    break
            
            if node1_mutex:
                A_constraints.append((agent1, node1, node1_level))
                
        All_A_constraints.append(A_constraints)
    
    return All_A_constraints



def Mutex(grid_pygame): #{agent number1:[start1,goal1],agent number2:[start2,goal2]}. Returns solution with a path per agent, otherwise returns None 
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
    mdd_dict = {}
    constraints = [] #vertex constraint: (ai,v,t) => constraints = [(ai,v,t),...]
    for agent in agent_dict:
        print(agent) 
        mdd_dict[agent] = GetMDD(grid,agent_dict[agent][0],agent_dict[agent][1]) #Get MDD of agent 1 and agent 2 and store it in mdd_dict  

    mdd_mutex_dict = GetMutexes(mdd_dict) #get the updated dict with 

    if(CheckCardinalConflict(mdd_mutex_dict)):
        All_A_constraints = GetConstraints(mdd_mutex_dict)

    for agent in agent_dict:
        path[agent] = LowLevelCBS(grid,agent_dict[agent][0],agent_dict[agent][1],All_A_constraints[agent-1],agent)

    print(grid)
    print(agent_dict)

    pathFound = False
    for pathKey in path:
        if len(path[pathKey]):
            pathFound = True
    if(not pathFound): return False,None,None 


#Implementation:
#agent_dict = {agent number1:[start1,goal1],agent number2:[start2,goal2]}
#solution = HighLevelCBS(agent_dict) #Returns solution with a path per agent, otherwise returns None
            


        