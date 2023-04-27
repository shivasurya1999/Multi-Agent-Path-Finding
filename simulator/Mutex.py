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
"""

def SIC(solution): #i/p: dict of key as agent and value as list of the path coordinates, returns sum of all agent costs 
    cost = 0
    for path in solution:
        cost += len(solution[path])
    return cost  

def isallMutex(parent1, parents2, mdd1):
    parent1_mutexes = mdd1.nodes.get(parent1, {}).get('mutex')

    if((not parent1_mutexes)or(not parents2)):
        return False  

    for parent1_mutex in parent1_mutexes:
        if not any(parent1_mutex in parent2 for parent2 in parents2):
            return False  # mutex not found in any of the parents2

    return True  # all mutexes found in at least one of the parents2


def GetMutexes(mdd_dict):
    # Get the two MDDs from the input dictionary
    mdd1 = mdd_dict[1]
    mdd2 = mdd_dict[2] 

    # Set initial mutexes between nodes at the same level in the two MDDs
    levels1 = [data['level'] for _, data in mdd1.nodes(data=True)]
    levels2 = [data['level'] for _, data in mdd2.nodes(data=True)]
    max_level1 = max(levels1, default=0)
    max_level2 = max(levels2,default=0)
    min_level = min(max_level1,max_level2)
    nx.set_node_attributes(mdd1, False, 'is_mutex')
    nx.set_node_attributes(mdd1, [], 'mutex') 
    nx.set_node_attributes(mdd2, False, 'is_mutex')
    nx.set_node_attributes(mdd2, [], 'mutex')
    for level in range(0, min_level + 1):
        # Get the nodes at the current level for each MDD
        level_nodes1 = [node for node in mdd1.nodes if mdd1.nodes.get(node, {}).get('level') == level]
        level_nodes2 = [node for node in mdd2.nodes if mdd2.nodes.get(node, {}).get('level') == level]
        print("level") 
        print(level)
        print("level_nodes1")
        print(level_nodes1)
        print("level_nodes2")
        print(level_nodes2)
        for node1 in level_nodes1: 
            for node2 in level_nodes2:
                if(node1==node2): 
                    mdd1.nodes[node1]['is_mutex'] = True 
                    mdd1.nodes[node1]['mutex'].append(node2)
                    mdd2.nodes[node2]['is_mutex'] = True 
                    mdd2.nodes[node2]['mutex'].append(node1)
                    print("Got initial mutexes at level",level,"node1=",node1,"node2=",node2)

    # Propagate the mutexes to higher levels 
    for level in range(1, min_level + 1):
        # Get the nodes at the current level for each MDD 
        level_nodes1 = [node for node in mdd1.nodes if mdd1.nodes.get(node, {}).get('level') == level]
        level_nodes2 = [node for node in mdd2.nodes if mdd2.nodes.get(node, {}).get('level') == level]

        # Iterate over pairs of nodes at the current level
        for node1 in level_nodes1: 
            for node2 in level_nodes2:
                # Check if all parents of the two nodes are mutex
                parents1 = list(mdd1.predecessors(node1))
                parents2 = list(mdd2.predecessors(node2))
                # print("The parents at level ",level," for node1  ",node1," are parent1: ",parents1,"and for node2",node2,"parent2 ",parents2)
                # Check if all parents of node1 are mutex with all parents of node2 
                for parent1 in parents1:
                    if(isallMutex(parent1,parents2,mdd1)): 
                        # If all parents are mutex, set the current nodes to be mutex and update their mutex pointers
                        mdd1.nodes[node1]['is_mutex'] = True
                        mdd1.nodes[node1]['mutex'].append(node2)
                        mdd2.nodes[node2]['is_mutex'] = True
                        mdd2.nodes[node2]['mutex'].append(node1)
                        print("Got propagated mutexes at level",level,"node1=",node1,"node2=",node2)
    
        # for node1 in level_nodes1:
        #     print("node,level,mutex")
        #     print(node1,level,mdd1.nodes[node1]['mutex'])
        # for node1 in level_nodes2:
        #     print("node,level,mutex")
        #     print(node2,level,mdd2.nodes[node2]['mutex'])
    # Return the updated MDD dictionary
    return mdd_dict


def CheckCardinalConflict(mdd_mutex_dict):
    # Check if the goal nodes of the agents are mutex and return True in such case 
    
    last_nodes = []
    
    for G in mdd_mutex_dict.values():  # Iterate through all MDDs
        # Get the last level node of the MDD
        last_level_node = [node for node in G.nodes if G.nodes[node]['level'] == max(nx.get_node_attributes(G, 'level').values())][-1]
        print("last level node",last_level_node)
        # Check if the last node is mutex
        if not G.nodes.get(last_level_node, {}).get('is_mutex'):
            return False  # If not, return False (no conflict)
    
    return True  # If all last nodes are mutex, return True (cardinal conflict exists)



def GetConstraints(mdd_dict):
    """
    Iterate over all the agents in the mdd_dict, create a list A_constraints for each agent, and check all the nodes 
    from pre-final layer to the layer just after the start layer of that agent's MDD. If a node from MDD of agent 1 is mutex with all the MDD 
    nodes from MDD2 of the same level, then add the constraint as a tuple (agent1, node1, node1_level) to A_constraints. 
    Once done with all the nodes of the MDD of agent 1,append the list A_constraints to another list All_A_constraints 
    which was created before iterating over all agents of mdd_dict. Once the constraints of all agents have been collected 
    in the mdd_dict, return the list All_A_constraints.
    """
    All_A_constraints = []
    agents = list(mdd_dict.keys())

    for agent in agents:
        A_constraints = []
        mdd = mdd_dict[agent]

        # Get the levels of the MDD
        levels = list(set(nx.get_node_attributes(mdd, 'level').values()))
        levels.sort(reverse=True)

        # Check nodes from pre-final layer to just after the start layer
        for level in levels[:-1]:
            nodes1 = [node for node in mdd.nodes if mdd.nodes.get(node, {}).get('level') == level]
            nodes2 = [mdd.nodes[node]['mutex'] for node in nodes1]

            for node1 in nodes1:
                nodes2 = [mdd.nodes[node]['mutex'] for node in nodes1 if 'mutex' in mdd.nodes[node]]

                # Check if node1 is mutex with all nodes at the same level in other MDDs
                if all(mdd2.nodes.get(node2, {}).get('is_mutex') for mdd2 in mdd_dict.values() if mdd2 != mdd
                    for node2 in nodes2 if 'mutex' in mdd2.nodes.get(node2, {}) and mdd2.nodes.get(node2, {}).get('level') == level):
                        A_constraints.append((agent, node1, level))


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

    All_A_constraints = []
    print(CheckCardinalConflict(mdd_mutex_dict))
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
            


        