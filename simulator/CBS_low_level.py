import queue 
import heapq

# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.g = None         # cost to come (previous g + moving cost)
        self.h = None            # heuristic
        self.cost = None      # total cost (depend on the algorithm)
        self.parent = None    # previous node
        self.index = None    # index to keep track of the order of objects
        self.time = None  #time at which the node has been expanded 

    def __lt__(self,other): #method that pops out the elements of priority queue as per their cost 
        if self.cost == other.cost: #tie breaker if the cost of two nodes is same 
            return self.index < other.index
        return self.cost < other.cost


def GetPathToGoal(node): #takes a node and returns a path from that node to the start node 
    path = []
    while node.parent: #keep looping until there is no parent to the node 
        path.append([node.row,node.col]) #append the node to path 
        node = node.parent
    path.append([node.row,node.col]) #append start node to path 
    path.reverse() #reverse the path because the start node should be first and the goal node at last 
    return path  #return the final path from start to the goal 


def GetHeuristic(node,goal): #returns the manhattan distance to goal as heuristic 
    n_row = node[0]
    n_col = node[1]
    g_row = goal[0]
    g_col = goal[1] 
    return abs(n_row-g_row) + abs(n_col-g_col)


def GetNeighbours(current_node,grid,idx,agent_constraints,agent): #takes a node and returns its right,down,left and up neighbours in order 

    current_time = current_node.time + 1
    neighbours = []
    rows = len(grid)
    cols = len(grid[0])
    ind = idx 

    #at the current_time we check if the (v,t) for any neighbour that we are about to expand is in agent_constraints
    #we flag such neighbour to False so that we do not expand it 
    right_flag = True 
    down_flag = True 
    left_flag = True 
    up_flag = True 
    wait_flag = True 
    for agent_constraint in agent_constraints:
        if(agent_constraint[2]==current_time):
            if([current_node.row,current_node.col+1]==agent_constraint[1]):
                right_flag = False
            if([current_node.row+1,current_node.col]==agent_constraint[1]):
                down_flag = False 
            if([current_node.row,current_node.col-1]==agent_constraint[1]):
                left_flag = False 
            if([current_node.row-1,current_node.col]==agent_constraint[1]):
                up_flag = False 
            if([current_node.row,current_node.col]==agent_constraint[1]): 
                wait_flag = False 
    

    if(0<=current_node.col+1<=cols-1): #if right neighbour exists in grid bounds 
        if((grid[current_node.row][current_node.col+1]==0)and(right_flag==True)): #if right neighbour is not an abstacle, has not been visited or flagged false 
            #create the right neighbour 
            ind = ind + 1
            right_row = current_node.row
            right_col = current_node.col + 1
            right_neighbour = Node(right_row,right_col,False)
            right_neighbour.parent = current_node
            right_neighbour.g = current_node.g + 1 #cost is 1 greater than the parent node's cost (no specific costs given in grid)
            right_neighbour.index = ind 
            right_neighbour.time = current_time
            neighbours.append(right_neighbour) #add it to the neighbours list 
            grid[right_row][right_col] = 2 #mark the node as visited 
            
    
    if(0<=current_node.row+1<=rows-1): #if down neighbour exists in grid bounds 
        if((grid[current_node.row+1][current_node.col]==0)and(down_flag==True)): #if down neighbour is not an abstacle, has not been visited or flagged false 
            #create the down neighbour 
            ind = ind + 1
            down_row = current_node.row + 1
            down_col = current_node.col 
            down_neighbour = Node(down_row,down_col,False)
            down_neighbour.parent = current_node
            down_neighbour.g = current_node.g + 1 #cost is 1 greater than the parent node's cost (no specific costs given in grid)
            down_neighbour.index = ind 
            down_neighbour.time = current_time
            neighbours.append(down_neighbour) #add it to the neighbours list
            grid[down_row][down_col] = 2 #mark the node as visited

    if(0<=current_node.col-1<=cols-1): #if left neighbour exists in grid bounds 
        if((grid[current_node.row][current_node.col-1]==0)and(left_flag==True)): #if left neighbour is not an abstacle, has not been visited or flagged false 
            #create the left neighbour 
            ind = ind + 1 
            left_row = current_node.row 
            left_col = current_node.col - 1
            left_neighbour = Node(left_row,left_col,False)
            left_neighbour.parent = current_node
            left_neighbour.g = current_node.g + 1 #cost is 1 greater than the parent node's cost (no specific costs given in grid)
            left_neighbour.index = ind 
            left_neighbour.time = current_time
            neighbours.append(left_neighbour) #add it to the neighbours list
            grid[left_row][left_col] = 2 #mark the node as visited

    if(0<=current_node.row-1<=rows-1): #if up neighbour exists in grid bounds 
        if((grid[current_node.row-1][current_node.col]==0)and(up_flag==True)): #if up neighbour is not an abstacle, has not been visited or flagged false 
            #create the up neighbour
            ind = ind + 1
            up_row = current_node.row - 1
            up_col = current_node.col 
            up_neighbour = Node(up_row,up_col,False)
            up_neighbour.parent = current_node
            up_neighbour.g = current_node.g + 1 #cost is 1 greater than the parent node's cost (no specific costs given in grid)
            up_neighbour.index = ind 
            up_neighbour.time = current_time
            neighbours.append(up_neighbour) #add it to the neighbours list
            grid[up_row][up_col] = 2  #mark the node as visited

    # #waiting at the current node
    if(wait_flag==True): #we wait only if waiting is not prohibited 
        ind = ind + 1
        wait_row = current_node.row 
        wait_col = current_node.col 
        wait_node = Node(current_node.row,current_node.col,False)
        wait_node.parent = current_node
        wait_node.g = current_node.g + 1
        wait_node.index = ind 
        wait_node.time = current_time
        neighbours.append(wait_node)

    return neighbours,ind #return the list of neighbours 


def LowLevelCBS(grid, start, goal,constraints,agent):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]
    constraints - constraints for finding the path (agent, vertex, time). e.g. (1,[1,2],5)
    agent - the agent number for which path is being found 

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###

    #we make sure that from the constraints we have we only consider the current agent's constraints 
    agent_constraints = []
    for constraint in constraints:
        if constraint[0] == agent:
            agent_constraints.append(constraint)


    path = []
    steps = 0
    found = False
    idx = 0
    time = 0 

    start_node = Node(start[0],start[1],False)
    start_node.g = 0
    start_node.h = 0
    start_node.cost = start_node.g + start_node.h
    start_node.parent = None 
    start_node.index = idx
    start_node.time = 0 

    pq = [] #initialize a priority queue 
    heapq.heappush(pq, start_node) #insert the start node into the queue
    grid[start_node.row][start_node.col] = 2 #mark visited node 

    while (pq and idx<100000): #run it for a given number of iterations and return no path if path does not exist 

        current_node = heapq.heappop(pq) #pop out the highest priority node from the priority queue
        steps = steps + 1 #increment the number of steps

        if [current_node.row,current_node.col] == goal : #if that node is equal to the goal 
            path = GetPathToGoal(current_node) #back trace from the curent node to the goal to give the path 

        else:
            neighbours,ind = GetNeighbours(current_node,grid,idx,agent_constraints,agent) #get neighbours of the current node 
            idx = ind 
            if not neighbours: continue #if there are no neighbours to the current node, let it be popped out of the queue 
            else: #if current node has neighbours 
                for neighbour in neighbours: #for each neighbour 
                    neighbour.h = GetHeuristic([neighbour.row,neighbour.col],goal)
                    neighbour.cost = neighbour.g + neighbour.h  

                    if [neighbour.row,neighbour.col] == goal: #if it is equal to the goal
                        steps = steps + 1 #increment number of steps 
                        path = GetPathToGoal(neighbour) #back trace from this node to the start and return the path 
                        break 
                    else: 
                        heapq.heappush(pq, neighbour) #place the neighbour in the priority queue 
        
        if path: 
            found = True #mark that the path has been found 
            break #if path has been found exit the loop 

    for i in range(len(grid)): #for all the grid elements 
        for j in range(len(grid[0])):
            if(grid[i][j]==2): grid[i][j] = 0 #earlier as the visited elements had been marked as 2. Mark them back as 0 

    if found:
        print(f"path found")
        print(agent,path)
    else:
        print("No path found")
    return path

