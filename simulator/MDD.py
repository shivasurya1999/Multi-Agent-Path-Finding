#GetMDD
"""
Set Discovered[s]=true and discovered[v]=false for all other v
Initialize L[0] to equal s
Set layer counter ito equal 0
While L[i] not empty:
Initialize an empty list L[i+1]
For each node u in L[i]
Consider each edge (u,v) adjacent to u
If discovered[v]=false
discovered[v]=true
Add v to L[i+1]
Endfor
Increment ito i+1
Endwhile
"""

#exploration order: right,down,left,up 

import networkx as nx


def EliminateMDDFar(current_layer,previous_layer): #eliminate the nodes of MDD which don't give shortest path to goal node 
    new_previous = []
    for current_node in current_layer:
        for previous_node in previous_layer:
            diff_node = []
            diff_node.append(abs(current_node[0]-previous_node[0]))
            diff_node.append(abs(current_node[1]-previous_node[1]))
            if(abs(diff_node[0]+diff_node[1])==1):
                new_previous.append(previous_node)
    return new_previous


def GetMDD(grid, start, goal):
    L_mdd = []
    L = []
    L.append(start)
    grid[start[0]][start[1]] = 2
    L_mdd.append(L) 
    layer = 0 
    rows = len(grid)
    cols = len(grid[0])
    while (grid[goal[0]][goal[1]] != 2):
        L_next = []
        for i in range(len(L)): 
            u = L[i]  
            if (0 <= u[1] + 1 <= cols - 1) and (grid[u[0]][u[1] + 1] == 0):
                L_next.append([u[0], u[1] + 1])
                grid[u[0]][u[1] + 1] = 2
            if (0 <= u[0] - 1 <= rows - 1) and (grid[u[0] - 1][u[1]] == 0):
                L_next.append([u[0] - 1, u[1]])
                grid[u[0] - 1][u[1]] = 2
            if (0 <= u[1] - 1 <= cols - 1) and (grid[u[0]][u[1] - 1] == 0):
                L_next.append([u[0], u[1] - 1])
                grid[u[0]][u[1] - 1] = 2
            if (0 <= u[0] + 1 <= rows - 1) and (grid[u[0] + 1][u[1]] == 0):
                L_next.append([u[0] + 1, u[1]])
                grid[u[0] + 1][u[1]] = 2
            if(len(L_next)==0):
                break 
        L = L_next
        L_mdd.append(L_next)
        
        layer += 1

    L_mdd[-1] = [goal]
    graph_levels = L_mdd # Define the list of lists representing the graph levels
    
    for r in range(len(grid)): #undo visited nodes for future agents 
        for c in range(len(grid[0])):
            if(grid[r][c]==2): grid[r][c] = 0
    # print("agent levels list")
    # print(graph_levels)

    for i in range(len(graph_levels)-1,1,-1):
        current_layer = graph_levels[i]
        previous_layer = graph_levels[i-1]
        graph_levels[i-1] = EliminateMDDFar(current_layer,previous_layer)

    # Create an empty directed graph
    G = nx.DiGraph() 
 
    # Add nodes from each level to the graph
    for level, nodes in enumerate(graph_levels):
        for node in nodes: 
            node = tuple(node)
            G.add_node(node,level=level)
            # print("node and level")
            # print(node,level) 

    # print(G.nodes(data=True))
    # Add edges between nodes at adjacent levels
    for i in range(len(graph_levels) - 1):
        curr_level_nodes = graph_levels[i] 
        next_level_nodes = graph_levels[i + 1]
        # if(i==0): 
        #     print("level 0")
        #     print(curr_level_nodes)
        for u in curr_level_nodes:
            for v in next_level_nodes:
                if ([u[0], u[1] + 1] == v) or ([u[0] - 1, u[1]] == v) or ([u[0], u[1] - 1] == v) or ([u[0] + 1, u[1]] == v):
                    G.add_edge(tuple(u), tuple(v))
    # print(G.nodes[0])
    # print(G.edges(data=True))
    # print(G.nodes(data=True))
    return G 




