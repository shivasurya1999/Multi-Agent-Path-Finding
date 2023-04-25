#! /usr/bin/env python3

import roslib
roslib.load_manifest('mapf')
import rospy
import actionlib
import numpy as np

from mapf.msg import FindPathAction, FindPathGoal, FindPathFeedback, FindPathResult, AgentStartAndGoal, AgentPath, CellPosition

class MAPFServer(object):
    def __init__(self, algorithm_name, function):
        self._action_name = algorithm_name
        self.function = function
        self._server = actionlib.SimpleActionServer(self._action_name, FindPathAction, execute_cb=self.execute_cb, auto_start = False)
        self._server.start()
        rospy.loginfo(self._action_name + " Server Initialized!")

    def execute_cb(self, goal):
        grid = np.frombuffer(goal.mapf_goal.GridMap.MapData,dtype=np.uint8).reshape(goal.mapf_goal.GridMap.MapHeight, goal.mapf_goal.GridMap.MapWidth).tolist()
        agent_dict = {}
        for agentInfo in goal.mapf_goal.Agents:
            agent_dict[agentInfo.AgentID] = [[agentInfo.Start.Row, agentInfo.Start.Col]]
            agent_dict[agentInfo.AgentID].append([agentInfo.Goal.Row, agentInfo.Goal.Col])
        
        print(grid)
        print(agent_dict)
        foundPath, pathDict, pathCost = self.function(grid, agent_dict)

        result = FindPathResult()
        result.mapf_result.PlanSuccessful = foundPath
        for key in pathDict:
            path = AgentPath()
            path.AgentID = key
            for pos in pathDict[key]:
                cell = CellPosition()
                cell.Row = pos[0]
                cell.Col = pos[1]
                path.Path.append(cell)
            result.mapf_result.AgentPaths.append(path)
        self._server.set_succeeded(result)

