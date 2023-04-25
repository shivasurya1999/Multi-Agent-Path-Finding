#! /usr/bin/env python3

import roslib
roslib.load_manifest('mapf')
import rospy
import actionlib
import numpy as np

from mapf.msg import FindPathAction, FindPathGoal, FindPathFeedback, FindPathResult,AgentStartAndGoal

# if __name__ == '__main__':
#     rospy.init_node('mapf_client')
#     client = actionlib.SimpleActionClient('find_path', FindPathAction)
#     client.wait_for_server()

#     goal = DoDishesGoal()
#     # Fill in the goal here
#     client.send_goal(goal)
#     client.wait_for_result(rospy.Duration.from_sec(5.0))
class MAPFClient(object):
    def __init__(self, algorithm_name):
        self.planSuccessful = False
        self._action_name = algorithm_name
        self._client = actionlib.SimpleActionClient(self._action_name, FindPathAction)
        self._client.wait_for_server()
        rospy.loginfo(self._action_name + " Client Initialized!")

    def cancel_all_goals(self):
        self._client.cancel_all_goals()
        
    def setGoal(self, map, AgentsInfo):
        goal = FindPathGoal()
        goal.mapf_goal.GridMap.MapWidth = map.shape[1]
        goal.mapf_goal.GridMap.MapHeight = map.shape[0]
        goal.mapf_goal.GridMap.MapData = np.ravel(map).tolist()

        for AgentId in AgentsInfo:
            agent = AgentStartAndGoal()
            agent.AgentID = AgentId
            agent.Start.Row = AgentsInfo[AgentId][0][0]
            agent.Start.Col = AgentsInfo[AgentId][0][1]
            agent.Goal.Row = AgentsInfo[AgentId][1][0]
            agent.Goal.Col = AgentsInfo[AgentId][1][1]

            goal.mapf_goal.Agents.append(agent)
        
        self.planSuccessful = False
        self._client.send_goal(goal, done_cb = self.get_result_callback, active_cb=self.goal_response_callback, feedback_cb=self.feedback_callback)
    
    def get_result_callback(self, state, result_msg):
        self.planSuccessful = result_msg.mapf_result.PlanSuccessful
        print(self._action_name)
        print(self.planSuccessful)
        if(self.planSuccessful):
            self.pathDict = {}
            for agent_path in result_msg.mapf_result.AgentPaths:
                self.pathDict[agent_path.AgentID] = []
                for cell in agent_path.Path:
                    self.pathDict[agent_path.AgentID].append([cell.Row,cell.Col])
        
        # pathDict = {}
        # for agent_path in self.planResult:
        #     pathDict[agent_path.AgentID] = []
        #     for cell in agent_path.Path:
        #         pathDict[agent_path.AgentID].append([cell.Row,cell.Col])
        # print(self.pathDict)
        
    def goal_response_callback(self):
        rospy.loginfo(self._action_name)
        rospy.loginfo('Goal accepted :)')

    def feedback_callback(self, feedback_msg):
        rospy.loginfo(feedback_msg)
