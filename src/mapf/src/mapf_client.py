#! /usr/bin/env python3

import roslib
roslib.load_manifest('mapf')
import rospy
import actionlib

from mapf.msg import FindPathAction, FindPathActionGoal, FindPathActionFeedback, FindPathActionResult,AgentStartAndGoal

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

    def setGoal(self, map, AgentsInfo):
        goal = FindPathActionGoal()
        goal.GridMap.MapWidth = map.size()[1]
        goal.GridMap.MapLength = map.size()[0]
        goal.GridMap.MapData = map.data

        for AgentInfo in AgentsInfo:
            agent = AgentStartAndGoal()
            agent.AgentID = AgentInfo.id
            agent.Start.Row = AgentInfo.start[0]
            agent.Start.Col = AgentInfo.start[1]
            agent.Goal.Row = AgentInfo.goal[0]
            agent.Goal.Col = AgentInfo.goal[1]

            goal.Agents.append(agent)
        
        self.planSuccessful = False
        self._client.send_goal(goal, done_cb = self.get_result_callback, active_cb=self.goal_response_callback, feedback_cb=self.feedback_callback)
    
    def get_result_callback(self, state, result_msg):
        self.planSuccessful = result_msg.PlanSuccessful
        if(self.planSuccessful):
            self.planResult = result_msg.AgentPaths

        self.planSuccessful = False

    def goal_response_callback(self):
        rospy.loginfo(self._action_name, 'Goal accepted :)')

    def feedback_callback(self, feedback_msg):
        rospy.loginfo(feedback_msg)
