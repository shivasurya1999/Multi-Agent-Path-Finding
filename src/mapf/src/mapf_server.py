#! /usr/bin/env python3

import roslib
roslib.load_manifest('mapf')
import rospy
import actionlib

from mapf.msg import FindPathAction, FindPathActionGoal, FindPathActionFeedback, FindPathActionResult,AgentStartAndGoal

class MAPFServer(object):
    def __init__(self, algorithm_name):
        self._action_name = algorithm_name
        self._server = actionlib.SimpleActionServer(self._action_name, FindPathAction, execute_cb=self.execute_cb, auto_start = False)
        self._server.start()
        rospy.loginfo(self._action_name + " Server Initialized!")
    
    def execute_cb(self, goal):
        pass