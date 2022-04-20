#!/usr/bin/env python

from operator import imod
import rospy
import actionlib
from std_msgs.msg import String

from tiago_msgs.msg import FindHumanActionFeedback, FindHumanActionResult, \
    FindHumanAction
from tiago_move_interface import TiagoInterface

class FindHumanActionClass(object):
    _feedback = FindHumanActionFeedback()
    _result = FindHumanActionResult()

    def __init__(self, name):
        self._action_name = name
        self.found_human = False
        self.person_detect_sub = None
        self.tiago_interface = TiagoInterface.TiagoNode()
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                FindHumanAction,
                                                execute_cb=self.execute_cb,
                                                auto_start = False)

        self._as.start()

    def execute_cb(self, goal):
        # connect to publisher

        # move head
        self.tiago_interface.move_joints("look_left")

        self.tiago_interface.move_joints("look_right")

        if self.found_human:
            self._result.trigger = True
            self._as.set_succeeded = True
        
        else:
            self._result.trigger = False
            self._as.set_succeeded = False

        # disconnect from publisher

        