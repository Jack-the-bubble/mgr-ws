#!/usr/bin/env python

from operator import imod
import rospy
import actionlib
from std_msgs.msg import String

from tiago_msgs.msg import FindHumanActionFeedback, FindHumanActionResult, \
    FindHumanAction
from tiago_move_interface import TiagoInterface
from pal_detection_msgs.msg import Detections2d

class FindHumanActionClass(object):
    _feedback = FindHumanActionFeedback()
    _result = FindHumanActionResult()

    def __init__(self, name):
        self._action_name = name
        self.found_human = False
        self.person_detect_sub = rospy.Subscriber('/person_detector/detections',
                                                  Detections2d, self.detector_cb)
        self.tiago_interface = TiagoInterface.TiagoNode()
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                FindHumanAction,
                                                execute_cb=self.execute_cb,
                                                auto_start = False)

        self.move_list = ['look_left_1_5', 'look_left_1_4', 'look_left_1_3',
                          'look_left_1_2', 'look_left_1_1', 'look_left_1_0',
                          'look_left_0_3', 'look_right_0_3', 'look_right_1_5']

        self._as.start()

    def detector_cb(self, msg):
        if len(msg.detections) > 0:
            self.found_human = True

    def execute_cb(self, goal):
        # connect to publisher

        # move head
        for motion in self.move_list:
            self.tiago_interface.move_joints(motion)

            if self.found_human:
                break

        if self.found_human:
            self._result.trigger = True
            self._as.set_succeeded = True

        else:
            self._result.trigger = False
            self._as.set_succeeded = False

        # disconnect from publisher

