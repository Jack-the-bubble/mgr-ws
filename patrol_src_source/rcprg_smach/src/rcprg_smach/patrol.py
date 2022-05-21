
#!/usr/bin/env python
# encoding: utf8

import rospy
import smach
import smach_ros
import dynamic_reconfigure.client
import actionlib
import math
import threading
import copy

import yaml

from move_base_msgs.msg import *
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
import std_srvs.srv as std_srvs

from tf.transformations import quaternion_from_euler, euler_from_quaternion

from TaskER.TaskER import TaskER
from task_manager import PoseDescription
from .smach_rcprg import StateMachine
from .navigation import RememberCurrentPose, UnderstandGoal, SayImGoingTo, MoveToAwareHazards, ClearCostMaps, SayIArrivedTo, SayIdontKnow
from rcprg_smach.hazard_detector import HazardDetector
from tiago_smach import tiago_torso_controller
from pal_common_msgs.msg import DisableAction, DisableActionGoal, DisableGoal
from control_msgs.msg import PointHeadAction, PointHeadActionGoal, PointHeadGoal
from actionlib_msgs.msg import GoalID
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from patrol_map_msgs.srv import GetNextLocation
from std_msgs.msg import String
NAVIGATION_MAX_TIME_S = 100

class Patrol(StateMachine):
    def __init__(self, sim_mode, conversation_interface, kb_places):
        StateMachine.__init__(self, outcomes=['FINISHED', 'PREEMPTED', 'FAILED', 'shutdown'],
                                            # input_keys=['goal', 'susp_data'])
                                            input_keys=['susp_data'], 
                                            output_keys=['goal'])

        self.description = u'Jade do okreslonego miejsca'

        with self:
            StateMachine.add('RememberCurrentPose', RememberCurrentPose(sim_mode),
                                    transitions={'ok':'GetNextGoal', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'current_pose':'current_pose'})

            # determine next goal based on initial pose?
            StateMachine.add('GetNextGoal', GetNextGoal(sim_mode, kb_places),
                             transitions={'next': 'UnderstandGoal', 'none': 'FINISHED', 'preemption': 'PREEMPTED', 'error': 'FAILED',
                             'shutdown': 'shutdown'}, remapping={'in_current_pose': 'current_pose', 'goal': 'goal'})

            StateMachine.add('UnderstandGoal', UnderstandGoal(sim_mode, conversation_interface, kb_places),
                                    transitions={'ok':'MoveTo', 'preemption':'PREEMPTED', 'error': 'SayIdontKnow',
                                    'shutdown':'shutdown'},
                                    remapping={'in_current_pose':'current_pose', 'goal_pose':'goal', 'move_goal':'move_goal'})

            StateMachine.add('MoveTo', MoveToAwareHazards(sim_mode, conversation_interface),
                                    transitions={'ok':'FINISHED', 'preemption':'PREEMPTED', 'error': 'FAILED', 'stall':'ClearCostMaps',
                                    'shutdown':'shutdown'},
                                    remapping={'move_goal':'move_goal', 'susp_data':'susp_data'})

            StateMachine.add('ClearCostMaps', ClearCostMaps(sim_mode),
                                    transitions={'ok':'MoveTo', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'})

            StateMachine.add('SayIdontKnow', SayIdontKnow(sim_mode, conversation_interface),
                                    transitions={'ok':'FAILED', 'shutdown':'shutdown'},
                                    remapping={'goal_pose':'goal'})


class GetNextGoal(TaskER.BlockingState):
    def __init__(self, sim_mode, kb_places):
        TaskER.BlockingState.__init__(self, tf_freq=10, input_keys=['in_current_pose'],
                                      output_keys=['goal'],
                                      outcomes=['next', 'none', 'preemption', 'error', 'shutdown'])

        self.next_goal_srv = None # get service to ask for next goal
        self.next_goal = False
        self.srv_proxy = rospy.ServiceProxy('get_next', GetNextLocation)

    def transition_function(self, userdata):
        ''' call service to receive next goal and pass it forward '''

        # call srv

        # if can't communicate with service
        print("############################ Get new place #####################")

        resp = self.srv_proxy(String('bla'))
        print("Got new place {}".format(resp.next_pose.data))

        userdata.goal = PoseDescription({'place_name': u'kuchnia'})
        self.next_goal = True

        if None:
            return 'error'

        if self.preempt_requested():
            self.service_preempt()
            return 'preemption'

        if self.__shutdown__:
            return 'shutdown'

        if self.next_goal:
            return 'next'

        else:
            return 'none'

class SayNoPlace(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface):
        TaskER.BlockingState.__init__(self,tf_freq=10, input_keys=['goal_pose'],
                             outcomes=['ok', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Mowie, ze nie wiem o co chodzi'

    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        place_name = userdata.move_goal.parameters['place_name']
        assert isinstance(place_name, unicode)
        #self.conversation_interface.addSpeakSentence( u'Nie wiem gdzie jest {"' + place_name + u'", mianownik}' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe nie wiem gdzie jest {"' + place_name + u'", mianownik}' )

        if self.preempt_requested():
            #self.conversation_interface.removeExpected('q_current_task')
            self.service_preempt()
            return 'preemption'

        if self.__shutdown__:
            return 'shutdown'

        return 'ok'