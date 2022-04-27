#!/usr/bin/env python
# encoding: utf8

import rospy
from tiago_msgs.msg import Command
import actionlib
from std_msgs.msg import Bool
from repeat_action_server.msg import AskToRepeatAction, AskToRepeatGoal

INTENT_NAME = "projects/incare-dialog-agent/agent/intents/2f028022-05b6-467d-bcbe-e861ab449c17"
client = actionlib.SimpleActionClient('/repeat_action', AskToRepeatAction)
        
class Node:
    def __init__(self):
        self.vad = True
        rospy.init_node('asker', anonymous=True)
        rospy.Subscriber('/vad_enabled', Bool, lambda x: self.callback_vad(x))
        rospy.Subscriber('/rico_cmd', Command, lambda x: self.callback(x))

    def callback_vad(self, data):
        self.vad = data.data
            
    def callback(self, cmd):
        if cmd.intent_name == INTENT_NAME:
            while not self.vad:
                pass

            goal = AskToRepeatGoal()
            client.send_goal(goal)
            client.wait_for_result()

            print("publishing to /vad_run_once")
            pub = rospy.Publisher('/vad_run_once', Bool, queue_size=10)
            msg = Bool()
            pub.publish(msg)

    def run(self):
        rospy.spin()

n = Node()
n.run()
