#!/usr/bin/env python
import rospy
from std_msgs.msg import String

from patrol_map_msgs.srv import GetNextLocation, GetNextLocationResponse

class DummyStrategyService:
    def __init__(self):
        ''' Initialize dummy strategy service'''
        
        rospy.init_node('strategy_server')
        self.srv = rospy.Service('get_next', GetNextLocation,
                                 self.get_next_location)
        self.current_iteration = 0
        self.places = [u'kuchnia', u'none']

    def get_next_location(self, req):
        self.current_iteration = self.current_iteration + 1
        if self.current_iteration > len(self.places) + 1:
            self.current_iteration = 0

        rsp = GetNextLocationResponse(String(self.places[self.current_iteration -1]))
        rsp = GetNextLocationResponse(String(u'kuchnia'))
        return rsp

if __name__ == '__main__':
    service = DummyStrategyService()

    rospy.spin()
