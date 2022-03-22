#!/usr/bin/env python

# ROS
import re
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from dynamic_reconfigure.server import Server

# LOCAL
from patrol_map_divider.cfg import PatrolMapDividerConfig
from patrol_map_divider_pkg.PatrolMapDivider import PatrolMapDivider
from patrol_map_divider_pkg.MapSectionPoint import MapSectionPoint


class PatrolMapDividerROS(PatrolMapDivider):
    def __init__(self, nh):
        super(PatrolMapDividerROS, self).__init__()
        self.nh_ = nh
        self.pub_ = rospy.Publisher("sections_publisher", MarkerArray,
                                    queue_size=1)
        self.dynparam_server_ = Server(PatrolMapDividerConfig,
                                       self.dynamic_callback)

    '''Publish data'''
    def update_robot_status(self):
        # convert section_array to MarkerArray
        marker_array = self.getMarkerArray()
        self.pub_.publish(marker_array)

    '''Parse data from dynamic reconfigure iterating over class fields'''
    def dynamic_callback(self, config, level):
        # %TODO list of MapSectionPoints instead of a single one
        self.sections_structure['section_1'] = self.parse_point(
            config['point_1'])

        return config

    '''use regex to divide string into point coordinates'''
    def parse_point(self, point_string):
        point_list = re.findall(r'\d+.\d+', point_string)
        if len(point_list) != 2:
            print("Expected 2 digits with decimation point, got ",
                  str(point_list))
            return [0.0, 0.0]

        point_list = [float(x) for x in point_list]

        return MapSectionPoint(*point_list)
