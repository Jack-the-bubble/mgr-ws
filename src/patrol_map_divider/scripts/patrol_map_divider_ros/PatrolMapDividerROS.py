#!/usr/bin/env python

# ROS
import rospy
from visualization_msgs.msg import MarkerArray
from dynamic_reconfigure.server import Server

# LOCAL
from patrol_map_divider.cfg import PatrolMapDividerConfig
from patrol_map_divider_ros.PatrolMapDivider import PatrolMapDivider


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
        # convert sections_structure to MarkerArray
        marker_array = self.getMarkerArray()
        self.pub_.publish(marker_array)

    '''Parse data from dynamic reconfigure iterating over class fields'''
    def dynamic_callback(self, config, level):
        # update all groups
        for idx in range(self.SECTION_COUNT):
            section_name = self.section_prefix_ + str(idx)
            point_string_list = [val for key, val in config.items()
                                         if key.startswith(section_name)]

             # weird convention for config object
            if not point_string_list:
                print("No group with name " + section_name)
                continue

            self.sections_structure[section_name] = self.parsePointsFromGroup(
                point_string_list)

        return config


    # def parse_point(self, point_string):
    #     point_list = re.findall(r'\d+.\d+', point_string)
    #     if len(point_list) != 2:
    #         print("Expected 2 digits with decimation point, got ",
    #               str(point_list))
    #         return [0.0, 0.0]

    #     point_list = [float(x) for x in point_list]

    #     return MapSectionPoint(*point_list)
