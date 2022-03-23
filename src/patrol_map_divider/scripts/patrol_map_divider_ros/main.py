#!/usr/bin/env python

# ROS
import rospy

# LOCAL
from patrol_map_divider_ros.PatrolMapDividerROS import PatrolMapDividerROS


def main():

    nh = rospy.init_node("patrol_map_divider", anonymous=False)
    map_divider = PatrolMapDividerROS(nh)

    rospy.spin()
    # rate = rospy.Rate(1.0)
    # while not rospy.is_shutdown():
    #     map_divider.update_robot_status()
    # #     map_divider.parse_point('3.5 1.0')
    #     rate.sleep()
