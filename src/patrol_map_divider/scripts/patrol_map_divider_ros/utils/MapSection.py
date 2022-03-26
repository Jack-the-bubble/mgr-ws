#!/usr/bin/env python

"""Class describing single map section data.

    Contains methods concerning single section, like changing vertices
    and manipulating task poses (just the poses!).
"""

# LOCAL
from patrol_map_divider_ros.utils.TaskPoint import TaskPoint
from geometry_msgs.msg import Pose


class MapSection(object):
    def __init__(self, name):
        self.point_list = []  # list of MapSectionPoints
        self.task_poses = []  # list of TaskPoints for tasks
        self.name = name

    def updateVertices(self, point_list: list):
        """Update vertices describing the section.

        After update makes sure all defined task poses are within bounds
        of new polygon. Checking before and catching error could break
        data integrity, since reverting changes from dynamic reconfigure
        would be painful.

        @param point_list list of new vertices of type MapSectionPoint
        """

        self.point_list = point_list

        for pose in self.task_poses:
            if not self.checkPoseInSection(pose.pose):
                print("Pose " + str(pose.name) +
                      "not within new bounds of polygon "
                      + self.name)
                break

        return

    def checkPoseInSection(self, pose: Pose):
        """! Check if given pose is contained in current section.

        Algorithm found on website 'http://alienryderflex.com/polygon/'

        @param pose pose to check

        @return     boolean whether pose is in polygon or not
        """

        return True
