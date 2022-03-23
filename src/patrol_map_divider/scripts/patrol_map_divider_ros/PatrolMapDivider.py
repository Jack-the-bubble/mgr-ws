#!/usr/bin/env python

# PYTHON
import re

# ROS
from visualization_msgs.msg import MarkerArray, Marker


# LOCAL
from patrol_map_divider_ros.utils.MapSectionPoint import MapSectionPoint
from patrol_map_divider_ros.utils.MapSection import MapSection

class PatrolMapDivider(object):
    def __init__(self):
        # how many sections to divide the map into - not bigger than number
        #   of groups in config file
        self.SECTION_COUNT = 5

        # prefix of group in PatrolMapDivider.cfg
        self.section_prefix_ = 'section_'
        # prepare sections_structure based on SECTION_COUNT
        self.sections_structure = {} # structure with all the points - important
        for idx in range(self.SECTION_COUNT):
            self.sections_structure[self.section_prefix_ + str(idx)] = []

        self.frame_id = 'map'
        self.robot_pose = None
        self.odom_sub = None
        self.current_section = None # wich section robot is currently in

    def getMarkerArray(self):
        marker_array = MarkerArray()
        for points_list in self.section_array_:
            marker = self.getSingleMarker(points_list)
            marker_array.markers.append(marker)

    '''One marker for one section of map'''
    def getSingleMarker(self, points_list):
        triangle_count = len(points_list) - 2
        if triangle_count < 1:
            return None

        # %TODO add color and other stuff to display properly
        marker = Marker()
        marker.type = Marker.TRIANGLE_LIST
        marker.header.frame_id = self.frame_id

        # rearrange points to form triangles as described in
        # http://wiki.ros.org/rviz/DisplayTypes/Marker
        for index in range(1, triangle_count):
            marker.points.append(self.section_array_[0])
            marker.points.append(self.section_array_[index])
            marker.points.append(self.section_array_[index+1])

        return marker

    def getSectionPoseIsIn(self):
        return self.current_section

    def parsePointsFromGroup(self, point_string_list: list):
        """! Get list of points from list of strings.

        @param point_string_list list with point strings
        @return list of MapSectionPoint objects
        """
        return_list = []
        for point_string in point_string_list:
            single_point = self.parsePoint(point_string)
            return_list.append(single_point)

        return return_list

    def parsePoint(self, point_string):
        """! Use regex to divide string into point coordinates.

        @param point_string string to get floating numbers out of.
            It is expected to have exactly two numbers with decimation points
            and fraction after the period sign, even if the fraction part is '0'.

        @return An instance of MapSectionPoint initialized with parsed 2 values.
        """
        point_list = re.findall(r'\d+.\d+', point_string)
        if len(point_list) != 2:
            print("Expected 2 digits with decimation point, got ",
                  str(point_list))
            return [0.0, 0.0]

        point_list = [float(x) for x in point_list]

        return MapSectionPoint(*point_list)

    def checkPoseInSection(self, section_name):
        """! Check if current pose is in given section.

        Algorithm found on website 'http://alienryderflex.com/polygon/'

        @param section_name Section with point list to construct polygon

        @return boolean whether pose is in polygon or not
        """

        pass

    def findSectionPoseIsIn(self):
        """! Check all sections to find name where point is inside.

        Returns after first found section, rest is irrelevant,
            updates internal fileds.
        """

        pass