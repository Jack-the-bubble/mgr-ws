#!/usr/bin/env python

# PYTHON
import re

# ROS
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Pose
from nav_msgs.msg import Odometry


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
        self.sections_structure = {}  # structure with all the points-important
        for idx in range(self.SECTION_COUNT):
            name = self.section_prefix_ + str(idx)
            self.sections_structure[name] = MapSection(name)

        self.frame_id = 'map'
        self.robot_pose = Pose()
        self.current_section = None  # wich section robot is currently in

    def getMarkerArray(self):
        """Create ros marker array to enable section visualization in rviz."""

        marker_array = MarkerArray()
        for section in self.sections_structure.values():
            marker = self.getSingleMarker(section)
            if marker:
                marker_array.markers.append(marker)

        return marker_array

    '''One marker for one section of map'''
    def getSingleMarker(self, section: MapSection):
        """Create marker for given map section using marker with triangle list.

        Marker is created using section vertices to create triangles
            that will be drawn e.g. in rviz. Color is decided
            based on section number, last character MUST be a digit.

        @param section instance to create a marker from

        @return Marker based on given section points
        """

        colors = [ColorRGBA(1.0, 0.0, 0.0, 0.2),
                  ColorRGBA(0.0, 1.0, 0.0, 0.2),
                  ColorRGBA(0.0, 0.0, 1.0, 0.2)]
        color_idx = int(section.name[-1]) % 3
        point_list = section.point_list
        triangle_count = len(point_list) - 2
        if triangle_count < 1:
            print("Not enough vertices to create triangles in section " +
                  section.name)
            return None

        # %TODO add color and other stuff to display properly
        marker = Marker()
        marker.type = Marker.TRIANGLE_LIST
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.get_rostime()
        marker.ns = section.name
        marker.id = 0
        marker.color = colors[color_idx]
        marker.scale = Vector3(1.0, 1.0, 1.0)
        marker.pose.orientation.w = 1.0

        # rearrange points to form triangles as described in
        # http://wiki.ros.org/rviz/DisplayTypes/Marker
        for index in range(triangle_count):
            marker.points.append(point_list[0].getPoint())
            marker.points.append(point_list[index+1].getPoint())
            marker.points.append(point_list[index+2].getPoint())

        return marker

    def getSectionPoseIsIn(self):
        return self.current_section

    def updateSectionFromGroup(self, point_string_list: list, name: str):
        """! Get Section vertices from list of strings.

        @param point_string_list list with point strings
        @param name              name of section to update
        """
        point_list = []
        for point_string in point_string_list:
            single_point = self.parsePoint(point_string)
            point_list.append(single_point)

        # remove duplicates
        point_list = self.removeDuplicatePoints(point_list)
        if len(point_list) < 3:
            print("Too few vertices to create a polygon, update values")
            return

        self.sections_structure[name].updateVertices(point_list)

    def updateRobotOdom(self, odom: Odometry):
        """Update current robot odometry data and section it's currently in.

        @param odom odometry data
        """

        self.robot_pose = odom.pose.pose

        self.findSectionPoseIsIn()

    def parsePoint(self, point_string):
        """! Use regex to divide string into point coordinates.

        @param point_string string to get floating numbers out of.
            It is expected to have exactly two numbers with decimation points
            and fraction after the period sign,
            even if the fraction part is '0'.

        @return An instance of MapSectionPoint initialized
            with parsed 2 values.
        """
        point_list = re.findall(r'-?(?:\d+.\d+)', point_string)
        if len(point_list) != 2:
            print("Expected 2 digits with decimation point, got ",
                  str(point_list))
            return MapSectionPoint(0.0, 0.0)

        point_list = [float(x) for x in point_list]

        return MapSectionPoint(*point_list)

    def removeDuplicatePoints(self, point_list: list):
        """Remove duplicated points based on point coordinates.

        @param point_list  in-going list of MapSectionPoints
        @param return_list list without duplicates
        """

        return_list = []

        for idx, point in enumerate(point_list):
            found = False
            for index in range(idx+1, len(point_list)):
                if point.x == point_list[index].x and \
                   point.y == point_list[index].y:
                    found = True
                    break

            if not found:
                return_list.append(point)

        return return_list

    def findSectionPoseIsIn(self):
        """! Check all sections to find name where point is inside.

        Returns after first found section, rest is irrelevant,
            updates internal fileds.
        """

        # check last saved section
        if self.current_section:
            last_section = self.sections_structure[self.current_section]
            if last_section.checkPoseInSection(self.robot_pose):
                return

        # if not found yet or robot moved out of last section, look through all
        self.current_section = None
        for section in self.sections_structure.values():
            if section.checkPoseInSection(self.robot_pose):
                self.current_section = section.name
                break

        if not self.current_section:
            print("given point {}, {} outside ".format(
                  self.robot_pose.position.x, self.robot_pose.position.y) +
                  "of currently defined sections")
