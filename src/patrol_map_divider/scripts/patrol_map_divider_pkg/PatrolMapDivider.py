#!/usr/bin/env python

# ROS
from visualization_msgs.msg import MarkerArray, Marker


class PatrolMapDivider(object):
    def __init__(self):
        # lists of all points - most important data
        self.section_array_ = list()
        self.sections_structure = {
            'section_1': []
        }
        self.frame_id = 'map'

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
