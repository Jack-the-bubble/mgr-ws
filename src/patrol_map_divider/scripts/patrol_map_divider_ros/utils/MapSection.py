#!/usr/bin/env python

"""Class describing single map section data"""

class MapSection(object):
    def __init__(self, name):
        self.point_list = []  # list of MapSectionPoints
        self.task_poses = []  # list of TaskPoints for scanning section
        self.name = name