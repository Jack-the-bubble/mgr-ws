#!/usr/bin/env python

"""Class TaskPoint contains all tasks that need to be completed
    in certain step when scanning a single section
"""


class TaskPoint(object):
    def __init__(self):
        self.pose = None  # pose in map coordinates (?) for robot to navigate to
        self.name = None
