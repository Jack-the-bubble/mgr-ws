#!/usr/bin/env python

"""Class describing single map section data.

    Contains methods concerning single section, like changing vertices
    and manipulating task poses (just the poses!).
"""

# LOCAL
from turtle import update
from geometry_msgs.msg import Pose
from patrol_map_msgs.msg import TaskPointMsg


class MapSection(object):
    def __init__(self, name):
        self.point_list = []  # list of MapSectionPoints describing the polygon
        # dictionary of TaskPoint objects - key is task name
        self.task_list = {}
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

        # check if task points are within bounds of new polygon
        for pose in self.task_list:
            if not self.checkPoseInSection(pose.pose):
                print("Pose " + str(pose.name) +
                      "not within new bounds of polygon "
                      + self.name)
                break

        return

    def checkPoseInSection(self, pose: Pose):
        """! Check if given pose is contained in current section.

        Algorithm found on website 'http://alienryderflex.com/polygon/'.
        Important requirement is no repeating vertices in polygon.

        @param pose pose to check

        @return     boolean whether pose is in polygon or not
        """

        x, y = pose.position.x, pose.position.y
        points = self.point_list
        j = len(points) - 1
        oddNodes = False

        for i in range(len(points)):
            if points[i].y < y and points[j].y >= y \
               or points[j].y < y and points[i].y > y:
                if points[i].x + (y-points[i].y)/(points[j].y-points[i].y) * \
                   (points[j].x-points[i].x) < x:
                    oddNodes = not oddNodes

            j = i

        return oddNodes

    def setTaskPoints(self, task_list: list):
        """Replace current dictionary of tasks with new list

        If any task is outside of current polygon, the function will just
            spit out a warning, but all points will be stored
            in Section object. This method converts list of tasks
            to a dictionary.

        @param task_points list of TaskPoint objects to replace the old data

        @return update_issues says if there were any issues
            while updating the tasks dicitonary
        """

        self.task_list.clear()
        update_issues = False  # indicate whether there were any issues

        # check, if any new pose is outside of section polygon
        for task in task_list:
            if not self.checkPoseInSection(task.pose):
                print("Task {} is outside of section {}".format(
                    task.name, self.name))
                update_issues = True

            self.task_list[task.name] = task

        return update_issues

    def updateTask(self, task: TaskPointMsg):
        """Update specified task.

        If there is no task with provided name, adds new entry to dictionary
            of tasks. Task is added even if outside of current polygon.

        @param task TaskPoint object with data to update dictionary with
        """

        if not self.checkPoseInSection(task.pose):
            print("Task {} is outside of section {}".format(
                  task.name, self.name))

        self.task_list[task.name] = task

    def removeTask(self, name: str):
        """Remove task with specified name from the dictionary.

        @param name name of task to remove
        """

        self.task_list.pop(name)
