#!/usr/bin/env python

# ROS
import rospy
from visualization_msgs.msg import MarkerArray
from dynamic_reconfigure.server import Server
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# LOCAL
from patrol_map_divider.cfg import PatrolMapDividerConfig
from patrol_map_divider_ros.PatrolMapDivider import PatrolMapDivider
from patrol_map_msgs.srv import SectionTasksRequest, SectionTasksResponse


class PatrolMapDividerROS(PatrolMapDivider):
    def __init__(self, nh):
        super(PatrolMapDividerROS, self).__init__()
        self.nh_ = nh
        self.pub_ = rospy.Publisher("sections_publisher", MarkerArray,
                                    queue_size=1)
        self.current_section_sub_ = rospy.Publisher("current_section",
                                                    String,
                                                    queue_size=1,
                                                    latch=True)

        self.odom_sub_ = rospy.Subscriber("odom", Odometry, self.odomCallback)
        self.dynparam_server_ = Server(PatrolMapDividerConfig,
                                       self.dynamicCallback)

        self.update_task_srv_ = rospy.Service("tasks_update_srv",
                                              SectionTasks,
                                              self.updateTasksCallback)

    '''Publish data'''
    def updateRobotStatus(self):
        # convert sections_structure to MarkerArray
        marker_array = self.getMarkerArray()
        self.pub_.publish(marker_array)

    '''Parse data from dynamic reconfigure iterating over class fields'''
    def dynamicCallback(self, config, level):
        # update all groups
        for idx in range(self.SECTION_COUNT):
            section_name = self.section_prefix_ + str(idx)

            # doesn't matter how many vertices in a section
            point_string_list = [val for key, val in config.items()
                                 if key.startswith(section_name)]

            # weird convention for config object
            if not point_string_list:
                print("No group with name " + section_name)
                continue

            self.updateSectionFromGroup(point_string_list, section_name)

        self.updateRobotStatus()  # temporary

        return config

    def odomCallback(self, msg: Odometry):
        """Update robot position

        @param msg odometry data
        """

        self.updateRobotOdom(msg)

        current_section = String(self.current_section)
        self.current_section_sub_.publish(current_section)

    def updateTasksCallback(self, req: SectionTasksRequest):
        """Update tasks of a given section with new specified list

        @param req  object with new tasks list and section name

        @return res SectionTasksResponse object with information if everything
            all points were updated without issues
        """

        res = SectionTasksResponse()

        section_name = req.section_name
        res.res = self.sections_structure[section_name].setTasksPoints(
            req.tasks_list)

        return res
