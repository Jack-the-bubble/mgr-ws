'''Class used in PatrolMapDivider class.
    It is used to make sure that the given point in each section the map
    is divided into consists of points only with 2 dimensions.
'''

# ROS
from geometry_msgs.msg import Point


class MapSectionPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def getPoint(self):
        """Get geometry_msgs.Point from x and y values"""
        return Point(self.x, self.y, 0.0)
