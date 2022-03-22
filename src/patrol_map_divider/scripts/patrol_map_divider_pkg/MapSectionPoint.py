'''Class used in PatrolMapDivider class.
    It is used to make sure that the given point in each section the map
    is divided into consists of points only with 2 dimensions.
'''


class MapSectionPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y
