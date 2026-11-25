import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely import LineString

class Obstacle:
    def __init__(self, points: list):
        self.polygon_points = np.array(points)  

    def check_collision_goal(self, point: np.ndarray, radius:float) -> bool:    
        p = Point(point)
        radius = radius
        circle = p.buffer(radius)
        polygon = Polygon(self.polygon_points)

        return polygon.intersects(circle)

    def check_collision_line(self, point_1: np.ndarray, point_2: np.ndarray) -> bool:    
        line = LineString([point_1, point_2])
        polygon = Polygon(self.polygon_points)

        return line.intersects(polygon)
