import cv2 as cv
import numpy as np

class Point2d:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def print_info(self):
        print(f"x = {self.x}\ny = {self.y}")

class Line2d:
    def __init__(self, p1: Point2d, p2: Point2d):
        self.p1 = p1
        self.p2 = p2
        

    def intersect_line(self, line):
        if type(line) is not Line2d:
            return None
        # Calculate the differences in x and y for both lines
        dx1 = self.p2.x - self.p1.x
        dy1 = self.p2.y - self.p1.y
        dx2 = line.p2.x - line.p1.x
        dy2 = line.p2.y - line.p1.y

        # Calculate the determinant
        determinant = dx1 * dy2 - dx2 * dy1

        if determinant == 0:
            # Lines are parallel or coincident
            return None

        # Calculate the differences in x and y for the starting points
        dx3 = line.p1.x - self.p1.x
        dy3 = line.p1.y - self.p1.y

        # Calculate the parameters t and u
        t = (dx3 * dy2 - dy3 * dx2) / determinant
        u = (dx3 * dy1 - dy3 * dx1) / determinant

        # If t and u are between 0 and 1, the lines intersect within the segments
        if 0 <= t <= 1 and 0 <= u <= 1:
            # Calculate the intersection point
            intersection_x = self.p1.x + t * dx1
            intersection_y = self.p1.y + t * dy1
            return Point2d(intersection_x, intersection_y)

        # Lines do not intersect within the segments
        return None


class SquareRegion:
    def __init__(self, center_point: Point2d, side_size: float):
        self.center_point = center_point
        self.side_size = side_size
        
        # x forward, y left (ROS2 frames)
        self.top_left_p = Point2d(center_point.x + side_size / 2, center_point.y + side_size / 2)
        self.top_right_p = Point2d(center_point.x + side_size / 2, center_point.y - side_size / 2)
        self.bottom_right_p = Point2d(center_point.x - side_size / 2, center_point.y - side_size / 2)
        self.bottom_left_p = Point2d(center_point.x - side_size / 2, center_point.y + side_size / 2)
    
        self.left_side = Line2d(self.top_left_p, self.bottom_left_p)
        self.right_side = Line2d(self.top_right_p, self.bottom_right_p)
        self.top_side = Line2d(self.top_left_p, self.top_right_p)
        self.bottom_side = Line2d(self.bottom_left_p, self.bottom_right_p)

    def print_info(self):
        print(f"center: ")
        self.center_point.print_info()
        print(f"top_left: ")
        self.top_left_p.print_info()
        print(f"top_right: ")
        self.top_right_p.print_info()
        print(f"bottom_right: ")
        self.bottom_right_p.print_info()
        print(f"bottom_left: ")
        self.bottom_left_p.print_info()

    def intersect_line(self, line: Line2d):
        intersections = [False, False, False, False] # left, right, top, bottom
        
        # # left
        # if not bool(self.left_side.p1.y > line.p1.y and self.left_side.p1.y > line.p2.y or self.left_side.p1.y < line.p1.y and self.left_side.p1.y < line.p2.y):
        #     intersections[0] = True
        # # right
        # if not bool(self.right_side.p1.y > line.p1.y and self.right_side.p1.y > line.p2.y or self.right_side.p1.y < line.p1.y and self.right_side.p1.y < line.p2.y):
        #     intersections[1] = True
        # # top
        # if not bool(self.top_side.p1.x > line.p1.x and self.top_side.p1.x > line.p2.x or self.top_side.p1.x < line.p1.x and self.top_side.p1.x < line.p2.x):
        #     intersections[2] = True
        # # bottom
        # if not bool(self.bottom_side.p1.x > line.p1.x and self.bottom_side.p1.x > line.p2.x or self.bottom_side.p1.x < line.p1.x and self.bottom_side.p1.x < line.p2.x):
        #     intersections[3] = True
        
        if True in intersections:
            return True
        else: 
            return False


RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)

class GeometryDrawer:
    def __init__(self):
        pass

    def draw_line(self, line: Line2d, frame, color):
        cv.line(frame, (int(line.p1.x), int(line.p1.y)), (int(line.p2.x), int(line.p2.y)), color, 1)
    
    def draw_point(self, point: Point2d, frame, color):
        cv.circle(frame, (int(point.x), int(point.y)), 3, color, 2)
    
    def draw_square(self, square: SquareRegion, frame, color):
        cv.rectangle(frame, (int(square.top_left_p.x), int(square.top_left_p.y)), (int(square.bottom_right_p.x), int(square.bottom_right_p.y)), color, 1)



c = Point2d(500,500)
sq = SquareRegion(c, 100)

# line6 = Line2d(Point2d(0, 15), Point2d(10, 15))

line1 = Line2d(Point2d(0, 0), Point2d(100, 100))
line2 = Line2d(Point2d(0, 100), Point2d(100, 0))

p = line1.intersect_line(line2)

drawer = GeometryDrawer()

canvas = np.zeros((1000, 1000, 3))
cv.line(canvas, (int(line1.p1.x), int(line1.p1.y)), (int(line1.p2.x), int(line1.p2.y)), (255, 0, 0), 1)
cv.line(canvas, (int(line2.p1.x), int(line2.p1.y)), (int(line2.p2.x), int(line2.p2.y)), (0, 255, 0), 1)
cv.circle(canvas, (int(p.x), int(p.y)), 3, (0, 0, 255), 2)

drawer.draw_square(sq, canvas, RED)

cv.imwrite("result.png", canvas)

