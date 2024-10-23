import cv2 as cv
import numpy as np
from abc import ABC, abstractmethod


run_tests = False


class Point2d:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def print_info(self):
        print(f"x = {self.x}\ny = {self.y}")

class LineLike(ABC):
    def __init__(self, p1: Point2d, p2: Point2d):
        self.p1 = p1
        self.p2 = p2

    @abstractmethod
    def intersect_line(self, line) -> Point2d:
        pass


class PolyLike(ABC):
    def __init__(self, lines: list[LineLike]):
        self.sides  = lines
        self.x_min = None
        self.y_min = None
        self.x_max = None
        self.y_max = None

        for side in self.sides:
            if self.x_min is None or side.p1.x < self.x_min:
                self.x_min = side.p1.x
            if self.y_min is None or side.p1.y < self.y_min:
                self.y_min = side.p1.y
            if self.x_max is None or side.p1.x > self.x_max:
                self.x_max = side.p1.x
            if self.y_max is None or side.p1.y > self.y_max:
                self.y_max = side.p1.y
        

    @abstractmethod
    def intersect_poly(self, poly):
        quit(404)
        

    def intersect_line(self, line_in: LineLike):
        intersections = []
        
        for line in self.sides:
            intersection = line.intersect_line(line_in)
            if intersection is not None:
                intersections.append(intersection)

        return intersections
    
    def point_in_poly(self, point: Point2d) -> bool:
        # if len(point) != 2:
        #     print("Error! Point in poly")
        #     return False

        crossed_sides = []

        for i in range(len(self.sides) - 1):
            if point.y >= self.sides[i].p1.y and point.y <= self.sides[i].p2.y:
                crossed_sides.append((self.sides[i].p1, self.sides[i].p2))
            elif point.y <= self.sides[i].p1.y and point.y >= self.sides[i].p2.y:
                crossed_sides.append((self.sides[i].p1, self.sides[i].p2))

        j = len(self.sides) - 1

        if point.y >= self.sides[0].p1.y and point.y <= self.sides[j].p2.y:
            crossed_sides.append((self.sides[0].p1, self.sides[j].p2))
        elif point.y <= self.sides[0].p1.y and point.y >= self.sides[j].p2.y:
            crossed_sides.append((self.sides[0].p1, self.sides[j].p2))

        crossing_points = []

        for side in crossed_sides:
            x1, y1 = side[0].x, side[0].y
            x2, y2 = side[1].x, side[1].y

            if y1 == y2:
                continue

            x = x1 + (point.y - y1) * (x2 - x1) / (y2 - y1)
            y = point.y

            if (x, y) not in crossing_points:
                crossing_points.append((x, y))

        left_c, right_c = 0, 0

        for cpoint in crossing_points:
            xcp, ycp = cpoint
            x, y = point.x, point.y

            if x == xcp and y == ycp:
                return True

            if xcp < x:
                left_c += 1
            elif xcp > x:
                right_c += 1
            else:
                return True

        if left_c % 2 == 1 and right_c % 2 == 1:
            return True
        else:
            return False



class Line2d(LineLike):
    def __init__(self, p1: Point2d, p2: Point2d):
        super().__init__(p1, p2)

    def intersect_line(self, line: LineLike):
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


class SquareRegion(PolyLike):
    def __init__(self, center_point: Point2d, side_size: float):
        self.center_point = center_point
        self.side_size = side_size
        
        # x forward, y left (ROS2 frames)
        self.top_left_p = Point2d(center_point.x + side_size / 2, center_point.y + side_size / 2)
        self.top_right_p = Point2d(center_point.x + side_size / 2, center_point.y - side_size / 2)
        self.bottom_right_p = Point2d(center_point.x - side_size / 2, center_point.y - side_size / 2)
        self.bottom_left_p = Point2d(center_point.x - side_size / 2, center_point.y + side_size / 2)

        lines = []

        lines.append(Line2d(self.top_left_p, self.bottom_left_p))
        lines.append(Line2d(self.top_right_p, self.bottom_right_p))
        lines.append(Line2d(self.top_left_p, self.top_right_p))
        lines.append(Line2d(self.bottom_left_p, self.bottom_right_p))

        super().__init__(lines)

    def intersect_poly(self):
        quit(404)

    def line_is_inside(self, line: LineLike):
        x_max = self.top_left_p.x
        x_min = self.bottom_left_p.x

        y_max = self.bottom_left_p.y
        y_min = self.bottom_right_p.y

        xl_max = None
        xl_min = None
        yl_max = None
        yl_min = None

        if line.p1.x > line.p2.x:
            xl_max = line.p1.x
            xl_min = line.p2.x
        else: 
            xl_min = line.p1.x
            xl_max = line.p2.x

        if line.p1.y > line.p2.y:
            yl_max = line.p1.y
            yl_min = line.p2.y
        else: 
            yl_min = line.p1.y
            yl_max = line.p2.x

        if x_max > xl_max and x_min < xl_min and y_max > yl_max and y_min < yl_min:
            return True
        else: 
            return False


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


class Polygon(PolyLike):
    def __init__(self, lines: list[Line2d]):
        super().__init__(lines=lines)
    
    def intersect_poly(self, poly):
        quit(404)


class GeometryDrawer:
    def __init__(self):
        pass

    def draw_line(self, line: Line2d, frame, color):
        cv.line(frame, (int(line.p1.x), frame.shape[0] - int(line.p1.y)), (int(line.p2.x), frame.shape[0] - int(line.p2.y)), color, 1)
    
    def draw_point(self, point: Point2d, frame, color):
        cv.circle(frame, (int(point.x), frame.shape[0] - int(point.y)), 3, color, 2)
    
    def draw_square(self, square: SquareRegion, frame, color):
        # top_left = (frame.shape[1] - int(square.bottom_right_p.x), int(square.top_left_p.y))
        # bottom_right = (frame.shape[1] - int(square.top_left_p.x), int(square.bottom_right_p.y))
        top_left = (int(square.bottom_right_p.x), int(square.top_left_p.y))
        bottom_right = (int(square.top_left_p.x), int(square.bottom_right_p.y))
        # Draw the rectangle
        cv.rectangle(frame, top_left, bottom_right, color, 1)
        # cv.rectangle(frame, (int(square.top_left_p.x), frame.shape[0] - int(square.top_left_p.y)), (int(square.bottom_right_p.x), frame.shape[0] - int(square.bottom_right_p.y)), color, 1)

    # def draw_quad(self, quad, frame, color):
    #     cv.rectangle(frame, (100 * int(square.top_left_p.x), 100 * (frame.shape[0] - int(square.top_left_p.y))), (100 * int(square.bottom_right_p.x), 100 * (frame.shape[0] - int(square.bottom_right_p.y))), color, -1)



if run_tests:

    RED = (0, 0, 255)
    GREEN = (0, 255, 0)
    BLUE = (255, 0, 0)

    canvas = np.zeros((1000, 1000, 3))
    drawer = GeometryDrawer()

    c = Point2d(500,500)
    sq = SquareRegion(c, 100)

    # Test 1: Line intersects the top side
    line1 = Line2d(Point2d(400, 450), Point2d(600, 450))
    drawer.draw_square(sq, canvas, RED)
    drawer.draw_line(line1, canvas, color=GREEN)
    intersections = sq.intersect_line(line1)
    for i in intersections:
        if i:
            drawer.draw_point(i, canvas, BLUE)

    # Test 2: Line intersects the left side
    line2 = Line2d(Point2d(550, 300), Point2d(550, 700))
    drawer.draw_line(line2, canvas, color=GREEN)
    intersections = sq.intersect_line(line2)
    for i in intersections:
        if i:
            drawer.draw_point(i, canvas, BLUE)

    # Test 3: Line intersects the bottom side
    line3 = Line2d(Point2d(400, 550), Point2d(600, 550))
    drawer.draw_line(line3, canvas, color=GREEN)
    intersections = sq.intersect_line(line3)
    for i in intersections:
        if i:
            drawer.draw_point(i, canvas, BLUE)

    # Test 4: Line intersects the right side
    line4 = Line2d(Point2d(450, 300), Point2d(450, 700))
    drawer.draw_line(line4, canvas, color=GREEN)
    intersections = sq.intersect_line(line4)
    for i in intersections:
        if i:
            drawer.draw_point(i, canvas, BLUE)

    # Test 5: Line does not intersect the square
    line5 = Line2d(Point2d(600, 600), Point2d(1000, 1000))
    drawer.draw_line(line5, canvas, color=GREEN)
    intersections = sq.intersect_line(line5)
    for i in intersections:
        if i:
            drawer.draw_point(i, canvas, BLUE)


    # Test 6: Line intersects at a corner
    line6 = Line2d(Point2d(400, 400), Point2d(600, 600))
    drawer.draw_line(line6, canvas, color=GREEN)
    intersections = sq.intersect_line(line6)
    for i in intersections:
        if i:
            drawer.draw_point(i, canvas, BLUE)

    # Test 7: Line is parallel to the sides
    line7 = Line2d(Point2d(450, 400), Point2d(450, 600))
    drawer.draw_line(line7, canvas, color=GREEN)
    intersections = sq.intersect_line(line7)
    for i in intersections:
        if i:
            drawer.draw_point(i, canvas, BLUE)

    # Test 8: Line intersects multiple sides
    line8 = Line2d(Point2d(400, 450), Point2d(600, 550))
    drawer.draw_line(line8, canvas, color=GREEN)
    intersections = sq.intersect_line(line8)
    for i in intersections:
        if i:
            drawer.draw_point(i, canvas, BLUE)

    # Test 9: Line intersects the square at the center
    line9 = Line2d(Point2d(500, 300), Point2d(500, 700))
    drawer.draw_line(line9, canvas, color=GREEN)
    intersections = sq.intersect_line(line9)
    for i in intersections:
        if i:
            drawer.draw_point(i, canvas, BLUE)

    # Test 10: Line is very close to the square but does not intersect
    line10 = Line2d(Point2d(400, 400), Point2d(600, 399))
    drawer.draw_line(line10, canvas, color=GREEN)
    intersections = sq.intersect_line(line10)
    for i in intersections:
        if i:
            drawer.draw_point(i, canvas, BLUE)


    drawer.draw_square(sq, canvas, RED)

    canvas = cv.rotate(canvas, cv.ROTATE_180)
    cv.imwrite("result.png", canvas)
