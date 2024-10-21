# Initial code was taken from https://www.geeksforgeeks.org/quad-tree/
from ppg_planner.geometry import Point2d, SquareRegion, LineLike, GeometryDrawer, Line2d, PolyLike
import numpy as np
import math
import cv2 as cv


# The objects that we want stored in the quadtree
class Node:
    """
    Abstract:
        Node is a class that represents the data object that we want to put inside a quad tree
    """
    def __init__(self, pos: Point2d, data):
        self.pos = pos
        self.data = data


class Quad(SquareRegion):
    def __init__(self, center_point, side_size, max_size_downstep_level, current_size_downstep_level=0):
        super().__init__(center_point=center_point, side_size=side_size)
        self.max_size_downstep_level = max_size_downstep_level
        # self.min_size_downstep_level = min_size_downstep_level
        self.current_size_downstep_level = current_size_downstep_level

        self.divided = False

        self.top_left_quad = None
        self.top_right_quad = None
        self.bottom_right_quad = None
        self.bottom_left_quad = None

        self.data = None
    
    def divide_by_line(self, line: LineLike):
        start_division = False

        # if self.current_size_downstep_level < self.min_size_downstep_level:
        #     start_division = True
        
        if self.line_is_inside(line=line):
            start_division = True

        if not start_division:
            for side in self.sides:
                intersection = side.intersect_line(line)
                if intersection is not None:
                    start_division = True
        
        if start_division:
            if not self.divided:
                if self.current_size_downstep_level < self.max_size_downstep_level:
                    # frameinfo = getframeinfo(currentframe())
                    # print(frameinfo.filename, frameinfo.lineno)
                    self.divided = True
                    self.top_left_quad = Quad(Point2d(x=self.center_point.x + self.side_size / 4, y=self.center_point.y + self.side_size / 4), self.side_size /2 , self.max_size_downstep_level, self.current_size_downstep_level + 1)
                    self.top_right_quad = Quad(Point2d(x=self.center_point.x + self.side_size / 4, y=self.center_point.y - self.side_size / 4), self.side_size /2 , self.max_size_downstep_level, self.current_size_downstep_level + 1)
                    self.bottom_right_quad = Quad(Point2d(x=self.center_point.x - self.side_size / 4, y=self.center_point.y - self.side_size / 4), self.side_size /2 , self.max_size_downstep_level, self.current_size_downstep_level + 1)
                    self.bottom_left_quad = Quad(Point2d(x=self.center_point.x - self.side_size / 4, y=self.center_point.y + self.side_size / 4), self.side_size /2 , self.max_size_downstep_level, self.current_size_downstep_level + 1)

                    if self.current_size_downstep_level + 1 < self.max_size_downstep_level:
                        # frameinfo = getframeinfo(currentframe())
                        # print(frameinfo.filename, frameinfo.lineno)
                        self.top_left_quad.divide_by_line(line)
                        self.top_right_quad.divide_by_line(line)
                        self.bottom_right_quad.divide_by_line(line)
                        self.bottom_left_quad.divide_by_line(line)
            else:
                # frameinfo = getframeinfo(currentframe())
                # print(frameinfo.filename, frameinfo.lineno)
                if self.current_size_downstep_level + 1 < self.max_size_downstep_level:
                    # frameinfo = getframeinfo(currentframe())
                    # print(frameinfo.filename, frameinfo.lineno)
                    self.top_left_quad.divide_by_line(line)
                    self.top_right_quad.divide_by_line(line)
                    self.bottom_right_quad.divide_by_line(line)
                    self.bottom_left_quad.divide_by_line(line)

    def divide_until_all_quads_are_smallest_size(self, divide_only_included=True):
        if not self.divided:
            if divide_only_included:
                included = (255, 0, 0)
                if self.data is None or self.data == included:
                    if self.current_size_downstep_level < self.max_size_downstep_level:
                        # frameinfo = getframeinfo(currentframe())
                        # print(frameinfo.filename, frameinfo.lineno)
                        self.divided = True
                        self.top_left_quad = Quad(Point2d(x=self.center_point.x + self.side_size / 4, y=self.center_point.y + self.side_size / 4), self.side_size /2 , self.max_size_downstep_level, self.current_size_downstep_level + 1)
                        self.top_right_quad = Quad(Point2d(x=self.center_point.x + self.side_size / 4, y=self.center_point.y - self.side_size / 4), self.side_size /2 , self.max_size_downstep_level, self.current_size_downstep_level + 1)
                        self.bottom_right_quad = Quad(Point2d(x=self.center_point.x - self.side_size / 4, y=self.center_point.y - self.side_size / 4), self.side_size /2 , self.max_size_downstep_level, self.current_size_downstep_level + 1)
                        self.bottom_left_quad = Quad(Point2d(x=self.center_point.x - self.side_size / 4, y=self.center_point.y + self.side_size / 4), self.side_size /2 , self.max_size_downstep_level, self.current_size_downstep_level + 1)

                        if self.current_size_downstep_level + 1 < self.max_size_downstep_level:
                            # frameinfo = getframeinfo(currentframe())
                            # print(frameinfo.filename, frameinfo.lineno)
                            self.top_left_quad.divide_until_all_quads_are_smallest_size()
                            self.top_right_quad.divide_until_all_quads_are_smallest_size()
                            self.bottom_right_quad.divide_until_all_quads_are_smallest_size()
                            self.bottom_left_quad.divide_until_all_quads_are_smallest_size()
            else:
                if self.current_size_downstep_level < self.max_size_downstep_level:
                    # frameinfo = getframeinfo(currentframe())
                    # print(frameinfo.filename, frameinfo.lineno)
                    self.divided = True
                    self.top_left_quad = Quad(Point2d(x=self.center_point.x + self.side_size / 4, y=self.center_point.y + self.side_size / 4), self.side_size /2 , self.max_size_downstep_level, self.current_size_downstep_level + 1)
                    self.top_right_quad = Quad(Point2d(x=self.center_point.x + self.side_size / 4, y=self.center_point.y - self.side_size / 4), self.side_size /2 , self.max_size_downstep_level, self.current_size_downstep_level + 1)
                    self.bottom_right_quad = Quad(Point2d(x=self.center_point.x - self.side_size / 4, y=self.center_point.y - self.side_size / 4), self.side_size /2 , self.max_size_downstep_level, self.current_size_downstep_level + 1)
                    self.bottom_left_quad = Quad(Point2d(x=self.center_point.x - self.side_size / 4, y=self.center_point.y + self.side_size / 4), self.side_size /2 , self.max_size_downstep_level, self.current_size_downstep_level + 1)

                    if self.current_size_downstep_level + 1 < self.max_size_downstep_level:
                        # frameinfo = getframeinfo(currentframe())
                        # print(frameinfo.filename, frameinfo.lineno)
                        self.top_left_quad.divide_until_all_quads_are_smallest_size()
                        self.top_right_quad.divide_until_all_quads_are_smallest_size()
                        self.bottom_right_quad.divide_until_all_quads_are_smallest_size()
                        self.bottom_left_quad.divide_until_all_quads_are_smallest_size()
        else:
            if divide_only_included:
                included = (255, 0, 0)
                if self.data is None or self.data == included:
                    if self.current_size_downstep_level + 1 < self.max_size_downstep_level:
                        self.top_left_quad.divide_until_all_quads_are_smallest_size()
                        self.top_right_quad.divide_until_all_quads_are_smallest_size()
                        self.bottom_right_quad.divide_until_all_quads_are_smallest_size()
                        self.bottom_left_quad.divide_until_all_quads_are_smallest_size()
            else:
                if self.current_size_downstep_level + 1 < self.max_size_downstep_level:
                    self.top_left_quad.divide_until_all_quads_are_smallest_size()
                    self.top_right_quad.divide_until_all_quads_are_smallest_size()
                    self.bottom_right_quad.divide_until_all_quads_are_smallest_size()
                    self.bottom_left_quad.divide_until_all_quads_are_smallest_size()
    def visualize_quad_tree(self, drawer=None, canvas=None, color=(255,255,255)):
        write = False
        color = self.data
        if drawer is None:
            write = True
            drawer = GeometryDrawer()
            canvas = np.zeros((math.ceil(self.side_size), math.ceil(self.side_size), 3))
        # frameinfo = getframeinfo(currentframe())
        # print(frameinfo.filename, frameinfo.lineno)
        if self.divided:
            # frameinfo = getframeinfo(currentframe())
            # print(frameinfo.filename, frameinfo.lineno)
            self.top_left_quad.visualize_quad_tree(drawer, canvas, color)
            self.top_right_quad.visualize_quad_tree(drawer, canvas, color)
            self.bottom_right_quad.visualize_quad_tree(drawer, canvas, color)
            self.bottom_left_quad.visualize_quad_tree(drawer, canvas, color)
        else:
            # frameinfo = getframeinfo(currentframe())
            # print(frameinfo.filename, frameinfo.lineno)
            drawer.draw_square(self, canvas, color)
        
        if write:
            # canvas = cv.rotate(canvas, cv.ROTATE_180)
            cv.imwrite("/home/illarion/Pictures/result.png", canvas)
    
    def include_zone(self, zone):
        if self.divided:
            self.top_left_quad.include_zone(zone)
            self.top_right_quad.include_zone(zone)
            self.bottom_right_quad.include_zone(zone)
            self.bottom_left_quad.include_zone(zone)
        else:
            included = (255, 0, 0)
            excluded = (0, 0, 255)
            if zone.point_in_poly(self.center_point):
                self.data = included
            else:
                if self.data is None:
                    self.data = excluded

    def exclude_zone(self, zone):
        if self.divided:
            self.top_left_quad.exclude_zone(zone)
            self.top_right_quad.exclude_zone(zone)
            self.bottom_right_quad.exclude_zone(zone)
            self.bottom_left_quad.exclude_zone(zone)
        else:
            excluded = (0, 0, 255)
            if zone.point_in_poly(self.center_point):
                self.data = excluded
