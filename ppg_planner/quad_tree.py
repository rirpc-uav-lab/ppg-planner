# Initial code was taken from https://www.geeksforgeeks.org/quad-tree/
from ppg_planner.geometry import Point2d, SquareRegion, LineLike, GeometryDrawer, Line2d, PolyLike
import numpy as np
import math
import cv2 as cv
from getpass import getuser
from copy import copy, deepcopy


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
            canvas = np.zeros((math.ceil(self.side_size), math.ceil(self.side_size), 3), dtype=np.uint8)
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
            canvas = cv.flip(canvas, -1)
            cv.imwrite(f"/home/{getuser()}/Pictures/result.png", canvas)
        return canvas
        
    
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

    def get_included_node_list(self):
        included = (255, 0, 0)
        if self.divided:
            return self.top_left_quad.get_included_node_list() + self.top_right_quad.get_included_node_list() + self.bottom_right_quad.get_included_node_list() + self.bottom_left_quad.get_included_node_list()
        else:
            if self.data == included:
                return [self]
            else:
                return []
    
    def _get_min_side_size(self):
        if self.divided:
            sizes = (self.top_left_quad._get_min_side_size(), self.top_right_quad._get_min_side_size(), self.bottom_right_quad._get_min_side_size(), self.bottom_left_quad._get_min_side_size())
            return min(sizes)
        else:
            return self.side_size

    def _generate_included_incidency_matrix(self, min_side_size):
        included = (255, 0, 0)
        incidency_mx = {}
        node_dict = {}
        if self.divided:
            # Слияние различных матриц инциндентности должно происходить здесь, учитывая размеры квадов и их вазиморасположение отнсительно друг друга
            
            tl_nd, tl_inmx = self.top_left_quad._generate_included_incidency_matrix(min_side_size)
            tr_nd, tr_inmx = self.top_right_quad._generate_included_incidency_matrix(min_side_size)
            br_nd, br_inmx = self.bottom_right_quad._generate_included_incidency_matrix(min_side_size)
            bl_nd, bl_inmx = self.bottom_left_quad._generate_included_incidency_matrix(min_side_size)
            
            child_side_size_tmp = copy(self.side_size) / 2
            counter = 0
            while (child_side_size_tmp != min_side_size):
                child_side_size_tmp /= 2
                counter += 1
            
            side_index = 2 ** counter
            diag_distance = min_side_size * math.sqrt(2)

            # диагональные слияния не отработаны
            if tl_nd is not None and br_nd is not None:
                tlcenter = self.top_left_quad.center_point
                brcenter = self.bottom_right_quad.center_point
                tl_node = Point2d(x=(tlcenter.x - self.side_size / 2) + (0 * min_side_size + 0.5 * min_side_size), y=(tlcenter.y - self.side_size / 2) + (0 * min_side_size + 0.5 * min_side_size))
                br_node = Point2d(x=(brcenter.x - self.side_size / 2) + (side_index * min_side_size + 0.5 * min_side_size), y=(brcenter.y - self.side_size / 2) + (side_index * min_side_size + 0.5 * min_side_size))
                tl_uid = f"{tl_node.x},{tl_node.y}"
                br_uid = f"{br_node.x},{br_node.y}"
                if tl_uid in tl_nd:
                    if br_uid in br_nd:
                        tl_inmx[tl_uid][br_uid] = diag_distance
                if br_uid in br_nd:
                    if tl_uid in tl_nd:
                        br_inmx[br_uid][tl_uid] = diag_distance
                incidency_mx.update(tl_inmx)
                incidency_mx.update(br_inmx)
                node_dict.update(tl_nd)
                node_dict.update(br_nd)
            if tr_nd is not None and bl_nd is not None:
                trcenter = self.top_left_quad.center_point
                blcenter = self.bottom_right_quad.center_point
                tr_node = Point2d(x=(trcenter.x - self.side_size / 2) + (0 * min_side_size + 0.5 * min_side_size), y=(trcenter.y - self.side_size / 2) + (side_index * min_side_size + 0.5 * min_side_size))
                bl_node = Point2d(x=(blcenter.x - self.side_size / 2) + (side_index * min_side_size + 0.5 * min_side_size), y=(blcenter.y - self.side_size / 2) + (0 * min_side_size + 0.5 * min_side_size))
                tr_uid = f"{tr_node.x},{tr_node.y}"
                bl_uid = f"{bl_node.x},{bl_node.y}"
                if tr_uid in tr_nd:
                    if bl_uid in bl_nd:
                        tr_inmx[tr_uid][bl_uid] = diag_distance
                if bl_uid in bl_nd:
                    if tr_uid in tr_nd:
                        bl_inmx[bl_uid][tr_uid] = diag_distance
                incidency_mx.update(bl_inmx)
                incidency_mx.update(tr_inmx)
                node_dict.update(bl_nd)
                node_dict.update(tr_nd)


            # случаи, когда три из квадов пустые
            if tl_nd is not None:
                node_dict.update(tl_nd)
                incidency_mx.update(tl_inmx)
            if tr_nd is not None:
                node_dict.update(tr_nd)
                incidency_mx.update(tr_inmx)
            if bl_nd is not None:
                node_dict.update(bl_nd)
                incidency_mx.update(bl_inmx)
            if br_nd is not None:
                node_dict.update(br_nd)
                incidency_mx.update(br_inmx)

            # Попарное слияние
            if tl_nd is not None and tr_nd is not None:
                tlcenter = self.top_left_quad.center_point
                trcenter = self.top_right_quad.center_point
                for i in range(side_index):
                    tl_node = Point2d(x=(tlcenter.x - self.side_size / 2) + (i * min_side_size + 0.5 * min_side_size), y=(tlcenter.y - self.side_size / 2) + (0 * min_side_size + 0.5 * min_side_size))
                    tr_node = Point2d(x=(trcenter.x - self.side_size / 2) + (i * min_side_size + 0.5 * min_side_size), y=(trcenter.y - self.side_size / 2) + (side_index * min_side_size + 0.5 * min_side_size))
                    tl_uid = f"{tl_node.x},{tl_node.y}"
                    tr_uid = f"{tr_node.x},{tr_node.y}"
                    if tl_uid in tl_nd:
                        if i == 0:
                            if tr_uid in tr_nd:
                                tl_inmx[tl_uid][tr_uid] = min_side_size

                            diag_uid = f"{tr_node.x + min_side_size},{tr_node.y}"
                            if diag_uid in tr_nd:
                                tl_inmx[tl_uid][diag_uid] = diag_distance
                        elif i == side_index:
                            if tr_uid in tr_nd:
                                tl_inmx[tl_uid][tr_uid] = min_side_size

                            diag_uid = f"{tr_node.x - min_side_size},{tr_node.y}"
                            if diag_uid in tr_nd:
                                tl_inmx[tl_uid][diag_uid] = diag_distance
                        else:
                            if tr_uid in tr_nd:
                                tl_inmx[tl_uid][tr_uid] = min_side_size

                            diag_uid = f"{tr_node.x + min_side_size},{tr_node.y}"
                            if diag_uid in tr_nd:
                                tl_inmx[tl_uid][diag_uid] = diag_distance

                            diag_uid = f"{tr_node.x - min_side_size},{tr_node.y}"
                            if diag_uid in tr_nd:
                                tl_inmx[tl_uid][diag_uid] = diag_distance
                    if tr_uid in tr_nd:
                        if i == 0:
                            if tl_uid in tl_nd:
                                tr_inmx[tr_uid][tl_uid] = min_side_size

                            diag_uid = f"{tl_node.x + min_side_size},{tl_node.y}"
                            if diag_uid in tl_nd:
                                tr_inmx[tr_uid][diag_uid] = diag_distance
                        elif i == side_index:
                            if tl_uid in tl_nd:
                                tr_inmx[tr_uid][tl_uid] = min_side_size

                            diag_uid = f"{tl_node.x - min_side_size},{tl_node.y}"
                            if diag_uid in tl_nd:
                                tr_inmx[tr_uid][diag_uid] = diag_distance
                        else:
                            if tl_uid in tl_nd:
                                tr_inmx[tr_uid][tl_uid] = min_side_size

                            diag_uid = f"{tl_node.x + min_side_size},{tl_node.y}"
                            if diag_uid in tl_nd:
                                tr_inmx[tr_uid][diag_uid] = diag_distance

                            diag_uid = f"{tl_node.x - min_side_size},{tl_node.y}"
                            if diag_uid in tl_nd:
                                tr_inmx[tr_uid][diag_uid] = diag_distance
                incidency_mx.update(tl_inmx)
                incidency_mx.update(tr_inmx)
                node_dict.update(tl_nd)
                node_dict.update(tr_nd)
            if br_nd is not None and bl_nd is not None:
                blcenter = self.top_left_quad.center_point
                brcenter = self.top_right_quad.center_point
                for i in range(side_index):
                    bl_node = Point2d(x=(blcenter.x - self.side_size / 2) + (i * min_side_size + 0.5 * min_side_size), y=(blcenter.y - self.side_size / 2) + (0 * min_side_size + 0.5 * min_side_size))
                    br_node = Point2d(x=(brcenter.x - self.side_size / 2) + (i * min_side_size + 0.5 * min_side_size), y=(brcenter.y - self.side_size / 2) + (side_index * min_side_size + 0.5 * min_side_size))
                    bl_uid = f"{bl_node.x},{bl_node.y}"
                    br_uid = f"{br_node.x},{br_node.y}"
                    if bl_uid in bl_nd:
                        if i == 0:
                            if br_uid in br_nd:
                                bl_inmx[bl_uid][br_uid] = min_side_size

                            diag_uid = f"{br_node.x + min_side_size},{br_node.y}"
                            if diag_uid in br_nd:
                                bl_inmx[bl_uid][diag_uid] = diag_distance
                        elif i == side_index:
                            if br_uid in br_nd:
                                bl_inmx[bl_uid][br_uid] = min_side_size

                            diag_uid = f"{br_node.x - min_side_size},{br_node.y}"
                            if diag_uid in br_nd:
                                bl_inmx[bl_uid][diag_uid] = diag_distance
                        else:
                            if br_uid in br_nd:
                                bl_inmx[bl_uid][br_uid] = min_side_size

                            diag_uid = f"{br_node.x + min_side_size},{br_node.y}"
                            if diag_uid in br_nd:
                                bl_inmx[bl_uid][diag_uid] = diag_distance

                            diag_uid = f"{br_node.x - min_side_size},{br_node.y}"
                            if diag_uid in br_nd:
                                bl_inmx[bl_uid][diag_uid] = diag_distance
                    if br_uid in br_nd:
                        if i == 0:
                            if bl_uid in bl_nd:
                                br_inmx[br_uid][bl_uid] = min_side_size

                            diag_uid = f"{bl_node.x + min_side_size},{bl_node.y}"
                            if diag_uid in bl_nd:
                                br_inmx[br_uid][diag_uid] = diag_distance
                        elif i == side_index:
                            if bl_uid in bl_nd:
                                br_inmx[br_uid][bl_uid] = min_side_size

                            diag_uid = f"{bl_node.x - min_side_size},{bl_node.y}"
                            if diag_uid in bl_nd:
                                br_inmx[br_uid][diag_uid] = diag_distance
                        else:
                            if bl_uid in bl_nd:
                                br_inmx[br_uid][bl_uid] = min_side_size

                            diag_uid = f"{bl_node.x + min_side_size},{bl_node.y}"
                            if diag_uid in bl_nd:
                                br_inmx[br_uid][diag_uid] = diag_distance

                            diag_uid = f"{bl_node.x - min_side_size},{bl_node.y}"
                            if diag_uid in bl_nd:
                                br_inmx[br_uid][diag_uid] = diag_distance
                incidency_mx.update(bl_inmx)
                incidency_mx.update(br_inmx)
                node_dict.update(bl_nd)
                node_dict.update(br_nd)
            if tl_nd is not None and bl_nd is not None:
                tlcenter = self.top_left_quad.center_point
                blcenter = self.bottom_left_quad.center_point
                for j in range(side_index):
                    tl_node = Point2d(x=(tlcenter.x - self.side_size / 2) + (0 * min_side_size + 0.5 * min_side_size), y=(tlcenter.y - self.side_size / 2) + (j * min_side_size + 0.5 * min_side_size))
                    bl_node = Point2d(x=(tlcenter.x - self.side_size / 2) + (side_index * min_side_size + 0.5 * min_side_size), y=(tlcenter.y - self.side_size / 2) + (j * min_side_size + 0.5 * min_side_size))
                    tl_uid = f"{tl_node.x},{tl_node.y}"
                    bl_uid = f"{bl_node.x},{bl_node.y}"
                    if bl_uid in bl_nd:
                        if i == 0:
                            if tl_uid in tl_nd:
                                bl_inmx[bl_uid][tl_uid] = min_side_size

                            diag_uid = f"{tl_node.x},{tl_node.y + min_side_size}"
                            if diag_uid in tl_nd:
                                bl_inmx[bl_uid][diag_uid] = diag_distance
                        elif i == side_index:
                            if tl_uid in tl_nd:
                                bl_inmx[bl_uid][tl_uid] = min_side_size
                            
                            diag_uid = f"{tl_node.x},{tl_node.y - min_side_size}"
                            if diag_uid in tl_nd:
                                bl_inmx[bl_uid][diag_uid] = diag_distance
                        else:
                            if tl_uid in tl_nd:
                                bl_inmx[bl_uid][tl_uid] = min_side_size

                            diag_uid = f"{tl_node.x},{tl_node.y + min_side_size}"
                            if diag_uid in tl_nd:
                                bl_inmx[bl_uid][diag_uid] = diag_distance

                            diag_uid = f"{tl_node.x},{tl_node.y - min_side_size}"
                            if diag_uid in tl_nd:
                                bl_inmx[bl_uid][diag_uid] = diag_distance
                    if tl_uid in tl_nd:
                        if i == 0:
                            if bl_uid in bl_nd:
                                tl_inmx[tl_uid][bl_uid] = min_side_size

                            diag_uid = f"{bl_node.x},{bl_node.y + min_side_size}"
                            if diag_uid in bl_nd:
                                tl_inmx[tl_uid][diag_uid] = diag_distance
                        elif i == side_index:
                            if bl_uid in bl_nd:
                                tl_inmx[tl_uid][bl_uid] = min_side_size
                            
                            diag_uid = f"{bl_node.x},{bl_node.y - min_side_size}"
                            if diag_uid in bl_nd:
                                tl_inmx[tl_uid][diag_uid] = diag_distance
                        else:
                            if bl_uid in bl_nd:
                                tl_inmx[tl_uid][bl_uid] = min_side_size

                            diag_uid = f"{bl_node.x},{bl_node.y + min_side_size}"
                            if diag_uid in bl_nd:
                                tl_inmx[tl_uid][diag_uid] = diag_distance

                            diag_uid = f"{bl_node.x},{bl_node.y - min_side_size}"
                            if diag_uid in bl_nd:
                                tl_inmx[tl_uid][diag_uid] = diag_distance
                incidency_mx.update(tl_inmx)
                incidency_mx.update(bl_inmx)
                node_dict.update(tl_nd)
                node_dict.update(bl_nd)
            if tr_nd is not None and br_nd is not None:
                tlcenter = self.top_left_quad.center_point
                blcenter = self.bottom_left_quad.center_point
                for j in range(side_index):
                    tr_node = Point2d(x=(tlcenter.x - self.side_size / 2) + (0 * min_side_size + 0.5 * min_side_size), y=(tlcenter.y - self.side_size / 2) + (j * min_side_size + 0.5 * min_side_size))
                    br_node = Point2d(x=(tlcenter.x - self.side_size / 2) + (side_index * min_side_size + 0.5 * min_side_size), y=(tlcenter.y - self.side_size / 2) + (j * min_side_size + 0.5 * min_side_size))
                    tr_uid = f"{tr_node.x},{tr_node.y}"
                    br_uid = f"{br_node.x},{br_node.y}"
                    if br_uid in br_nd:
                        if i == 0:
                            if tr_uid in tr_nd:
                                br_inmx[br_uid][tr_uid] = min_side_size

                            diag_uid = f"{tr_node.x},{tr_node.y + min_side_size}"
                            if diag_uid in tr_nd:
                                br_inmx[br_uid][diag_uid] = diag_distance
                        elif i == side_index:
                            if tr_uid in tr_nd:
                                br_inmx[br_uid][tr_uid] = min_side_size
                            
                            diag_uid = f"{tr_node.x},{tr_node.y - min_side_size}"
                            if diag_uid in tr_nd:
                                br_inmx[br_uid][diag_uid] = diag_distance
                        else:
                            if tr_uid in tr_nd:
                                br_inmx[br_uid][tr_uid] = min_side_size

                            diag_uid = f"{tr_node.x},{tr_node.y + min_side_size}"
                            if diag_uid in tr_nd:
                                br_inmx[br_uid][diag_uid] = diag_distance

                            diag_uid = f"{tr_node.x},{tr_node.y - min_side_size}"
                            if diag_uid in tr_nd:
                                br_inmx[br_uid][diag_uid] = diag_distance
                    if tr_uid in tr_nd:
                        if i == 0:
                            if br_uid in br_nd:
                                tr_inmx[tr_uid][br_uid] = min_side_size

                            diag_uid = f"{br_node.x},{br_node.y + min_side_size}"
                            if diag_uid in br_nd:
                                tr_inmx[tr_uid][diag_uid] = diag_distance
                        elif i == side_index:
                            if br_uid in br_nd:
                                tr_inmx[tr_uid][br_uid] = min_side_size
                            
                            diag_uid = f"{br_node.x},{br_node.y - min_side_size}"
                            if diag_uid in br_nd:
                                tr_inmx[tr_uid][diag_uid] = diag_distance
                        else:
                            if br_uid in br_nd:
                                tr_inmx[tr_uid][br_uid] = min_side_size

                            diag_uid = f"{br_node.x},{br_node.y + min_side_size}"
                            if diag_uid in br_nd:
                                tr_inmx[tr_uid][diag_uid] = diag_distance

                            diag_uid = f"{br_node.x},{br_node.y - min_side_size}"
                            if diag_uid in br_nd:
                                tr_inmx[tr_uid][diag_uid] = diag_distance
                incidency_mx.update(tr_inmx)
                incidency_mx.update(br_inmx)
                node_dict.update(tr_nd)
                node_dict.update(br_nd)
                # print(node_dict)

            return node_dict, incidency_mx
        else:
            if self.data == included:
                side_size_tmp = copy(self.side_size)
                counter = 0
                while (side_size_tmp != min_side_size):
                    side_size_tmp /= 2
                    counter += 1
                
                side_index = 2 ** counter
                
                diag_distance = min_side_size * math.sqrt(2)

                for i in range(side_index):
                    for j in range(side_index):
                        node = Point2d(x=(self.center_point.x - self.side_size / 2) + (i * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + (j * min_side_size + 0.5 * min_side_size))
                        uid = f"{node.x},{node.y}"
                        # node_dict.update({uid: node})
                        node_dict[uid] = node
                        incidency_mx[uid] = {}
                        
                        # if counter != 0:
                        #     if (i, j) == (0,0) and False: # bottom right corner
                        #         # pass
                        #         ortho_neighbour1 = Point2d(x=(self.center_point.x - self.side_size / 2) + (0 * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + (1 * min_side_size + 0.5 * min_side_size))
                        #         uid1 = f"{ortho_neighbour1.x},{ortho_neighbour1.y}"
                        #         ortho_neighbour2 = Point2d(x=(self.center_point.x - self.side_size / 2) + (1 * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + (0 * min_side_size + 0.5 * min_side_size))
                        #         uid2 = f"{ortho_neighbour2.x},{ortho_neighbour2.y}"
                        #         diag_neighbour3 = Point2d(x=(self.center_point.x - self.side_size / 2) + (1 * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + (1 * min_side_size + 0.5 * min_side_size))
                        #         uid3 = f"{diag_neighbour3.x},{diag_neighbour3.y}"
                        #         incidency_mx[uid][uid1] = min_side_size
                        #         incidency_mx[uid][uid2] = min_side_size
                        #         incidency_mx[uid][uid3] = diag_distance
                        #     # if (i, j) == (side_index,0):
                        #     elif (i, j) == (side_index,0) and False: # top right corner
                        #         ortho_neighbour1 = Point2d(x=(self.center_point.x - self.side_size / 2) + (side_index * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + (1 * min_side_size + 0.5 * min_side_size))
                        #         uid1 = f"{ortho_neighbour1.x},{ortho_neighbour1.y}"
                        #         ortho_neighbour2 = Point2d(x=(self.center_point.x - self.side_size / 2) + (side_index-1 * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + (0 * min_side_size + 0.5 * min_side_size))
                        #         uid2 = f"{ortho_neighbour2.x},{ortho_neighbour2.y}"
                        #         diag_neighbour3 = Point2d(x=(self.center_point.x - self.side_size / 2) + (side_index-1 * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + (1 * min_side_size + 0.5 * min_side_size))
                        #         uid3 = f"{diag_neighbour3.x},{diag_neighbour3.y}"
                        #         incidency_mx[uid][uid1] = min_side_size
                        #         incidency_mx[uid][uid2] = min_side_size
                        #         incidency_mx[uid][uid3] = diag_distance
                        #     elif (i, j) == (0,side_index) and False: # bottom left corner
                        #         ortho_neighbour1 = Point2d(x=(self.center_point.x - self.side_size / 2) + (1 * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + (side_index * min_side_size + 0.5 * min_side_size))
                        #         uid1 = f"{ortho_neighbour1.x},{ortho_neighbour1.y}"
                        #         ortho_neighbour2 = Point2d(x=(self.center_point.x - self.side_size / 2) + (0 * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + (side_index-1 * min_side_size + 0.5 * min_side_size))
                        #         uid2 = f"{ortho_neighbour2.x},{ortho_neighbour2.y}"
                        #         diag_neighbour3 = Point2d(x=(self.center_point.x - self.side_size / 2) + (1 * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + (side_index-1 * min_side_size + 0.5 * min_side_size))
                        #         uid3 = f"{diag_neighbour3.x},{diag_neighbour3.y}"
                        #         incidency_mx[uid][uid1] = min_side_size
                        #         incidency_mx[uid][uid2] = min_side_size
                        #         incidency_mx[uid][uid3] = diag_distance
                        #     elif (i, j) == (side_index,side_index) and False: # top left corner
                        #         ortho_neighbour1 = Point2d(x=(self.center_point.x - self.side_size / 2) + (side_index-1 * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + (side_index * min_side_size + 0.5 * min_side_size))
                        #         uid1 = f"{ortho_neighbour1.x},{ortho_neighbour1.y}"
                        #         ortho_neighbour2 = Point2d(x=(self.center_point.x - self.side_size / 2) + (side_index * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + (side_index-1 * min_side_size + 0.5 * min_side_size))
                        #         uid2 = f"{ortho_neighbour2.x},{ortho_neighbour2.y}"
                        #         diag_neighbour3 = Point2d(x=(self.center_point.x - self.side_size / 2) + (side_index-1 * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + (side_index-1 * min_side_size + 0.5 * min_side_size))
                        #         uid3 = f"{diag_neighbour3.x},{diag_neighbour3.y}"
                        #         incidency_mx[uid][uid1] = min_side_size
                        #         incidency_mx[uid][uid2] = min_side_size
                        #         incidency_mx[uid][uid3] = diag_distance
                        #     elif (j == 0 and i != 0 and i != side_index) and False: # right side
                        #         ortho_neighbour1 = Point2d(x=(self.center_point.x - self.side_size / 2) + (i * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + (1 * min_side_size + 0.5 * min_side_size))
                        #         uid1 = f"{ortho_neighbour1.x},{ortho_neighbour1.y}"
                        #         diag_neighbour2 = Point2d(x=(self.center_point.x - self.side_size / 2) + ((i+1) * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + (1 * min_side_size + 0.5 * min_side_size))
                        #         uid2 = f"{diag_neighbour2.x},{diag_neighbour2.y}"
                        #         diag_neighbour3 = Point2d(x=(self.center_point.x - self.side_size / 2) + ((i-1) * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + (1 * min_side_size + 0.5 * min_side_size))
                        #         uid3 = f"{diag_neighbour3.x},{diag_neighbour3.y}"
                        #         ortho_neighbour2 = Point2d(x=(self.center_point.x - self.side_size / 2) + ((i+1) * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + (0 * min_side_size + 0.5 * min_side_size))
                        #         uid4 = f"{ortho_neighbour2.x},{ortho_neighbour2.y}"
                        #         ortho_neighbour3 = Point2d(x=(self.center_point.x - self.side_size / 2) + ((i-1) * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + (0 * min_side_size + 0.5 * min_side_size))
                        #         uid5 = f"{ortho_neighbour3.x},{ortho_neighbour3.y}"
                        #         incidency_mx[uid][uid1] = min_side_size
                        #         incidency_mx[uid][uid2] = diag_distance
                        #         incidency_mx[uid][uid3] = diag_distance
                        #         incidency_mx[uid][uid4] = min_side_size
                        #         incidency_mx[uid][uid5] = min_side_size
                        #     elif (i == side_index and j != 0 and j != side_index) and False: # top side
                        #         ortho_neighbour1 = Point2d(x=(self.center_point.x - self.side_size / 2) + ((side_index - 1) * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + (j * min_side_size + 0.5 * min_side_size))
                        #         uid1 = f"{ortho_neighbour1.x},{ortho_neighbour1.y}"
                        #         diag_neighbour2 = Point2d(x=(self.center_point.x - self.side_size / 2) + ((side_index - 1) * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + ((j + 1) * min_side_size + 0.5 * min_side_size))
                        #         uid2 = f"{diag_neighbour2.x},{diag_neighbour2.y}"
                        #         diag_neighbour3 = Point2d(x=(self.center_point.x - self.side_size / 2) + ((side_index - 1) * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + ((j - 1) * min_side_size + 0.5 * min_side_size))
                        #         uid3 = f"{diag_neighbour3.x},{diag_neighbour3.y}"
                        #         ortho_neighbour2 = Point2d(x=(self.center_point.x - self.side_size / 2) + ((side_index) * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + ((j + 1) * min_side_size + 0.5 * min_side_size))
                        #         uid4 = f"{ortho_neighbour2.x},{ortho_neighbour2.y}"
                        #         ortho_neighbour3 = Point2d(x=(self.center_point.x - self.side_size / 2) + ((side_index) * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + ((j - 1) * min_side_size + 0.5 * min_side_size))
                        #         uid5 = f"{ortho_neighbour3.x},{ortho_neighbour3.y}"
                        #         incidency_mx[uid][uid1] = min_side_size
                        #         incidency_mx[uid][uid2] = diag_distance
                        #         incidency_mx[uid][uid3] = diag_distance
                        #         incidency_mx[uid][uid4] = min_side_size
                        #         incidency_mx[uid][uid5] = min_side_size
                        #     elif (j == side_index and i != 0 and i != side_index) and False: # left side
                        #         ortho_neighbour1 = Point2d(x=(self.center_point.x - self.side_size / 2) + (i * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + ((side_index - 1) * min_side_size + 0.5 * min_side_size))
                        #         uid1 = f"{ortho_neighbour1.x},{ortho_neighbour1.y}"
                        #         diag_neighbour2 = Point2d(x=(self.center_point.x - self.side_size / 2) + ((i + 1) * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + ((side_index - 1) * min_side_size + 0.5 * min_side_size))
                        #         uid2 = f"{diag_neighbour2.x},{diag_neighbour2.y}"
                        #         diag_neighbour3 = Point2d(x=(self.center_point.x - self.side_size / 2) + ((i - 1) * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + ((side_index - 1) * min_side_size + 0.5 * min_side_size))
                        #         uid3 = f"{diag_neighbour3.x},{diag_neighbour3.y}"
                        #         ortho_neighbour2 = Point2d(x=(self.center_point.x - self.side_size / 2) + ((i + 1) * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + (side_index * min_side_size + 0.5 * min_side_size))
                        #         uid4 = f"{ortho_neighbour2.x},{ortho_neighbour2.y}"
                        #         ortho_neighbour3 = Point2d(x=(self.center_point.x - self.side_size / 2) + ((i - 1) * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + (side_index * min_side_size + 0.5 * min_side_size))
                        #         uid5 = f"{ortho_neighbour3.x},{ortho_neighbour3.y}"
                        #         incidency_mx[uid][uid1] = min_side_size
                        #         incidency_mx[uid][uid4] = min_side_size
                        #         incidency_mx[uid][uid5] = min_side_size
                        #         incidency_mx[uid][uid2] = diag_distance
                        #         incidency_mx[uid][uid3] = diag_distance
                        #     elif (i == 0 and j != 0 and j != side_index) and False: # bottom side
                        #         ortho_neighbour1 = Point2d(x=(self.center_point.x - self.side_size / 2) + (1 * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + (j * min_side_size + 0.5 * min_side_size))
                        #         uid1 = f"{ortho_neighbour1.x},{ortho_neighbour1.y}"
                        #         diag_neighbour2 = Point2d(x=(self.center_point.x - self.side_size / 2) + (1 * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + ((j + 1) * min_side_size + 0.5 * min_side_size))
                        #         uid2 = f"{diag_neighbour2.x},{diag_neighbour2.y}"
                        #         diag_neighbour3 = Point2d(x=(self.center_point.x - self.side_size / 2) + (1 * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + ((j - 1) * min_side_size + 0.5 * min_side_size))
                        #         uid3 = f"{diag_neighbour3.x},{diag_neighbour3.y}"
                        #         ortho_neighbour2 = Point2d(x=(self.center_point.x - self.side_size / 2) + (0 * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + ((j + 1) * min_side_size + 0.5 * min_side_size))
                        #         uid4 = f"{ortho_neighbour2.x},{ortho_neighbour2.y}"
                        #         ortho_neighbour3 = Point2d(x=(self.center_point.x - self.side_size / 2) + (0 * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + ((j - 1) * min_side_size + 0.5 * min_side_size))
                        #         uid5 = f"{ortho_neighbour3.x},{ortho_neighbour3.y}"
                        #         incidency_mx[uid][uid1] = min_side_size
                        #         incidency_mx[uid][uid4] = min_side_size
                        #         incidency_mx[uid][uid5] = min_side_size
                        #         incidency_mx[uid][uid2] = diag_distance
                        #         incidency_mx[uid][uid3] = diag_distance
                        #     else: # inner node
                        ortho_neighbour1 = Point2d(x=(self.center_point.x - self.side_size / 2) + ((i + 1) * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + ((j + 0) * min_side_size + 0.5 * min_side_size))
                        uid1 = f"{ortho_neighbour1.x},{ortho_neighbour1.y}"
                        ortho_neighbour2 = Point2d(x=(self.center_point.x - self.side_size / 2) + ((i - 1) * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + ((j + 0) * min_side_size + 0.5 * min_side_size))
                        uid2 = f"{ortho_neighbour2.x},{ortho_neighbour2.y}"
                        ortho_neighbour3 = Point2d(x=(self.center_point.x - self.side_size / 2) + ((i + 0) * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + ((j + 1) * min_side_size + 0.5 * min_side_size))
                        uid3 = f"{ortho_neighbour3.x},{ortho_neighbour3.y}"
                        ortho_neighbour4 = Point2d(x=(self.center_point.x - self.side_size / 2) + ((i + 0) * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + ((j - 1) * min_side_size + 0.5 * min_side_size))
                        uid4 = f"{ortho_neighbour4.x},{ortho_neighbour4.y}"
                        incidency_mx[uid][uid1] = min_side_size
                        incidency_mx[uid][uid2] = min_side_size
                        incidency_mx[uid][uid3] = min_side_size
                        incidency_mx[uid][uid4] = min_side_size

                        diag_neighbour1 = Point2d(x=(self.center_point.x - self.side_size / 2) + ((i + 1) * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + ((j + 1) * min_side_size + 0.5 * min_side_size))
                        uid5 = f"{diag_neighbour1.x},{diag_neighbour1.y}"
                        diag_neighbour2 = Point2d(x=(self.center_point.x - self.side_size / 2) + ((i + 1) * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + ((j - 1) * min_side_size + 0.5 * min_side_size))
                        uid6 = f"{diag_neighbour2.x},{diag_neighbour2.y}"
                        diag_neighbour3 = Point2d(x=(self.center_point.x - self.side_size / 2) + ((i - 1) * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + ((j - 1) * min_side_size + 0.5 * min_side_size))
                        uid7 = f"{diag_neighbour3.x},{diag_neighbour3.y}"
                        diag_neighbour4 = Point2d(x=(self.center_point.x - self.side_size / 2) + ((i - 1) * min_side_size + 0.5 * min_side_size), y=(self.center_point.y - self.side_size / 2) + ((j + 1) * min_side_size + 0.5 * min_side_size))
                        uid8 = f"{diag_neighbour4.x},{diag_neighbour4.y}"
                        incidency_mx[uid][uid5] = diag_distance
                        incidency_mx[uid][uid6] = diag_distance
                        incidency_mx[uid][uid7] = diag_distance
                        incidency_mx[uid][uid8] = diag_distance
                return node_dict, incidency_mx
            else:
                return None, None

    def _filter_spear_edges(self, node_list, node_incidency_matrix):
        node_incidency_matrix_tmp = deepcopy(node_incidency_matrix)
        # Draw edges
        for i, connections in node_incidency_matrix.items():
            node1 = node_list[i]
            uid1 = f"{node1.x},{node1.y}"
            for j, distance in connections.items():
                if j not in node_list:
                    del(node_incidency_matrix_tmp[uid1][j])
        return node_list, node_incidency_matrix_tmp



    def generate_included_incidency_matrix(self):
        min_side_size = self._get_min_side_size()
        nodelist, incidency_mx = self._generate_included_incidency_matrix(min_side_size)
        nodelist, incidency_mx = self._filter_spear_edges(nodelist, incidency_mx)
        return nodelist, incidency_mx

