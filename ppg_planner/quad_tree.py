# Initial code was taken from https://www.geeksforgeeks.org/quad-tree/
from ppg_planner.geometry import Point2d, SquareRegion, LineLike, GeometryDrawer, Line2d
import numpy as np
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
        self.current_size_downstep_level = current_size_downstep_level

        self.divided = False

        self.top_left_quad = None
        self.top_right_quad = None
        self.bottom_right_quad = None
        self.bottom_left_quad = None

        self.data = []
    
    def divide_by_line(self, line: LineLike):
        start_division = False

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

    def visualize_quad_tree(self, drawer=None, canvas=None, color=(255,255,255)):
        write = False
        if drawer is None:
            write = True
            drawer = GeometryDrawer()
            canvas = np.zeros((int(self.side_size), int(self.side_size), 3))
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


# drawer = GeometryDrawer()
# canvas = np.zeros((1000, 1000, 3))

# tree = Quad(Point2d(500, 500), 1000, 9)
# line = Line2d(Point2d(x=400, y=400), Point2d(x=700, y=500))

# tree.divide_by_line(line)
# tree.visualize_quad_tree(drawer, canvas, (255, 0, 0))
# drawer.draw_line(line, canvas, (0,0,255))


# canvas = cv.rotate(canvas, cv.ROTATE_180)
# cv.imwrite("result.png", canvas)
























# # The main quadtree class
# class Quad:
#     def __init__(self, topL, botR):
#         self.topLeft = topL
#         self.botRight = botR
#         self.n = None
#         self.topLeftTree = None
#         self.topRightTree = None
#         self.botLeftTree = None
#         self.botRightTree = None

#     # Insert a node into the quadtree
#     def insert(self, node):
#         if node is None:
#             return

#         # Current quad cannot contain it
#         if not self.inBoundary(node.pos):
#             return

#         # We are at a quad of unit area
#         # We cannot subdivide this quad further
#         if abs(self.topLeft.x - self.botRight.x) <= 1 and abs(self.topLeft.y - self.botRight.y) <= 1:
#             if self.n is None:
#                 self.n = node
#             return

#         if (self.topLeft.x + self.botRight.x) / 2 >= node.pos.x:
#             # Indicates topLeftTree
#             if (self.topLeft.y + self.botRight.y) / 2 >= node.pos.y:
#                 if self.topLeftTree is None:
#                     self.topLeftTree = Quad(self.topLeft, Point2d((self.topLeft.x + self.botRight.x) / 2, (self.topLeft.y + self.botRight.y) / 2))
#                 self.topLeftTree.insert(node)
#             # Indicates botLeftTree
#             else:
#                 if self.botLeftTree is None:
#                     self.botLeftTree = Quad(Point2d(self.topLeft.x, (self.topLeft.y + self.botRight.y) / 2), Point2d((self.topLeft.x + self.botRight.x) / 2, self.botRight.y))
#                 self.botLeftTree.insert(node)
#         else:
#             # Indicates topRightTree
#             if (self.topLeft.y + self.botRight.y) / 2 >= node.pos.y:
#                 if self.topRightTree is None:
#                     self.topRightTree = Quad(Point2d((self.topLeft.x + self.botRight.x) / 2, self.topLeft.y), Point2d(self.botRight.x, (self.topLeft.y + self.botRight.y) / 2))
#                 self.topRightTree.insert(node)
#             # Indicates botRightTree
#             else:
#                 if self.botRightTree is None:
#                     self.botRightTree = Quad(Point2d((self.topLeft.x + self.botRight.x) / 2, (self.topLeft.y + self.botRight.y) / 2), self.botRight)
#                 self.botRightTree.insert(node)

#     # Find a node in a quadtree
#     def search(self, p):
#         # Current quad cannot contain it
#         if not self.inBoundary(p):
#             return 0  # Return 0 if point is not found

#         # We are at a quad of unit length
#         # We cannot subdivide this quad further
#         if self.n is not None:
#             return self.n

#         if (self.topLeft.x + self.botRight.x) / 2 >= p.x:
#             # Indicates topLeftTree
#             if (self.topLeft.y + self.botRight.y) / 2 >= p.y:
#                 if self.topLeftTree is None:
#                     return 0
#                 return self.topLeftTree.search(p)
#             # Indicates botLeftTree
#             else:
#                 if self.botLeftTree is None:
#                     return 0
#                 return self.botLeftTree.search(p)
#         else:
#             # Indicates topRightTree
#             if (self.topLeft.y + self.botRight.y) / 2 >= p.y:
#                 if self.topRightTree is None:
#                     return 0
#                 return self.topRightTree.search(p)
#             # Indicates botRightTree
#             else:
#                 if self.botRightTree is None:
#                     return 0
#                 return self.botRightTree.search(p)

#     # Check if current quadtree contains the point
#     def inBoundary(self, p):
#         return p.x >= self.topLeft.x and p.x <= self.botRight.x and p.y >= self.topLeft.y and p.y <= self.botRight.y

# # Driver program
# center = Quad(Point2d(0, 0), Point2d(8, 8))
# a = Node(Point2d(1, 1), 1)
# b = Node(Point2d(2, 5), 2)
# c = Node(Point2d(7, 6), 3)
# center.insert(a)
# center.insert(b)
# center.insert(c)
# print("Node a:", center.search(Point2d(1, 1)).data)
# print("Node b:", center.search(Point2d(2, 5)).data)
# print("Node c:", center.search(Point2d(7, 6)).data)
# print("Non-existing node:", center.search(Point2d(5, 5)))