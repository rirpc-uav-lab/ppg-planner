# Implementing an algorithm from https://alienryderflex.com/polygon/

import rclpy
import numpy as np
import cv2 as cv
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PolygonStamped, Point32
from ppg_planner_interfaces.msg import Poly
from ppg_planner_interfaces.srv import PolyGrid
import ppg_planner.geometry as gm
import ppg_planner.quad_tree as quad




    # def point_in_poly(self, point: Point2d) -> bool:
    #     crossed_sides = []

    #     for i in range(len(self.sides)):
    #         if point.y >= self.sides.p1.y and point.y <= self.sides.p2.y:
    #             crossed_sides.append((self.sides.p1, self.sides.p2))
    #         elif point.y <= self.sides.p1.y and point.y >= self.sides.p2.y:
    #             crossed_sides.append((self.sides.p1, self.sides.p2))

    #     # j = len(self.sides) - 1

    #     # if point.y >= self.sides[0][1] and point.y <= self.sides[j][1]:
    #     #     crossed_sides.append((self.sides[0], self.sides[j]))
    #     # elif point.y <= self.sides[0][1] and point.y >= self.sides[j][1]:
    #     #     crossed_sides.append((self.sides[0], self.sides[j]))

    #     crossing_points = []

    #     for side in crossed_sides:
    #         x1, y1 = side.p1.x, side.p1.y 
    #         x2, y2 = side.p2.x, side.p2.y

    #         if y1 == y2:
    #             continue

    #         x = x1 + (point.y - y1) * (x2 - x1) / (y2 - y1)
    #         y = point.y

    #         if (x, y) not in crossing_points:
    #             crossing_points.append((x, y))

    #     left_c, right_c = 0, 0

    #     for cpoint in crossing_points:
    #         xcp, ycp = cpoint
    #         x, y = point

    #         if x == xcp and y == ycp: 
    #             return True

    #         if xcp < x:
    #             left_c += 1
    #         elif xcp > x:
    #             right_c += 1
    #         else:
    #             return True

    #     if left_c % 2 == 1 and right_c % 2 == 1:
    #         return True
    #     else:
    #         return False


class PPGPlannerNode(Node):
    def __init__(self):
        super().__init__('ppg_planner_node')
        self.points_publisher = self.create_publisher(MarkerArray, 'points_in_poly', 10)
        self.poly_publisher = self.create_publisher(PolygonStamped, 'poly', 10)
        self.polygrid_srv = self.create_service(PolyGrid, 'ppg_service', self.ppg_service_clb)
        self.get_logger().error("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")


    def ppg_service_clb(self, request, response):
        self.get_logger().error("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")

        zone_poly = self.convert_point_list_to_poly(request.zone.poly)
        
        exclude_polygons_list = []
        for exclusion_zone in request.exclusion_zones:
            exclude_polygons_list.append(self.convert_point_list_to_poly(exclusion_zone.poly))

        points_raw = self.generate_points(zone_poly, exclude_polygons_list, request.step)

        return response


    def convert_point_list_to_poly(self, points: list[Point32]) -> gm.Polygon:
        poly_lines = []
        for i in range(len(points)):
            # if i == 0:
            #     continue
            line = None
            if i == len(points) - 1:
                line = gm.Line2d(gm.Point2d(x=points[i].x, y=points[i].y), gm.Point2d(x=points[0].x, y=points[0].y))
                poly_lines.append(line)
            else:
                line = gm.Line2d(gm.Point2d(x=points[i + 1].x, y=points[i + 1].y), gm.Point2d(x=points[i].x, y=points[i].y))
                poly_lines.append(line)
        return gm.Polygon(lines=poly_lines)


    def generate_points(self, zone_poly: gm.Polygon, exclude_poly_list: list[gm.Polygon], step):
        # """
        # Args:
        #     step: float
        #     poly: list - [(x1, y1), (x2, y2), ... , (xn, yn)]
        #     exclude_poly_list: list - [poly_1, poly_2, ... , poly_n]
        # """

        dx = zone_poly.x_max - zone_poly.x_min
        dy = zone_poly.y_max - zone_poly.y_min

        tree = None
        max_dwnstp_lvl = 10
        if dx > dy:
            side_size = dx
            center = gm.Point2d(x=zone_poly.x_min + side_size / 2, y=((zone_poly.y_max - zone_poly.y_min)/2 + zone_poly.y_min))
            tree = quad.Quad(center_point=center, side_size=side_size, max_size_downstep_level=max_dwnstp_lvl)
        else:
            side_size = dy
            center = gm.Point2d(x=((zone_poly.x_max - zone_poly.x_min)/2 + zone_poly.x_min), y=zone_poly.y_min + side_size / 2)
            tree = quad.Quad(center_point=center, side_size=side_size, max_size_downstep_level=max_dwnstp_lvl)

        for line in zone_poly.sides:
            tree.divide_by_line(line)
        
        for zone in exclude_poly_list:
            for line in zone.sides:
                tree.divide_by_line(line)
        
        tree.include_zone(zone_poly)
        for zone in exclude_poly_list:
            tree.exclude_zone(zone)

        tree.visualize_quad_tree(color=(255, 0, 255))




        # poly_msg = PolygonStamped()
        # poly_msg.header.frame_id = "map"
        # poly_msg.header.stamp = self.get_clock().now().to_msg()

        # for pt in poly:
        #     pt_msg = Point32()
        #     pt_msg.x = float(pt[0])
        #     pt_msg.y = float(pt[1])
        #     pt_msg.z = 0.0
        #     poly_msg.polygon.points.append(pt_msg)

        # self.poly_publisher.publish(poly_msg)

        # for ex_poly in exclude_poly_list:
        #     poly_msg = PolygonStamped()
        #     poly_msg.header.frame_id = "map"
        #     poly_msg.header.stamp = self.get_clock().now().to_msg()
        #     for pt in ex_poly:
        #         pt_msg = Point32()
        #         pt_msg.x = float(pt[0])
        #         pt_msg.y = float(pt[1])
        #         pt_msg.z = 0.0
        #         poly_msg.polygon.points.append(pt_msg)
        #     self.poly_publisher.publish(poly_msg)
        
        # x_min, y_min = None, None
        # x_max, y_max = None, None

        # for side in poly:
        #     if x_min is None or side[0] < x_min:
        #         x_min = side[0]
        #     if y_min is None or side[1] < y_min:
        #         y_min = side[1]
        #     if x_max is None or side[0] > x_max:
        #         x_max = side[0]
        #     if y_max is None or side[1] > y_max:
        #         y_max = side[1]
        
        # points_in_area = []
        # y_curr, x_curr = y_min, x_min
        # while y_curr <= y_max:
        #     while x_curr <= x_max:
        #         if point_in_poly((x_curr, y_curr), poly=poly):
        #             include = True
        #             for excluded_poly in exclude_poly_list:
        #                 if point_in_poly((x_curr, y_curr), excluded_poly):
        #                     include = False
        #                     break
        #             if include:
        #                 points_in_area.append((x_curr, y_curr))
        #         x_curr += step
        #     y_curr += step
        #     x_curr = x_min

        # array = MarkerArray()
        # for i in range(len(points_in_area)):
        #     msg = Marker()

        #     msg.type = 2
        #     msg.id = i

        #     msg.scale.x = 0.1
        #     msg.scale.y = 0.1
        #     msg.scale.z = 0.1

        #     msg.color.r = 1.0
        #     msg.color.g = 0.0
        #     msg.color.b = 0.0
        #     msg.color.a = 1.0

        #     msg.pose.position.x = float(points_in_area[i][0])
        #     msg.pose.position.y = float(points_in_area[i][1])
        #     msg.pose.position.z = 0.0

        #     msg.pose.orientation.x = 0.0 
        #     msg.pose.orientation.y = 0.0
        #     msg.pose.orientation.z = 0.0 

        #     msg.header.frame_id = "map"
        #     msg.header.stamp = self.get_clock().now().to_msg()
        #     array.markers.append(msg)

        # self.points_publisher.publish(array)
        # print("Published points")

        # return points_in_area


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = PPGPlannerNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# import quads

# tree = quads.QuadTree((0, 0), 20, 20, capacity=1)

# for i in range(-10, 10):
#     for j in range(-10, 10):
#         tree.insert((i,j))

# # board = quads.QuadTree((4, 4), 8, 8, capacity=1)

# # board.insert((1, 1), data="red")
# # board.insert((0, 0), data="black")
# # board.insert((3, 5), data="red")
# # board.insert((7, 7), data="black")
# # board.insert((4, 4), data="black")

# quads.visualize(tree)