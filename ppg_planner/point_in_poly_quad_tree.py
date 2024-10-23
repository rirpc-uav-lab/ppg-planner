# Implementing an algorithm from https://alienryderflex.com/polygon/
import time
import rclpy
import numpy as np
import cv2 as cv
from rclpy.node import Node
from copy import copy
from geometry_msgs.msg import PolygonStamped, Point32
from ppg_planner_interfaces.msg import Poly
from ppg_planner_interfaces.srv import PolyGrid
import ppg_planner.geometry as gm
import ppg_planner.quad_tree as quad
import ppg_planner.planners as planners

from getpass import getuser
from visualization_msgs.msg import Marker, MarkerArray
from pympler import asizeof


def visualize_node_graph(node_graph, canvas_size=1000, node_radius=2, line_thickness=1):
    # Create a black canvas
    canvas = np.zeros((canvas_size, canvas_size, 3), dtype=np.uint8)

    # Draw edges
    for i, connections in node_graph.node_incidency_matrix.items():
        node1 = node_graph.node_list[i]
        center1 = (int(node1.x), int(node1.y))
        for j, distance in connections.items():
            if j in node_graph.node_list:
                node2 = node_graph.node_list[j]
                center2 = (int(node2.x), int(node2.y))
                cv.line(canvas, center1, center2, (255, 0, 0), line_thickness)  # Draw edge as a white line
            else:
                x, y = j.split(sep=",")
                center2 = (int(float(x)), int(float(y)))
                cv.line(canvas, center1, center2, (0, 0, 255), line_thickness)  # Draw edge as a white line
    # Draw nodes
    for uid, node in node_graph.node_list.items():
        center = (int(node.x), int(node.y))
        cv.circle(canvas, center, node_radius, (0, 255, 0), -1)  # Draw node as a green circle

    # Flip the canvas vertically to match the coordinate system
    canvas = cv.flip(canvas, -1)

    cv.imwrite(f"/home/{getuser()}/Pictures/result_graph.png", canvas)    

    cv.imshow("Node Graph", canvas)
    cv.waitKey(0)
    cv.destroyAllWindows()
    return canvas


class PPGPlannerNode(Node):
    def __init__(self):
        super().__init__('ppg_planner_node')
        self.points_publisher = self.create_publisher(MarkerArray, 'points_in_poly', 10)
        self.poly_publisher = self.create_publisher(PolygonStamped, 'poly', 10)
        self.polygrid_srv = self.create_service(PolyGrid, 'ppg_service', self.ppg_service_clb)
        self.get_logger().error("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")


    def ppg_service_clb(self, request, response):
        self.get_logger().error("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")

        include_polygons_list = []
        for zone in request.zone:
            include_polygons_list.append(self.convert_point_list_to_poly(zone.poly))
        
        exclude_polygons_list = []
        for exclusion_zone in request.exclusion_zones:
            exclude_polygons_list.append(self.convert_point_list_to_poly(exclusion_zone.poly))

        print(f"request.step = {request.step}")

        points_raw = self.generate_points(include_polygons_list, exclude_polygons_list, request.step)

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

    def get_downstep_level_by_side_size(self, side_size, goal_side_size):
        counter = 0
        side_size_tmp = float(copy(side_size))
        goal_side_size_tmp = float(copy(goal_side_size))
        while (side_size_tmp > goal_side_size_tmp):
            side_size_tmp /= 2
            counter += 1
        return counter
    
    def get_poly_list_maxs_and_mins(self, poly_list: list[gm.Polygon]):
        """
        Calculate the maximum and minimum x and y values from a list of polygons.

        Args:
            poly_list (list[gm.Polygon]): List of polygons to extract max and min values from.

        Returns:
            tuple: A tuple containing the maximum x, minimum x, maximum y, and minimum y values.
        """
        x_max = None
        x_min = None
        y_max = None
        y_min = None

        for poly in poly_list:
            if x_max is None:
                x_max = poly.x_max
                x_min = poly.x_min
                y_max = poly.y_max
                y_min = poly.y_min

            if poly.x_max > x_max:
                x_max = poly.x_max
            if poly.x_min > x_min:
                x_min = poly.x_min
            if poly.y_max > y_max:
                y_max = poly.y_max
            if poly.y_min > y_min:
                y_min = poly.y_min

        self.get_logger().info(f"x_max: {x_max}, x_min: {x_min}, y_max: {y_max}, y_min: {y_min}")

        return x_max, x_min, y_max, y_min


    def generate_points(self, zone_poly: list[gm.Polygon], exclude_poly_list: list[gm.Polygon], step):
        # """
        # Args:
        #     step: float
        #     poly: list - [(x1, y1), (x2, y2), ... , (xn, yn)]
        #     exclude_poly_list: list - [poly_1, poly_2, ... , poly_n]
        # """
        start_time = time.time()

        x_max, x_min, y_max, y_min = self.get_poly_list_maxs_and_mins(zone_poly)
        self.get_logger().info(f"x_max: {x_max}, x_min: {x_min}, y_max: {y_max}, y_min: {y_min}")

        dx = x_max - x_min
        dy = y_max - y_min
        self.get_logger().info(f"dx: {dx}, dy: {dy}")

        tree = None
        if dx > dy:
            max_dwnstp_lvl = self.get_downstep_level_by_side_size(dx, float(step))
            side_size = dx
            center = gm.Point2d(x=x_min + side_size / 2, y=(dy/2 + y_min))
            tree = quad.Quad(center_point=center, side_size=side_size, max_size_downstep_level=max_dwnstp_lvl)
        else:
            max_dwnstp_lvl = self.get_downstep_level_by_side_size(dy, float(step))
            side_size = dy
            center = gm.Point2d(x=(dx/2 + x_min), y=y_min + side_size / 2)
            tree = quad.Quad(center_point=center, side_size=side_size, max_size_downstep_level=max_dwnstp_lvl)

        for zone in zone_poly:
            for line in zone.sides:
                tree.divide_by_line(line)
        for zone in exclude_poly_list:
            for line in zone.sides:
                tree.divide_by_line(line)
        
        for zone in zone_poly:
            tree.include_zone(zone)
        for zone in exclude_poly_list:
            tree.exclude_zone(zone)
        # tree.divide_until_all_quads_are_smallest_size(divide_only_included=True)
        for zone in zone_poly:
            tree.include_zone(zone)
        for zone in exclude_poly_list:
            tree.exclude_zone(zone)

        min_side_size = tree.get_min_side_size()
        nd, i_mx = tree.generate_included_incidency_matrix(min_side_size)

        a = planners.NodeGraph(nd, i_mx)

        # print(f"memory = {asizeof.asizeof(tree) / 8 / 1024 / 1024} Mb")
        # print(f"tree.get_included_node_list().size() = {")

        # a = planners.NodeGraph(tree.get_included_node_list())
        self.get_logger().warn(f"time = {time.time() - start_time}")
        visualize_node_graph(a)
        
        tree.visualize_quad_tree(color=(255, 0, 255))





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