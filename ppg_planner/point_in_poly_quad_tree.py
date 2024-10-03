# Implementing an algorithm from https://alienryderflex.com/polygon/

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PolygonStamped, Point32
from ppg_planner_interfaces.msg import Poly
from ppg_planner_interfaces.srv import PolyGrid
import geometry as gm


def point_in_poly(point: tuple, poly: list) -> bool:
    """
    Args:
        point: (x, y)
        poly: [(x1, y1), (x2, y2), ... , (xn, yn)]
    """

    if len(point) != 2:
        print("Error! Point in poly")
        return False
    
    crossed_sides = []

    for i in range(len(poly) - 1):
        if point[1] >= poly[i][1] and point[1] <= poly[i + 1][1]:
            crossed_sides.append((poly[i], poly[i + 1]))
        elif point[1] <= poly[i][1] and point[1] >= poly[i + 1][1]:
            crossed_sides.append((poly[i], poly[i + 1]))

    j = len(poly) - 1

    if point[1] >= poly[0][1] and point[1] <= poly[j][1]:
        crossed_sides.append((poly[0], poly[j]))
    elif point[1] <= poly[0][1] and point[1] >= poly[j][1]:
        crossed_sides.append((poly[0], poly[j]))

    crossing_points = []

    for side in crossed_sides:
        x1, y1 = side[0]
        x2, y2 = side[1]

        if y1 == y2:
            continue

        x = x1 + (point[1] - y1) * (x2 - x1) / (y2 - y1)
        y = point[1]

        if (x, y) not in crossing_points:
            crossing_points.append((x, y))

    left_c, right_c = 0, 0

    for cpoint in crossing_points:
        xcp, ycp = cpoint
        x, y = point

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



class PPGPlannerNode(Node):

    def __init__(self):
        super().__init__('ppg_planner_node')
        self.points_publisher = self.create_publisher(MarkerArray, 'points_in_poly', 10)
        self.poly_publisher = self.create_publisher(PolygonStamped, 'poly', 10)
        self.polygrid_srv = self.create_service(PolyGrid, 'ppg_service', self.ppg_service_clb)
        timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)


    def ppg_service_clb(self, request, response):
        print("entered srv clb")
        print("generating points...")

        zone_raw = []
        for pt in request.zone.poly:
            zone_raw.append((pt.x, pt.y))
        
        exclude_list_raw = []
        for i in range(len(request.exclusion_zones)):
            exclude_list_raw.append([])
            for pt in request.exclusion_zones[i].poly:
                exclude_list_raw[i].append((pt.x, pt.y))

        points_raw = self.generate_points(zone_raw, exclude_list_raw, request.step)


        for point_raw in points_raw:
            point = Point32()
            point.x = point_raw[0]
            point.y = point_raw[1]
            point.z = 0.0
            response.point_grid.append(point)
        
        print("finished")

        return response


    def generate_points(self, poly, exclude_poly_list, step):
        """
        Args:
            step: float
            poly: list - [(x1, y1), (x2, y2), ... , (xn, yn)]
            exclude_poly_list: list - [poly_1, poly_2, ... , poly_n]
        """

        poly_msg = PolygonStamped()
        poly_msg.header.frame_id = "map"
        poly_msg.header.stamp = self.get_clock().now().to_msg()

        for pt in poly:
            pt_msg = Point32()
            pt_msg.x = float(pt[0])
            pt_msg.y = float(pt[1])
            pt_msg.z = 0.0
            poly_msg.polygon.points.append(pt_msg)

        self.poly_publisher.publish(poly_msg)

        for ex_poly in exclude_poly_list:
            poly_msg = PolygonStamped()
            poly_msg.header.frame_id = "map"
            poly_msg.header.stamp = self.get_clock().now().to_msg()
            for pt in ex_poly:
                pt_msg = Point32()
                pt_msg.x = float(pt[0])
                pt_msg.y = float(pt[1])
                pt_msg.z = 0.0
                poly_msg.polygon.points.append(pt_msg)
            self.poly_publisher.publish(poly_msg)
        
        x_min, y_min = None, None
        x_max, y_max = None, None

        for side in poly:
            if x_min is None or side[0] < x_min:
                x_min = side[0]
            if y_min is None or side[1] < y_min:
                y_min = side[1]
            if x_max is None or side[0] > x_max:
                x_max = side[0]
            if y_max is None or side[1] > y_max:
                y_max = side[1]
        
        points_in_area = []
        y_curr, x_curr = y_min, x_min
        while y_curr <= y_max:
            while x_curr <= x_max:
                if point_in_poly((x_curr, y_curr), poly=poly):
                    include = True
                    for excluded_poly in exclude_poly_list:
                        if point_in_poly((x_curr, y_curr), excluded_poly):
                            include = False
                            break
                    if include:
                        points_in_area.append((x_curr, y_curr))
                x_curr += step
            y_curr += step
            x_curr = x_min

        array = MarkerArray()
        for i in range(len(points_in_area)):
            msg = Marker()

            msg.type = 2
            msg.id = i

            msg.scale.x = 0.1
            msg.scale.y = 0.1
            msg.scale.z = 0.1

            msg.color.r = 1.0
            msg.color.g = 0.0
            msg.color.b = 0.0
            msg.color.a = 1.0

            msg.pose.position.x = float(points_in_area[i][0])
            msg.pose.position.y = float(points_in_area[i][1])
            msg.pose.position.z = 0.0

            msg.pose.orientation.x = 0.0 
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0 

            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            array.markers.append(msg)

        self.points_publisher.publish(array)
        print("Published points")

        return points_in_area


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