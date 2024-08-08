# Implementing an algorithm from https://alienryderflex.com/polygon/

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PolygonStamped, Point32


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
        super().__init__('minimal_publisher')
        self.points_publisher = self.create_publisher(MarkerArray, 'points_in_poly', 10)
        self.poly_publisher = self.create_publisher(PolygonStamped, 'poly', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):

        step = 0.3
        poly = [(0,0), (0, 1), (3,7), (4,5), (6, 0.4)]
        exclude_poly_list = [[(1,1), (1,2), (2, 3), (2, 1)], [(2.3, 2), (2.4, 3), (3, 3), (3, 2)]]

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
