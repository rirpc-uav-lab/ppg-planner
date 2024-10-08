# Implementing an algorithm from https://alienryderflex.com/polygon/

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PolygonStamped, Point32
from ppg_planner_interfaces.msg import Poly
from ppg_planner_interfaces.srv import PolyGrid


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



class TestClient(Node):

    def __init__(self):
        super().__init__('test_client_ppg')
        self.polygrid_client = self.create_client(PolyGrid, "ppg_service")
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        step = 0.15
        poly = [(0,0), (0, 1), (3,7), (4,5), (6, 0.4)]
        exclude_poly_list = [[(1,1), (1,2), (2, 3), (2, 1)], [(2.3, 2), (2.4, 3), (3, 3), (3, 2)]]

        print("generatinig request")

        request = PolyGrid.Request()
        request.step = step
        for point_raw in poly:
            point = Point32()
            point.x = float(point_raw[0])
            point.y = float(point_raw[1])
            point.z = float(0)
            request.zone.poly.append(point)

        for ex_zone in exclude_poly_list:
            ex_zone_cooked = Poly()
            for point_raw in ex_zone:
                point = Point32()
                point.x = float(point_raw[0])
                point.y = float(point_raw[1])
                point.z = float(0)
                ex_zone_cooked.poly.append(point)
            request.exclusion_zones.append(ex_zone_cooked)

        print("calling service")

        self.future = self.polygrid_client.call_async(request)
        
        # print("spinning until future complete")
        # rclpy.spin_until_future_complete(self, self.future)
        print("returning result")
        # return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = TestClient()

    minimal_publisher.timer_callback()

    # rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
