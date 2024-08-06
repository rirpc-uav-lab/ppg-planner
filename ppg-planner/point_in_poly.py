# Implementing an algorithm from https://alienryderflex.com/polygon/

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


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
        if point[1] > poly[i][1] and point[1] < poly[i + 1][1]:
            crossed_sides.append((poly[i], poly[i + 1]))
        elif point[1] < poly[i][1] and point[1] > poly[i + 1][1]:
            crossed_sides.append((poly[i], poly[i + 1]))

    j = len(poly) - 1

    if point[1] > poly[0][1] and point[1] < poly[j][1]:
        crossed_sides.append((poly[0], poly[j]))
    elif point[1] < poly[0][1] and point[1] > poly[j][1]:
        crossed_sides.append((poly[0], poly[j]))

    crossing_points = []

    for side in crossed_sides:
        x1, y1 = side[0]
        x2, y2 = side[1]

        if y1 == y2:
            continue

        x = x1 + (point[1] - y1) * (x2 - x1) / (y2 - y1)
        y = point[1]

        crossing_points.append((x, y))

    left_c, right_c = 0, 0

    for cpoint in crossing_points:
        xcp, ycp = cpoint
        x, y = point

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
        self.publisher_ = self.create_publisher(Marker, 'points_in_poly', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):

        poly = [(0,0), (1,0), (1,1), (0,1)]
        point = (0.5, 0.5)

        print(f'{point_in_poly(point=point, poly=poly)}')

        msg = Marker()

        self.publisher_.publish(msg)

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
