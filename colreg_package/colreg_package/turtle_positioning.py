import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import Spawn
from std_srvs.srv import Empty
import time
import math


class TurtlePositioning(Node):

    def __init__(self):
        super().__init__('turtle_positioning')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('x_tel', 0.0),
                ('y_tel', 0.0),
                ('theta_tel', 0.0),
                ('x_spawn', 0.0),
                ('y_spawn', 0.0),
                ('theta_spawn', 0.0)
            ])

        x_tel = self.get_parameter('x_tel')
        y_tel = self.get_parameter('y_tel')
        theta_tel = self.get_parameter('theta_tel')

        # x_spawn = self.get_parameter('x_spawn')
        # y_spawn = self.get_parameter('y_spawn')
        # theta_spawn = self.get_parameter('theta_spawn')
        self.call_service_teleport(x_tel.value, y_tel.value, theta_tel.value)
        time.sleep(1)
        positions = self.generate_circle_points()
        for i in range(len(positions)):
            # self.call_service_spawn(x_spawn.value, y_spawn.value, theta_spawn.value)
            self.call_service_spawn(positions[i][0], positions[i][1], positions[i][2])
            time.sleep(0.05)
        self.call_service_clear()

    def call_service_teleport(self, x, y, theta):
        client = self.create_client(TeleportAbsolute, "/turtle1/teleport_absolute")
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta
        future = client.call_async(request)

        def callback_call_service(self, future):
            try:
                response = future.result()
            except Exception as e:
                self.get_logger().error("Call failed")

    def call_service_spawn(self, x, y, theta):
        client = self.create_client(Spawn, "/spawn")
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        future = client.call_async(request)

        def callback_call_service(self, future):
            try:
                response = future.result()
            except Exception as e:
                self.get_logger().error("Call failed")

    def call_service_clear(self):
        client = self.create_client(Empty, "/clear")
        request = Empty.Request()
        future = client.call_async(request)

        def callback_call_service(self, future):
            try:
                response = future.result()
            except Exception as e:
                self.get_logger().error("Call failed")

    def generate_circle_points(self):

        a = 5.5
        b = 5.5
        r = 5.0

        stepSize = 0.4

        positions = []
        t = 0
        while t < 2 * math.pi:
            x = r * math.cos(t) + a
            y = r * math.sin(t) + b
            dx, dy = 5.5 - x, 5.5 - y
            angle = math.atan2(dy, dx)
            positions.append((x, y, angle))
            t += stepSize

        return positions


def main(args=None):
    rclpy.init(args=args)
    node = TurtlePositioning()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
