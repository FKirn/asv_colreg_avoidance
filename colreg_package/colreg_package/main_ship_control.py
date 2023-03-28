import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class MainShipController(Node):

    def __init__(self):
        super().__init__('main_ship_controller')

        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle2/pose", self.send_vel_command, 10)

    def send_vel_command(self, data):
        for i in range(18):
            vel_pub_topic = "/turtle" + str(i + 2) + "/cmd_vel"

            cmd = Twist()
            cmd.linear.x = 0.3
            self.create_publisher(Twist, vel_pub_topic, 10).publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MainShipController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
