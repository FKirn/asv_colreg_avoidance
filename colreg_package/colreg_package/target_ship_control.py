import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class TargetShipControl(Node):

    def __init__(self):
        super().__init__('target_ship_controller')

        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle2/pose", self.send_vel_command, 10)

    def send_vel_command(self, data):
        for i in range(1):
            vel_pub_topic = "/turtle" + str(i + 2) + "/cmd_vel"

            cmd = Twist()
            cmd.linear.x = 0.3
            self.create_publisher(Twist, vel_pub_topic, 10).publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TargetShipControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
