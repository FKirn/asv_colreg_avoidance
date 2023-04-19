import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class OwnShipControl(Node):

    def __init__(self):
        super().__init__('own_ship_controller')

        self.vel_publisher_ = self.create_publisher(Twist, "own_ship/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "own_ship/pose", self.send_vel_command, 10)

    def send_vel_command(self, pose: Pose):
        cmd = Twist()
        cmd.linear.x = 0.09
        self.vel_publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = OwnShipControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
