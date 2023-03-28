import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import math
from colreg_interfaces.msg import ShipData
import logging


class TcpaDcpa(Node):
    def __init__(self):
        super().__init__('tcpa_dcpa_calc')

        self.trg_pose = Pose()
        self.own_pose = Pose()
        self.own_lin_vel = 0.0
        self.own_vel_x = 0.0
        self.own_vel_y = 0.0
        self.trg_lin_vel = 0.0
        self.trg_vel_x = 0.0
        self.trg_vel_y = 0.0
        self.tcpa = 0.0
        self.dcpa = 0.0
        self.collision_point_x = 0.0
        self.collision_point_y = 0.0
        self.run_once = True

        self.own_ship_pose_subscriber_ = self.create_subscription(Pose, "/own_ship/pose", self.own_pose_callback, 10)

        self.subscribers = []
        self.publishers_ = []
        for i in range(18):
            sub_topic = "/turtle" + str(i+2) +"/pose"
            pub_topic = "ship_data/ship_" + str(i+1)
            self.publishers_.append(self.create_publisher(ShipData, pub_topic, 10))
            self.subscribers.append(self.create_subscription(Pose, sub_topic, self.create_trg_pose_callback(i), 10))


        # self.timer = self.create_timer(0.25, self.calculate_publish)

    def calculate_publish(self, sub_num):
        self.calculate_tcpa_dcpa()
        if self.tcpa > 0.0 and self.dcpa < 1.0:
            self.calculate_collision_point(self.own_vel_x, self.own_vel_y, self.own_pose.x, self.own_pose.y, self.tcpa)

        self.publish_ship_data(self.tcpa, self.dcpa, self.collision_point_x, self.collision_point_y, self.trg_pose.x,
                               self.trg_pose.y,
                               self.own_pose.x, self.own_pose.y, self.trg_pose.theta, self.own_pose.theta, sub_num)


    def create_trg_pose_callback(self, idx):
        return lambda m: self.trg_pose_callback(m, idx)
    def trg_pose_callback(self, data, arg):

        print(arg)
        self.trg_pose.x = data.x
        self.trg_pose.y = data.y
        self.trg_pose.theta = data.theta
        self.trg_lin_vel = data.linear_velocity
        self.trg_vel_x = self.calculate_velocity_x(self.trg_lin_vel, self.trg_pose.theta)
        self.trg_vel_y = self.calculate_velocity_y(self.trg_lin_vel, self.trg_pose.theta)


    def own_pose_callback(self, data):

        self.own_pose.x = data.x
        self.own_pose.y = data.y
        self.own_pose.theta = data.theta
        self.own_lin_vel = data.linear_velocity
        self.own_vel_x = self.calculate_velocity_x(self.own_lin_vel, self.own_pose.theta)
        self.own_vel_y = self.calculate_velocity_y(self.own_lin_vel, self.own_pose.theta)


    def calculate_tcpa_dcpa(self):
        x_t = self.trg_pose.x
        y_t = self.trg_pose.y
        x_o = self.own_pose.x
        y_o = self.own_pose.y
        x_t_vel = self.trg_vel_x
        y_t_vel = self.trg_vel_y
        x_o_vel = self.own_vel_x
        y_o_vel = self.own_vel_y

        # dcpa - udaljenost izmedu own broda i tocke u kojoj bi brodovi trebali biti najblizi jedan drugom temeljeno na trenutnim brzinama i smjerovima kretanja
        # tcpa - vrijeme potrebno da se dode do tocke u kojoj bi brodovi trebali biti najblizi jedan drugom temeljeno na trenutnim brzinama i smjerovima kretanja

        if (x_t_vel != x_o_vel or y_t_vel != y_o_vel):
            self.tcpa = -((y_t - y_o) * (y_t_vel - y_o_vel) + (x_t - x_o) * (x_t_vel - x_o_vel)) / (
                    math.pow((y_t_vel - y_o_vel), 2) + math.pow((x_t_vel - x_o_vel), 2))
            self.dcpa = math.sqrt(math.pow(((y_t - y_o) + (y_t_vel - y_o_vel) * self.tcpa), 2) + math.pow(
                ((x_t - x_o) + (x_t_vel - x_o_vel) * self.tcpa), 2))

    def publish_ship_data(self, tcpa, dcpa, collision_point_x, collision_point_y, x_t, y_t, x_o, y_o, theta_trg,
                          theta_own, sub_num):
        msg = ShipData()
        msg.tcpa = tcpa
        msg.dcpa = dcpa
        msg.collision_point_x = collision_point_x
        msg.collision_point_y = collision_point_y
        msg.x_target = x_t
        msg.y_target = y_t
        msg.x_own = x_o
        msg.y_own = y_o
        msg.theta_target = theta_trg
        msg.theta_own = theta_own
        msg.header.stamp = self.get_clock().now().to_msg()

        self.publishers_[sub_num].publish(msg)
        # self.ship_data_publisher_.publish(msg)

    def calculate_collision_point(self, v_0_x, v_0_y, x_o, y_o, tcpa):
        self.collision_point_x = tcpa * v_0_x + x_o
        self.collision_point_y = tcpa * v_0_y + y_o


    def calculate_velocity_x(self, velocity, theta):
        return velocity * math.cos(theta)

    def calculate_velocity_y(self, velocity, theta):
        return velocity * math.sin(theta)

def main(args=None):
    rclpy.init(args=args)
    node = TcpaDcpa()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
