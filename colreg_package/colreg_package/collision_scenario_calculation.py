import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from colreg_interfaces.msg import ShipData
from math import degrees
import numpy as np


def calculate_theta_from_quaternion(quaternion):

    siny_cosp = +2.0 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3])
    cosy_cosp = +1.0 - 2.0 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2])
    theta = math.atan2(siny_cosp, cosy_cosp)

    theta = (-np.degrees(theta))


    #transfer to turtlesim angles
    if theta == 0:
        theta = theta + 90
    elif theta == -180 or theta == 180:
        theta = 270
    elif theta == 90:
        theta = 0
    elif (0>theta> -180):
        theta = -theta + 90
    elif (90<theta< 180):
        theta = 450 - theta
    elif (0<theta<90):
        theta = 90 - theta
    return theta


class AvoidanceScenario(Node):
    def __init__(self):
        super().__init__('tcpa_dcpa_calculation')

        self.own_lin_vel = 0.0
        self.own_vel_x = 0.0
        self.own_vel_y = 0.0
        self.own_pose_x = 0.0
        self.own_pose_y = 0.0
        self.own_theta = 0.0
        self.trgShipsData = {}
        self.subscribers_pos = []
        self.subscribers_imu = []
        self.publishers_ = []

        self.previous_own_x_pos = None
        self.previous_own_y_pos = None
        self.previous_own_time = None

        own_ship_pos_topic = "/marus_boat/pos"
        own_ship_imu_topic = "/marus_boat/imu"
        self.own_ship_pos_subscriber_ = self.create_subscription(PoseWithCovarianceStamped, own_ship_pos_topic, self.own_pos_callback, 10)
        self.own_ship_imu_subscriber_ = self.create_subscription(Imu, own_ship_imu_topic,self.own_imu_callback, 10)

        for i in range(1):
            # sub_topic = "/marus_boat" + str(i + 0) + "/pos"
            sub_pos_topic = "/marus_boat1/pos"
            sub_imu_topic = "/marus_boat1/imu"
            # pub_topic = "ship_data/ship_" + str(i + 1)
            pub_pos_topic = "ship_data/ship_1"
            self.publishers_.append(self.create_publisher(ShipData, pub_pos_topic, 10))
            self.subscribers_pos.append(self.create_subscription(PoseWithCovarianceStamped, sub_pos_topic, self.create_trg_pose_callback(i), 10))
            self.subscribers_imu.append(self.create_subscription(Imu, sub_imu_topic, self.create_trg_imu_callback(i),10))

    def own_pos_callback(self, data):

        current_time = data.header.stamp.sec + data.header.stamp.nanosec * 1e-9

        linear_velocity_x = 0.0
        linear_velocity_y = 0.0
        if self.previous_own_x_pos is not None and self.previous_own_y_pos is not None and self.previous_own_time is not None:
            time_delta = current_time - self.previous_own_time

            delta_x = data.pose.pose.position.x - self.previous_own_x_pos
            delta_y = data.pose.pose.position.y - self.previous_own_y_pos

            linear_velocity_x = delta_x / time_delta
            linear_velocity_y = delta_y / time_delta

            # self.get_logger().info("...........................")
            # self.get_logger().info("pose x: " + str(data.pose.pose.position.x))
            # self.get_logger().info("pose y: " + str(data.pose.pose.position.y))
            # self.get_logger().info("delta_x: " + str(delta_x))
            # self.get_logger().info("delta_y: " + str(delta_y))
            # self.get_logger().info("timedelta: " + str(time_delta))

        self.own_pose_x = data.pose.pose.position.x
        self.own_pose_y = data.pose.pose.position.y
        self.own_vel_x = linear_velocity_x
        self.own_vel_y = linear_velocity_y
        self.own_lin_vel = math.sqrt(linear_velocity_x ** 2 + linear_velocity_y ** 2)
        self.previous_own_x_pos = data.pose.pose.position.x
        self.previous_own_y_pos = data.pose.pose.position.y
        self.previous_own_time = current_time

        # self.get_logger().info("own_lin_vel: " + str(self.own_lin_vel))
        # self.get_logger().info("linvel_x: " + str(linear_velocity_x))
        # self.get_logger().info("linvel_y: " +str(linear_velocity_y))
        # self.get_logger().info("...........................")

    def own_imu_callback(self, data):

        quaternion = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        self.own_theta = calculate_theta_from_quaternion(quaternion)



    def create_trg_pose_callback(self, idx):
        return lambda m: self.trg_pose_callback(m, idx)

    def create_trg_imu_callback(self, idx):
        return lambda m: self.trg_imu_callback(m, idx)

    def trg_pose_callback(self, data, index):

        trg_ship_name = "trg_ship_" + str(index)

        if trg_ship_name not in self.trgShipsData:
            self.trgShipsData.setdefault(trg_ship_name, {}).update(self.createShipDataDict(index))
        else:
            self.updateShipDataDict(trg_ship_name, data)

        self.calculate_publish(trg_ship_name)

    def trg_imu_callback(self, data, index):

        trg_ship_name = "trg_ship_" + str(index)
        if trg_ship_name not in self.trgShipsData:
            self.trgShipsData.setdefault(trg_ship_name, {}).update(self.createShipDataDict(index))

        quaternion = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        self.trgShipsData[trg_ship_name]['trg_theta'] = calculate_theta_from_quaternion(quaternion)
        self.get_logger().info("theta_trg: " + str(self.trgShipsData[trg_ship_name]['trg_theta']))

    def createShipDataDict(self, index):

        return {'index': index, 'trg_pose_x': 0.0, 'trg_pose_y': 0.0, 'trg_theta': 0.0,
                'trg_vel_x': 0.0, 'trg_vel_y': 0.0, 'previous_trg_x_pos': None, 'previous_trg_y_pos': None,
                'previous_trg_time': None, 'trg_lin_vel': None}

    def updateShipDataDict(self, trg_ship_name, data):

        current_time = data.header.stamp.sec + data.header.stamp.nanosec * 1e-9

        if self.trgShipsData[trg_ship_name]['previous_trg_x_pos'] is not None and self.trgShipsData[trg_ship_name]['previous_trg_y_pos'] is not None and self.trgShipsData[trg_ship_name]['previous_trg_time'] is not None:

            time_delta = current_time - self.trgShipsData[trg_ship_name]['previous_trg_time']
            delta_x = data.pose.pose.position.x - self.trgShipsData[trg_ship_name]['previous_trg_x_pos']
            delta_y = data.pose.pose.position.y - self.trgShipsData[trg_ship_name]['previous_trg_y_pos']
            linear_velocity_x = delta_x / time_delta
            linear_velocity_y = delta_y / time_delta

        else:
            linear_velocity_x = 0.0
            linear_velocity_y = 0.0

        self.trgShipsData[trg_ship_name]['trg_pose_x'] = data.pose.pose.position.x
        self.trgShipsData[trg_ship_name]['trg_pose_y'] = data.pose.pose.position.y
        self.trgShipsData[trg_ship_name]['trg_vel_x'] = linear_velocity_x
        self.trgShipsData[trg_ship_name]['trg_vel_y'] = linear_velocity_y
        self.trgShipsData[trg_ship_name]['trg_lin_vel'] = math.sqrt(linear_velocity_x ** 2 + linear_velocity_y ** 2)
        self.trgShipsData[trg_ship_name]['previous_trg_x_pos'] = data.pose.pose.position.x
        self.trgShipsData[trg_ship_name]['previous_trg_y_pos'] = data.pose.pose.position.y
        self.trgShipsData[trg_ship_name]['previous_trg_time'] = current_time

    def calculate_publish(self, trg_ship_name):

        self.calculate_tcpa_dcpa(trg_ship_name)

        if self.determine_collision_risk(trg_ship_name):
            self.calculate_collision_point(trg_ship_name)
            self.determine_avoidance_scenario(trg_ship_name)

        else:
            self.trgShipsData[trg_ship_name]['scenario'] = "NO COLLISION"
            self.trgShipsData[trg_ship_name]['collision_point_x'] = 0.0
            self.trgShipsData[trg_ship_name]['collision_point_y'] = 0.0

        self.publish_ship_data(trg_ship_name)

    def calculate_tcpa_dcpa(self, trg_ship_name):
        x_t = self.trgShipsData[trg_ship_name]['trg_pose_x']
        y_t = self.trgShipsData[trg_ship_name]['trg_pose_y']
        x_o = self.own_pose_x
        y_o = self.own_pose_y

        # dcpa - udaljenost izmedu own broda i tocke u kojoj bi brodovi trebali biti najblizi jedan drugom temeljeno na trenutnim brzinama i smjerovima kretanja
        # tcpa - vrijeme potrebno da se dode do tocke u kojoj bi brodovi trebali biti najblizi jedan drugom temeljeno na trenutnim brzinama i smjerovima kretanja
        tcpa = float('inf')
        dcpa = float('inf')

        x_o_vel = self.own_vel_x
        y_o_vel = self.own_vel_y
        x_t_vel = self.trgShipsData[trg_ship_name]['trg_vel_x']
        y_t_vel = self.trgShipsData[trg_ship_name]['trg_vel_y']

        if (x_t_vel != x_o_vel or y_t_vel != y_o_vel):
            tcpa = -((y_t - y_o) * (y_t_vel - y_o_vel) + (x_t - x_o) * (x_t_vel - x_o_vel)) / (
                    math.pow((y_t_vel - y_o_vel), 2) + math.pow((x_t_vel - x_o_vel), 2))
            dcpa = math.sqrt(math.pow(((y_t - y_o) + (y_t_vel - y_o_vel) * tcpa), 2) + math.pow(
                ((x_t - x_o) + (x_t_vel - x_o_vel) * tcpa), 2))

        self.trgShipsData[trg_ship_name]['tcpa'] = tcpa
        self.trgShipsData[trg_ship_name]['dcpa'] = dcpa

    def determine_collision_risk(self, trg_ship_name):

        if self.trgShipsData[trg_ship_name]['tcpa'] > 0.0 and self.trgShipsData[trg_ship_name]['dcpa'] <= 1.2:
            print("Danger! Extreme risk of collision!")
            return True
        elif self.trgShipsData[trg_ship_name]['tcpa'] > 0.0 and self.trgShipsData[trg_ship_name]['dcpa'] < 4.0:
            print("Danger! Ships on a possible collision course!")
            return False
        else:
            self.trgShipsData[trg_ship_name]['scenario'] = "NO COLLISION"
            print("Not on a collision course.")
            return False

    def calculate_collision_point(self, trg_ship_name):
        self.trgShipsData[trg_ship_name]['collision_point_x'] = self.trgShipsData[trg_ship_name][
                                                                    'tcpa'] * self.own_pose_x
        self.trgShipsData[trg_ship_name]['collision_point_y'] = self.trgShipsData[trg_ship_name][
                                                                    'tcpa'] * self.own_pose_y

    def determine_avoidance_scenario(self, trg_ship_name):

        course_trg_ship = degrees(self.trgShipsData[trg_ship_name]['trg_theta']) % 360
        if course_trg_ship < 0:
            course_trg_ship += 360
        course_own_ship = degrees(self.own_pose_theta) % 360
        if course_own_ship < 0:
            course_own_ship += 360

        own_ship_speed = abs(self.own_lin_vel)
        trg_ship_speed = abs(self.trgShipsData[trg_ship_name]['trg_lin_vel'])
        angle_between_vessels = abs(course_trg_ship - course_own_ship)
        relative_course = (course_trg_ship - course_own_ship) % 360

        if 185 >= angle_between_vessels >= 175:
            scenario = "HEAD ON"
        elif 10 < angle_between_vessels < 175 or 185 < angle_between_vessels < 360:

            if course_trg_ship > course_own_ship:
                if 10 < relative_course < 175:
                    scenario = "CROSSING STARBOARD TO PORT"
                elif 185 < relative_course < 360:
                    scenario = "CROSSING PORT TO STARBOARD"
            elif course_own_ship > course_trg_ship:
                if 10 < relative_course < 175:
                    scenario = "CROSSING STARBOARD TO PORT"
                elif 185 < relative_course < 360:
                    scenario = "CROSSING PORT TO STARBOARD"
        elif 0 <= angle_between_vessels <= 10:
            if own_ship_speed > trg_ship_speed:
                scenario = "OVERTAKING"
            elif own_ship_speed < trg_ship_speed:
                scenario = "BEING OVERTAKEN"
        else:
            scenario = "NO COLLISION"

        self.trgShipsData[trg_ship_name]['scenario'] = scenario

        print(scenario)

    def publish_ship_data(self, trg_ship_name):

        msg = ShipData()
        msg.situation = self.trgShipsData[trg_ship_name]['scenario']
        msg.tcpa = self.trgShipsData[trg_ship_name]['tcpa']
        msg.dcpa = self.trgShipsData[trg_ship_name]['dcpa']
        msg.collision_point_x = self.trgShipsData[trg_ship_name]['collision_point_x']
        msg.collision_point_y = self.trgShipsData[trg_ship_name]['collision_point_y']
        msg.x_target = self.trgShipsData[trg_ship_name]['trg_pose_x']
        msg.y_target = self.trgShipsData[trg_ship_name]['trg_pose_y']
        msg.x_own = self.own_pose_x
        msg.y_own = self.own_pose_y
        msg.theta_target = self.trgShipsData[trg_ship_name]['trg_theta']
        msg.theta_own = self.own_theta
        msg.header.stamp = self.get_clock().now().to_msg()

        self.publishers_[self.trgShipsData[trg_ship_name]['index']].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceScenario()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
