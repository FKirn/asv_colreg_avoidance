import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import math
from colreg_interfaces.msg import ShipData
from math import degrees
import numpy as np

class TcpaDcpa(Node):
    def __init__(self):
        super().__init__('tcpa_dcpa_calc')

        self.own_pose = Pose()
        self.own_lin_vel = 0.0
        self.own_vel_x = 0.0
        self.own_vel_y = 0.0
        self.own_pose_x = 0.0
        self.own_pose_y = 0.0
        self.own_pose_theta = 0.0
        self.own_ship_pose_subscriber_ = self.create_subscription(Pose, "/own_ship/pose", self.own_pose_callback, 10)

        self.trgShipsData = {}
        self.subscribers = []
        self.publishers_ = []
        for i in range(1):
            sub_topic = "/turtle" + str(i + 2) + "/pose"
            pub_topic = "ship_data/ship_" + str(i + 1)
            self.publishers_.append(self.create_publisher(ShipData, pub_topic, 10))
            self.subscribers.append(self.create_subscription(Pose, sub_topic, self.create_trg_pose_callback(i), 10))

    def own_pose_callback(self, data):

        self.own_pose_x = data.x
        self.own_pose_y = data.y
        self.own_pose_theta = data.theta
        self.own_lin_vel = data.linear_velocity
        self.own_vel_x = self.calculate_velocity_x(self.own_lin_vel, self.own_pose.theta)
        self.own_vel_y = self.calculate_velocity_y(self.own_lin_vel, self.own_pose.theta)

    def create_trg_pose_callback(self, idx):
        return lambda m: self.trg_pose_callback(m, idx)

    def trg_pose_callback(self, data, index):

        trg_ship_name = "trg_ship_" + str(index)

        if trg_ship_name not in self.trgShipsData:
            self.trgShipsData.setdefault(trg_ship_name, {}).update(self.createShipDataDict(data, index))
        else:
            self.updateShipDataDict(trg_ship_name, data)

        self.calculate_publish(trg_ship_name)

    def createShipDataDict(self, data, index):
        trg_pose_x = data.x
        trg_pose_y = data.y
        trg_theta = data.theta
        trg_lin_vel = data.linear_velocity
        trg_vel_x = self.calculate_velocity_x(trg_lin_vel, trg_theta)
        trg_vel_y = self.calculate_velocity_y(trg_lin_vel, trg_theta)

        return {'index': index,'trg_pose_x': trg_pose_x, 'trg_pose_y': trg_pose_y, 'trg_theta': trg_theta,
                'trg_lin_vel': trg_lin_vel, 'trg_vel_x': trg_vel_x, 'trg_vel_y': trg_vel_y}

    def updateShipDataDict(self, trg_ship_name, data):
        self.trgShipsData[trg_ship_name]['trg_pose_x'] = data.x
        self.trgShipsData[trg_ship_name]['trg_pose_y'] = data.y
        self.trgShipsData[trg_ship_name]['trg_theta'] = data.theta
        self.trgShipsData[trg_ship_name]['trg_lin_vel'] = data.linear_velocity
        self.trgShipsData[trg_ship_name]['trg_vel_x'] = self.calculate_velocity_x(data.linear_velocity, data.theta)
        self.trgShipsData[trg_ship_name]['trg_vel_y'] = self.calculate_velocity_y(data.linear_velocity, data.theta)

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
        x_t_vel = self.trgShipsData[trg_ship_name]['trg_vel_x']
        y_t_vel = self.trgShipsData[trg_ship_name]['trg_vel_y']

        x_o = self.own_pose_x
        y_o = self.own_pose_y
        x_o_vel = self.own_vel_x
        y_o_vel = self.own_vel_y
        # dcpa - udaljenost izmedu own broda i tocke u kojoj bi brodovi trebali biti najblizi jedan drugom temeljeno na trenutnim brzinama i smjerovima kretanja
        # tcpa - vrijeme potrebno da se dode do tocke u kojoj bi brodovi trebali biti najblizi jedan drugom temeljeno na trenutnim brzinama i smjerovima kretanja
        self.trgShipsData[trg_ship_name]['tcpa'] = 0.0
        self.trgShipsData[trg_ship_name]['dcpa'] = 0.0

        own_vessel_course = self.own_pose_theta
        other_vessel_course = self.trgShipsData[trg_ship_name]['trg_theta']



        ###############################################

        # pos_a = np.array([x_o, y_o])
        # pos_b = np.array([x_t, y_t])
        # vel_a = np.array([x_o_vel, y_o_vel])
        # vel_b = np.array([x_t_vel, y_t_vel])
        #
        # # Calculate relative position and velocity of vessel B with respect to vessel A
        # rel_pos = pos_b - pos_a
        # rel_vel = vel_b - vel_a
        #
        # dcpa = np.linalg.norm(rel_pos - np.dot(rel_pos, rel_vel) * rel_vel / np.linalg.norm(rel_vel))
        # tcpa = -np.dot(rel_pos, rel_vel) / np.linalg.norm(rel_vel) ** 2
        ################################################
        # calculate relative positions and velocities
        # rel_x = x_t - x_o
        # rel_y = y_t - y_o
        # rel_speed_x = self.trgShipsData[trg_ship_name]['trg_lin_vel'] * math.sin(other_vessel_course) - self.own_lin_vel * math.sin(
        #     own_vessel_course)
        # rel_speed_y = self.trgShipsData[trg_ship_name]['trg_lin_vel'] * math.cos(other_vessel_course) - self.own_lin_vel * math.cos(
        #     own_vessel_course)
        #
        # # calculate the quadratic equation coefficients
        # a = rel_speed_x ** 2 + rel_speed_y ** 2 - 0.25 * (self.own_lin_vel + self.trgShipsData[trg_ship_name]['trg_lin_vel']) ** 2
        # b = 2 * (rel_x * rel_speed_x + rel_y * rel_speed_y)
        # c = rel_x ** 2 + rel_y ** 2
        #
        # # calculate the time to CPA (tcpa) and the distance to CPA (dcpa)
        # if a == 0:
        #     tcpa = -c / b
        # else:
        #     disc = b ** 2 - 4 * a * c
        #     if disc < 0:
        #         tcpa = math.inf
        #     else:
        #         root = math.sqrt(disc)
        #         tcpa1 = (-b - root) / (2 * a)
        #         tcpa2 = (-b + root) / (2 * a)
        #         if tcpa1 < 0:
        #             tcpa = tcpa2
        #         elif tcpa2 < 0:
        #             tcpa = tcpa1
        #         else:
        #             tcpa = min(tcpa1, tcpa2)
        #
        # dcpa = math.sqrt((rel_x - 0.5 * self.own_lin_vel * tcpa * rel_speed_x) ** 2 + (
        #             rel_y - 0.5 * self.own_lin_vel * tcpa * rel_speed_y) ** 2)



        #########################################################

        # ship1_pos = (x_o,y_o)
        # ship2_pos = (x_t,y_t)
        #
        # ship1_vel = (x_o_vel, y_o_vel)
        # ship2_vel = (x_t_vel, y_t_vel)
        # ship1_speed = self.own_lin_vel
        # ship2_speed = self.trgShipsData[trg_ship_name]['trg_lin_vel']
        #
        # # Convert courses to radians
        # ship1_course_rad = math.radians(ship1_course)
        # ship2_course_rad = math.radians(ship2_course)
        #
        # # Calculate the ships' velocities in the x and y directions
        # ship1_vel = [ship1_speed * math.cos(ship1_course_rad), ship1_speed * math.sin(ship1_course_rad)]
        # ship2_vel = [ship2_speed * math.cos(ship2_course_rad), ship2_speed * math.sin(ship2_course_rad)]
        #
        # # Calculate the relative velocity of ship 2 with respect to ship 1
        # ship_rel_vel = [ship2_vel[0] - ship1_vel[0], ship2_vel[1] - ship1_vel[1]]
        #
        # # Calculate the distance between the two ships
        # dist = math.sqrt((ship2_pos[0] - ship1_pos[0]) ** 2 + (ship2_pos[1] - ship1_pos[1]) ** 2)
        #
        # # Calculate the time to closest point of approach (TCPA)
        # tcpa = -((ship2_pos[0] - ship1_pos[0]) * ship_rel_vel[0] + (ship2_pos[1] - ship1_pos[1]) * ship_rel_vel[1]) / (
        #             dist ** 2 - (ship_rel_vel[0] ** 2 + ship_rel_vel[1] ** 2))
        #
        # # Calculate the distance to closest point of approach (DCPA)
        # dcpa = math.sqrt((ship1_pos[0] + ship_rel_vel[0] * tcpa - ship2_pos[0]) ** 2 + (
        #             ship1_pos[1] + ship_rel_vel[1] * tcpa - ship2_pos[1]) ** 2)

        #########################################################


        if (x_t_vel != x_o_vel or y_t_vel != y_o_vel):
            tcpa = -((y_t - y_o) * (y_t_vel - y_o_vel) + (x_t - x_o) * (x_t_vel - x_o_vel)) / (
                    math.pow((y_t_vel - y_o_vel), 2) + math.pow((x_t_vel - x_o_vel), 2))
            dcpa = math.sqrt(math.pow(((y_t - y_o) + (y_t_vel - y_o_vel) * tcpa), 2) + math.pow(
                ((x_t - x_o) + (x_t_vel - x_o_vel) * self.trgShipsData[trg_ship_name]['tcpa']), 2))

        self.trgShipsData[trg_ship_name]['tcpa'] = tcpa
        self.trgShipsData[trg_ship_name]['dcpa'] = dcpa

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
        msg.theta_own = self.own_pose_theta
        msg.header.stamp = self.get_clock().now().to_msg()

        self.publishers_[self.trgShipsData[trg_ship_name]['index']].publish(msg)

    def determine_collision_risk(self, trg_ship_name):

        if self.trgShipsData[trg_ship_name]['tcpa'] > 0.0 and self.trgShipsData[trg_ship_name]['dcpa'] <= 2.0:
            print("Danger. Extreme risk of collision!")
            return True
        elif self.trgShipsData[trg_ship_name]['tcpa'] > 5.0 and self.trgShipsData[trg_ship_name]['dcpa'] < 5.0:
            print("Danger. Ships on a possible collision course!")
            return False
        else:
            self.trgShipsData[trg_ship_name]['scenario'] = "NO COLLISION"
            print("Not on a collision course.")
            return False

    def calculate_collision_point(self, trg_ship_name):
        self.trgShipsData[trg_ship_name]['collision_point_x'] = self.trgShipsData[trg_ship_name]['tcpa'] * self.own_pose_x
        self.trgShipsData[trg_ship_name]['collision_point_y'] = self.trgShipsData[trg_ship_name]['tcpa'] * self.own_pose_y

    def determine_avoidance_scenario(self, trg_ship_name):

        course_trg_ship = degrees(self.trgShipsData[trg_ship_name]['trg_theta']) % 360
        if course_trg_ship < 0:
            course_trg_ship += 360
        course_own_ship = degrees(self.own_pose_theta) % 360
        if course_own_ship < 0:
            course_own_ship += 360

        # Convert ship angles from radians to degrees
        own_ship_speed = abs(self.own_lin_vel)
        trg_ship_speed = abs(self.trgShipsData[trg_ship_name]['trg_lin_vel'])
        # Calculate relative position and speed of the two ships
        rel_pos = (self.own_pose.x - self.trgShipsData[trg_ship_name]['trg_pose_x'], self.own_pose.y - self.trgShipsData[trg_ship_name]['trg_pose_y'])

        print("trg: " + str(course_trg_ship))
        print("own: " + str(course_own_ship))

        angle_between_vessels = abs(course_trg_ship - course_own_ship)
        print("trg: " + str(course_trg_ship))
        print("own: " + str(course_own_ship))
        print("angle: " + str(angle_between_vessels))

        if angle_between_vessels <= 185 and angle_between_vessels >= 175:
            scenario = "Head-on situation"
        elif angle_between_vessels > 10 and angle_between_vessels < 175:
            if course_own_ship > course_trg_ship:
                scenario = "CROSSING PORT TO STARBOARD"
            else:
                scenario = "CROSSING STARBOARD TO PORT"
        elif angle_between_vessels >= 0 and angle_between_vessels <= 10:
            if own_ship_speed > trg_ship_speed:
                scenario = "OVERTAKING"
            elif own_ship_speed < trg_ship_speed:
                scenario = "BEING OVERTAKEN"
        else:
            scenario = "NO COLLISION"

        self.trgShipsData[trg_ship_name]['scenario'] = scenario

        print(scenario)

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
