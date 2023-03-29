import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import math
from colreg_interfaces.msg import ShipData
from colreg_interfaces.msg import AvoidanceScenario
import logging
from math import degrees

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
            print("Rizik od sudara")
            self.calculate_collision_point(trg_ship_name, self.own_vel_x, self.own_vel_y, self.own_pose.x, self.own_pose.y, self.trgShipsData[trg_ship_name]['tcpa'])
            self.determine_avoidance_scenario(trg_ship_name)
        else:
            self.trgShipsData[trg_ship_name]['collision_point_x'] = 0.0
            self.trgShipsData[trg_ship_name]['collision_point_y'] = 0.0
        self.publish_ship_data(trg_ship_name, self.trgShipsData[trg_ship_name]['scenario'], self.trgShipsData[trg_ship_name]['tcpa'], self.trgShipsData[trg_ship_name]['dcpa'],
                               self.trgShipsData[trg_ship_name]['collision_point_x'], self.trgShipsData[trg_ship_name]['collision_point_y'],
                               self.trgShipsData[trg_ship_name]['trg_pose_x'], self.trgShipsData[trg_ship_name]['trg_pose_y'],
                               self.own_pose_x, self.own_pose_y, self.trgShipsData[trg_ship_name]['trg_theta'], self.own_pose_theta)

    def calculate_tcpa_dcpa(self, trg_ship_name):
        x_t = self.trgShipsData[trg_ship_name]['trg_pose_x']
        y_t = self.trgShipsData[trg_ship_name]['trg_pose_y']
        x_t_vel = self.trgShipsData[trg_ship_name]['trg_vel_x']
        y_t_vel = self.trgShipsData[trg_ship_name]['trg_vel_y']

        x_o = self.own_pose.x
        y_o = self.own_pose.y
        x_o_vel = self.own_vel_x
        y_o_vel = self.own_vel_y
        # dcpa - udaljenost izmedu own broda i tocke u kojoj bi brodovi trebali biti najblizi jedan drugom temeljeno na trenutnim brzinama i smjerovima kretanja
        # tcpa - vrijeme potrebno da se dode do tocke u kojoj bi brodovi trebali biti najblizi jedan drugom temeljeno na trenutnim brzinama i smjerovima kretanja
        self.trgShipsData[trg_ship_name]['tcpa'] = 0.0
        self.trgShipsData[trg_ship_name]['dcpa'] = 0.0

        if (x_t_vel != x_o_vel or y_t_vel != y_o_vel):
            self.trgShipsData[trg_ship_name]['tcpa'] = -((y_t - y_o) * (y_t_vel - y_o_vel) + (x_t - x_o) * (x_t_vel - x_o_vel)) / (
                    math.pow((y_t_vel - y_o_vel), 2) + math.pow((x_t_vel - x_o_vel), 2))
            self.trgShipsData[trg_ship_name]['dcpa'] = math.sqrt(math.pow(((y_t - y_o) + (y_t_vel - y_o_vel) * self.trgShipsData[trg_ship_name]['tcpa']), 2) + math.pow(
                ((x_t - x_o) + (x_t_vel - x_o_vel) * self.trgShipsData[trg_ship_name]['tcpa']), 2))

        # print(self.trgShipsData)
    def publish_ship_data(self, trg_ship_name, situation, tcpa, dcpa, collision_point_x, collision_point_y, x_t, y_t, x_o, y_o, theta_trg,
                          theta_own):
        msg = ShipData()
        msg.situation = situation
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

        self.publishers_[self.trgShipsData[trg_ship_name]['index']].publish(msg)
        # self.ship_data_publisher_.publish(msg)

    def determine_collision_risk(self, trg_ship_name):

        # if tcpa > 0.0:
        #     logging.warning("Ships are approaching!")
        # elif tcpa < 0.0:
        #     logging.warning("Ships are moving away and are out of danger!")

        if self.trgShipsData[trg_ship_name]['tcpa'] > 0.0 and self.trgShipsData[trg_ship_name]['dcpa'] < 1.0:
            # logging.warning("High risk of collision!!!")
            return True
        else:
            self.trgShipsData[trg_ship_name]['scenario'] = "NO COLLISION"
            return False

    def calculate_collision_point(self,trg_ship_name, v_0_x, v_0_y, x_o, y_o, tcpa):
        self.trgShipsData[trg_ship_name]['collision_point_x'] = tcpa * v_0_x + x_o
        self.trgShipsData[trg_ship_name]['collision_point_y'] = tcpa * v_0_y + y_o

    def determine_avoidance_scenario(self, trg_ship_name):

        msg = AvoidanceScenario()
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
        rel_speed = trg_ship_speed - own_ship_speed

        # Calculate the angle between the relative position and the true north
        rel_angle = math.atan2(rel_pos[1], rel_pos[0])
        rel_angle = math.degrees(rel_angle)

        # Calculate the angle between the true north and the course of each ship
        ship1_angle = (course_own_ship - rel_angle) % 360
        ship2_angle = (course_trg_ship - rel_angle) % 360

        scenario = "NO COLLISION"
        # Determine the collision situation based on the angles and the TCPA and DCPA
        if ship1_angle < 90 or ship1_angle >= 270:
            print("Head-on")
            scenario = "HEAD_ON"
        elif ship1_angle < ship2_angle:
            print("Crossing, port to port")
            scenario = "CROSSING_PORT"
        elif ship1_angle > ship2_angle:
            print("Crossing, starboard to starboard")
            scenario = "CROSSING_STARBOARD"
        elif ship1_angle == ship2_angle:
            if own_ship_speed < trg_ship_speed:
                print("Overtaking")
                scenario = "OVERTAKING"
            elif own_ship_speed > trg_ship_speed:
                print("Being overtaken")
                scenario = "BEING OVERTAKEN"
            else:
                print("Parallel courses")
                scenario = "PARALEL COURSES"
        self.trgShipsData[trg_ship_name]['scenario'] = scenario

        # if angle_between_vessels < 22.5:
        #     return "Head-on situation - both vessels should alter course to starboard"
        # elif angle_between_vessels < 67.5:
        #     return "Crossing situation - give-way vessel should alter course to starboard"
        # elif angle_between_vessels < 112.5:
        #     return "Crossing situation - both vessels should maintain course and speed"
        # elif angle_between_vessels < 157.5:
        #     return "Crossing situation - give-way vessel should alter course to port"
        # else:
        #     return "Head-on situation - both vessels should alter course to port"


        # msg = AvoidanceScenario()
        # ships_dist_x_axis = self.trg_pose.x - self.own_pose.x
        # ships_dist_y_axis = self.trg_pose.y - self.own_pose.y
        # angle_between_ships = math.atan(ships_dist_y_axis / ships_dist_x_axis)
        # angle_between_ships_abs = self.own_pose.theta - self.trg_pose.theta
        #
        # if angle_between_ships < 0:
        #     angle_between_ships = angle_between_ships + math.pi
        #
        # if self.own_pose.theta < 0:
        #     own_ship_angle_degrees = (self.own_pose.theta * 57.295779513) + 360
        #
        # if self.trg_pose.theta < 0:
        #     trg_ship_angle_degrees = (self.trg_pose.theta * 57.295779513) + 360
        #
        #
        #
        # if ((own_ship_angle_degrees <= 95 and own_ship_angle_degrees >= 85 and trg_ship_angle_degrees <= 275 and trg_ship_angle_degrees >= 265)
        #     or (own_ship_angle_degrees <= 275 and own_ship_angle_degrees >= 265 and trg_ship_angle_degrees <= 95 and trg_ship_angle_degrees >= 85)):
        #     msg.scenario = msg.HEAD_ON
        #elif():


        # if (angle_between_ships > 1.745329 and angle_between_ships < 3.490659):
        #     msg.scenario = msg.CROSSING_PORT
        #
        # elif (
        #         angle_between_ships >= 5.934119 and angle_between_ships <= 6.283185 or angle_between_ships >= 0.0 and angle_between_ships < 1.396263):
        #     msg.scenario = msg.CROSSING_STARBOARD
        #
        # elif (angle_between_ships >= 1.396263 and angle_between_ships <= 1.745329):
        #     if (angle_between_ships_abs < 0.349065 and angle_between_ships_abs > -0.349065):
        #         msg.scenario = msg.OVERTAKING
        #     elif (angle_between_ships_abs < 3.490658 and angle_between_ships_abs > 2.792526):
        #         msg.scenario = msg.HEAD_ON
        # else:
        #     msg.scenario = msg.NO_COLLISION


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
