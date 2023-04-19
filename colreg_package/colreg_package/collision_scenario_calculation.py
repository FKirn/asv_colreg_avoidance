import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import math
from colreg_interfaces.msg import ShipData
from math import degrees

class AvoidanceScenario(Node):
    def __init__(self):
        super().__init__('tcpa_dcpa_calculation')

        self.own_pose = Pose()
        self.own_lin_vel = 0.0
        self.own_vel_x = 0.0
        self.own_vel_y = 0.0
        self.own_pose_x = 0.0
        self.own_pose_y = 0.0
        self.own_pose_theta = 0.0
        self.trgShipsData = {}
        self.subscribers = []
        self.publishers_ = []

        self.own_ship_pose_subscriber_ = self.create_subscription(Pose, "/own_ship/pose", self.own_pose_callback, 10)
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

        return {'index': index,'trg_pose_x': trg_pose_x, 'trg_pose_y': trg_pose_y, 'trg_theta': trg_theta,
                'trg_lin_vel': trg_lin_vel}

    def updateShipDataDict(self, trg_ship_name, data):
        self.trgShipsData[trg_ship_name]['trg_pose_x'] = data.x
        self.trgShipsData[trg_ship_name]['trg_pose_y'] = data.y
        self.trgShipsData[trg_ship_name]['trg_theta'] = data.theta
        self.trgShipsData[trg_ship_name]['trg_lin_vel'] = data.linear_velocity

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

        x_o_vel = self.own_lin_vel * math.cos(self.own_pose_theta)
        y_o_vel = self.own_lin_vel * math.sin(self.own_pose_theta)
        x_t_vel = self.trgShipsData[trg_ship_name]['trg_lin_vel'] * math.cos(self.trgShipsData[trg_ship_name]['trg_theta'])
        y_t_vel = self.trgShipsData[trg_ship_name]['trg_lin_vel'] * math.sin(self.trgShipsData[trg_ship_name]['trg_theta'])


        if (x_t_vel != x_o_vel or y_t_vel != y_o_vel):
            tcpa = -((y_t - y_o) * (y_t_vel - y_o_vel) + (x_t - x_o) * (x_t_vel - x_o_vel)) / (
                    math.pow((y_t_vel - y_o_vel), 2) + math.pow((x_t_vel - x_o_vel), 2))
            dcpa = math.sqrt(math.pow(((y_t - y_o) + (y_t_vel - y_o_vel) * tcpa), 2) + math.pow(
                ((x_t - x_o) + (x_t_vel - x_o_vel) * tcpa), 2))

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
        self.trgShipsData[trg_ship_name]['collision_point_x'] = self.trgShipsData[trg_ship_name]['tcpa'] * self.own_pose_x
        self.trgShipsData[trg_ship_name]['collision_point_y'] = self.trgShipsData[trg_ship_name]['tcpa'] * self.own_pose_y

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
        # print(angle_between_vessels)
        # print("trg_ " + str(course_trg_ship))
        # print("own_ " + str(course_own_ship))

        relative_course = (course_trg_ship - course_own_ship) % 360
        # print("relative_ " + str(relative_course))
        if angle_between_vessels <= 185 and angle_between_vessels >= 175:
            scenario = "HEAD ON"
        elif angle_between_vessels > 10 and angle_between_vessels < 175 or angle_between_vessels > 185 and angle_between_vessels < 360:

            if course_trg_ship > course_own_ship:
                if relative_course > 10 and relative_course  < 175:
                    scenario = "CROSSING STARBOARD TO PORT"
                elif relative_course > 185 and relative_course  < 360:
                    scenario = "CROSSING PORT TO STARBOARD"
            elif course_own_ship > course_trg_ship:
                if relative_course > 10 and relative_course  < 175:
                    scenario = "CROSSING STARBOARD TO PORT"
                elif relative_course > 185 and relative_course  < 360:
                    scenario = "CROSSING PORT TO STARBOARD"
        elif angle_between_vessels >= 0 and angle_between_vessels <= 10:
            if own_ship_speed > trg_ship_speed:
                scenario = "OVERTAKING"
            elif own_ship_speed < trg_ship_speed:
                scenario = "BEING OVERTAKEN"
        else:
            scenario = "NO COLLISION"

        self.trgShipsData[trg_ship_name]['scenario'] = scenario

        print(scenario)

def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceScenario()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
