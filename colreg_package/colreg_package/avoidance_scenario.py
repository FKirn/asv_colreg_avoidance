import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from colreg_interfaces.msg import ShipData
from colreg_interfaces.msg import AvoidanceScenario
import logging
import math


class AvoidanceScenarioClass(Node):

    def __init__(self):
        super().__init__('determine_avoidance_scenario')
        self.trg_pose = Pose()
        self.own_pose = Pose()
        self.tcpa = 0.0
        self.dcpa = 0.0
        self.collision_point_x = 0.0
        self.collision_point_y = 0.0
        self.theta_target = 0.0
        self.theta_own = 0.0
        self.avoidance_scenario_type = 0
        self.ships_data_subscriber_ = self.create_subscription(ShipData, "ship_data", self.ships_data_callback, 10)
        self.avoidance_scenario_publisher_ = self.create_publisher(AvoidanceScenario, "avoidance_scenario", 10)

    def ships_data_callback(self, data):
        self.trg_pose.x = data.x_target
        self.trg_pose.y = data.y_target
        self.collision_point_x = data.collision_point_x
        self.collision_point_y = data.collision_point_y
        self.tcpa = data.tcpa
        self.dcpa = data.dcpa
        self.own_pose.x = data.x_own
        self.own_pose.y = data.y_own
        self.theta_target = data.theta_target
        self.theta_own = data.theta_own

        self.run_avoidance_scenario()

    def run_avoidance_scenario(self):
        if self.determine_collision_risk(self.tcpa, self.dcpa) is True:
            self.determine_avoidance_scenario()

    def determine_collision_risk(self, tcpa, dcpa):

        # if tcpa > 0.0:
        #     logging.warning("Ships are approaching!")
        # elif tcpa < 0.0:
        #     logging.warning("Ships are moving away and are out of danger!")

        if tcpa > 0.0 and dcpa < 1.0:
            # logging.warning("High risk of collision!!!")
            return True
        else:
            return False

    def determine_avoidance_scenario(self):

        msg = AvoidanceScenario()
        ships_dist_x_axis = self.trg_pose.x - self.own_pose.x
        ships_dist_y_axis = self.trg_pose.y - self.own_pose.y
        angle_between_ships = math.atan(ships_dist_y_axis / ships_dist_x_axis)
        angle_between_ships_abs = self.own_pose.theta - self.trg_pose.theta

        if angle_between_ships < 0:
            angle_between_ships = angle_between_ships + math.pi

        if self.own_pose.theta < 0:
            own_ship_angle_degrees = (self.own_pose.theta * 57.295779513) + 360

        if self.trg_pose.theta < 0:
            trg_ship_angle_degrees = (self.trg_pose.theta * 57.295779513) + 360



        if ((own_ship_angle_degrees <= 95 and own_ship_angle_degrees >= 85 and trg_ship_angle_degrees <= 275 and trg_ship_angle_degrees >= 265)
            or (own_ship_angle_degrees <= 275 and own_ship_angle_degrees >= 265 and trg_ship_angle_degrees <= 95 and trg_ship_angle_degrees >= 85)):
            msg.scenario = msg.HEAD_ON
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




        msg.header.stamp = self.get_clock().now().to_msg()
        self.avoidance_scenario_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceScenarioClass()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
