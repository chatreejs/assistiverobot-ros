#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleAvoidance():
    def __init__(self):
        rospy.init_node('ObstacleAvoidance', anonymous=False)

        rospy.loginfo("Press CTRL+c to stop Kobuki")

        rospy.on_shutdown(self.shutdown)

        self.cmd_vel = rospy.Publisher(
            '/mobile_base/commands/velocity', Twist, queue_size=10)
        self.laser_sensor = rospy.Subscriber(
            '/laser/scan', LaserScan, self.callback_laser)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()

    def callback_laser(self, msg):
        regions = {
            "right": min(min(msg.ranges[500:699]), 10),
            "fright": min(min(msg.ranges[700:899]), 10),
            "front": min(min(msg.ranges[900:1099]), 10),
            "fleft": min(min(msg.ranges[1100:1299]), 10),
            "left": min(min(msg.ranges[1300:1499]), 10),
        }

        self.take_action(regions)

    def take_action(self, regions):
        move_cmd = Twist()
        linear = 0
        angular = 0

        state_description = ""

        if regions["front"] > 1 and regions["fleft"] > 1 and regions["fright"] > 1:
            state_description = "case 1 - nothing"
            linear = 0.4
            angular = 0
        elif regions["front"] < 1 and regions["fleft"] > 1 and regions["fright"] > 1:
            state_description = "case 2 - front"
            linear = 0
            angular = 0.7
        elif regions["front"] > 1 and regions["fleft"] > 1 and regions["fright"] < 1:
            state_description = "case 3 - fright"
            linear = 0
            angular = 0.7
        elif regions["front"] > 1 and regions["fleft"] < 1 and regions["fright"] > 1:
            state_description = "case 4 - fleft"
            linear = 0
            angular = -0.7
        elif regions["front"] < 1 and regions["fleft"] > 1 and regions["fright"] < 1:
            state_description = "case 5 - front and fright"
            linear = 0
            angular = 0.7
        elif regions["front"] < 1 and regions["fleft"] < 1 and regions["fright"] > 1:
            state_description = "case 6 - front and fleft"
            linear = 0
            angular = -0.7
        elif regions["front"] < 1 and regions["fleft"] < 1 and regions["fright"] < 1:
            state_description = "case 7 - front and fleft and fright"
            linear = 0
            angular = 0.5
        elif regions["front"] > 1 and regions["fleft"] < 1 and regions["fright"] < 1:
            state_description = "case 8 - fleft and fright"
            linear = 0.3
            angular = 0
        else:
            state_description = "unknown case"
            rospy.loginfo(regions)

        rospy.loginfo(state_description)
        move_cmd.linear.x = linear
        move_cmd.angular.z = angular
        self.cmd_vel.publish(move_cmd)

    def shutdown(self):
        rospy.loginfo("Stopping Kobuki")

        self.cmd_vel.publish(Twist())

        rospy.sleep(1)


if __name__ == '__main__':
    try:
        ObstacleAvoidance()
    except:
        rospy.loginfo("End of the trip for Kobuki")
