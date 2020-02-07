#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan


def LaserCallBack(msg):
    # 1000 / 5 = 200
    # regions = [
    #     min(min(msg.ranges[500:699]), 10),
    #     min(min(msg.ranges[700:899]), 10),
    #     min(min(msg.ranges[900:1099]), 10),
    #     min(min(msg.ranges[1100:1299]), 10),
    #     min(min(msg.ranges[1300:1499]), 10),
    # ]
    # # rospy.loginfo(regions)
    rospy.loginfo(len(msg.ranges))


def main():
    rospy.init_node("reading_laser")

    rospy.Subscriber("/scan", LaserScan, LaserCallBack)

    rospy.spin()


if __name__ == "__main__":
    main()
