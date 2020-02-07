#! /usr/bin/env python

import rospy

from kobuki_msgs.msg import BumperEvent


def BumperCallBack(msg):
    if (msg.bumper == BumperEvent.CENTER and msg.state == BumperEvent.PRESSED):
        rospy.loginfo("Front hit")
    elif (msg.bumper == BumperEvent.LEFT and msg.state == BumperEvent.PRESSED):
        rospy.loginfo("Left hit")
    elif (msg.bumper == BumperEvent.RIGHT and msg.state == BumperEvent.PRESSED):
        rospy.loginfo("Right hit")
    else:
        rospy.loginfo("Nothing hit")


def main():
    rospy.init_node("reading_bumper")

    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, BumperCallBack)

    rospy.spin()


if __name__ == "__main__":
    main()
