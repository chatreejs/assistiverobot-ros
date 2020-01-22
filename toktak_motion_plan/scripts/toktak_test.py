#!/usr/bin/env python

import rospy

import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import PowerSystemEvent, AutoDockingAction, AutoDockingGoal, SensorState
from kobuki_msgs.msg import ButtonEvent
import math

import time


class toktak_test():
    ####### OPTIONVAL VALUES TO CHANGE ##########
    kobuki_base_max_charge = 160
    ####### END OPTIONVAL VALUES TO CHANGE ##########

    # defaults
    battery_is_low = False
    kobuki_previous_battery_level = 1000
    charging_at_dock_station = False
    cannot_move_until_b0_pressed = False
    data = False

    def __init__(self):
        rospy.init_node("toktak_test", anonymous=False)

        rospy.on_shutdown(self.shutdown)

        rospy.Subscriber("/mobile_base/sensors/core",
                         SensorState, self.SensorPowerEventCallback)
        rospy.Subscriber("/mobile_base/events/button",
                         ButtonEvent, self.ButtonEventCallback)

    def walking(self):
        if (self.cannot_move_until_b0_pressed):
            rospy.loginfo("Waiting for button B0 to be pressed.")
            time.sleep(2)
            return True

        if (self.data):
            self.DoWeNeedToBackUpFromChargingStation()
        else:
            self.DockWithChargingStation()

        return True

    def DoWeNeedToBackUpFromChargingStation(self):
        if (self.charging_at_dock_station):
            rospy.loginfo("We're at the docking station")
            cmd_vel = rospy.Publisher(
                "/mobile_base/commands/velocity", Twist, queue_size=10)
            move_cmd = Twist()
            move_cmd.linear.x = -0.1
            move_cmd.angular.z = 0

            r = rospy.Rate(10)
            temp_count = 0
            while (not rospy.is_shutdown() and temp_count < 20):
                cmd_vel.publish(move_cmd)
                temp_count = temp_count + 1
                r.sleep()
            cmd_vel.publish(Twist())
            return True

    def SensorPowerEventCallback(self, data):
        # kobuki's batttery value tends to bounce up and down 1 constantly so only report if difference greater than 1
        if (math.fabs(int(data.battery) - self.kobuki_previous_battery_level) > 2):
            rospy.loginfo("Kobuki's battery is now: " + str(
                round(float(data.battery) / float(self.kobuki_base_max_charge) * 100)) + "%")
            self.kobuki_previous_battery_level = int(data.battery)

        if (int(data.charger) == 0):
            if (self.charging_at_dock_station):
                rospy.loginfo("Stopped charging at docking station")
            self.charging_at_dock_station = False
        else:
            if (not self.charging_at_dock_station):
                rospy.loginfo("Charging at docking station")
            self.charging_at_dock_station = True

        if (round(float(data.battery) / float(self.kobuki_base_max_charge) * 100) < 50):
            if (not self.battery_is_low):
                rospy.loginfo("Kobuki battery is low")
            self.battery_is_low = True
        # the logic of not using the same value (e.g. 50) for both the battery is low & battery is fine is that 
        # it'll leave and immediatly return for more power.  
        # The reason why we don't use == 100 is that we hope that proactive charging between coffee deliveries will charge it soon and we don't want people waiting.
        elif (round(float(data.battery) / float(self.kobuki_base_max_charge) * 100) > 60):
            if (self.battery_is_low):
                rospy.loginfo("Kobuki battery is fine")
            self.battery_is_low = False

    def DockWithChargingStation(self):
        if (True):
            return self.StartDocking()

    def StartDocking(self):
        self._client = actionlib.SimpleActionClient(
            "/dock_drive_action", AutoDockingAction)
        rospy.loginfo("Waiting for auto_docking server")
        self._client.wait_for_server()
        rospy.loginfo("auto_docking server found")
        goal = AutoDockingGoal()
        rospy.loginfo(
            "Sending auto_docking goal and waiting for result (times out in 180 seconds and will try again if required)")
        self._client.send_goal(goal)

        success = self._client.wait_for_result(rospy.Duration(60))

        if success:
            rospy.loginfo("auto_docking succeeded")
            self.charging_at_dock_station = True
            return True
        else:
            rospy.loginfo("auto_docking failed")
            return False

    def ButtonEventCallback(self, msg):
        if (msg.button == ButtonEvent.Button0):
            self.cannot_move_until_b0_pressed = False

        if (msg.button == ButtonEvent.Button2):
            self.data = True

    def shutdown(self):
        rospy.loginfo("Stop")


if __name__ == '__main__':
    delivery_checks = 0
    try:
        toktakbot = toktak_test()
        while (toktakbot.walking() and not rospy.is_shutdown()):
            delivery_checks = delivery_checks + 1
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")
