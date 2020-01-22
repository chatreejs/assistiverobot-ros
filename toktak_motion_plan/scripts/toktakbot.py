#!/usr/bin/env python

import rospy

import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import PowerSystemEvent, AutoDockingAction, AutoDockingGoal, SensorState
from kobuki_msgs.msg import ButtonEvent
import math

import time


class ToktakTest():
    ######## CHANGE THE FOLLOWING VALUES #########
    # x coordinate for pose approx 1 meter from docking station
    NEAR_DOCKING_STATION_X = -1.88
    # y coordinate for pose approx 1 meter from docking station
    NEAR_DOCKING_STATION_Y = 0.06
    ######## END CHANGE THE FOLLOWING VALUES #########

    ####### OPTIONVAL VALUES TO CHANGE ##########
    KOBUKI_BASE_MAX_CHARGE = 160
    ####### END OPTIONVAL VALUES TO CHANGE ##########

    ####### DEFAULT vALUES ##########
    # is kobuki's battery low?
    is_battery_low = False
    # 1000 isn't possible. Just a large fake # so the script starts believing the battery is fine
    kobuki_previous_battery_level = 1000
    # can't leave docking station until it's full because battery was low
    is_charging = False
    # should TurtleBot stay still until B0 is pressed (e.g. while the person is brewing coffee)?
    cannot_move_until_b0_pressed = False
    # fake data
    data = False
    ####### END DEFAULT VALUES ##########

    def __init__(self):
        # Initialize ros node
        rospy.init_node("toktak_test", anonymous=False)

        # What to do if shut down (e.g. ctrl + C or failure)
        rospy.on_shutdown(self.shutdown)

        # Monitor Kobuki's power and charging status.
        # If an event occurs (low battery, charging, not charging etc) call function SensorPowerEventCallback
        rospy.Subscriber("/mobile_base/sensors/core",
                         SensorState, self.SensorPowerEventCallback)

        # To avoid TurtleBot from driving to another pose while someone is making coffee ...
        # TurtleBot isn't allowed to move until the person presses the B0 button.
        # To implement this we need to monitor the kobuki button events
        rospy.Subscriber("/mobile_base/events/button",
                         ButtonEvent, self.ButtonEventCallback)

    def walking(self):
        if (self.cannot_move_until_b0_pressed):
            rospy.loginfo("Waiting for button B0 to be pressed.")
            time.sleep(2)
            return True
        
        if (self.is_need_power()):
            return True

        if (self.data):
            self.exit_from_charging_station()
        else:
            if (not self.is_charging):
                self.dock_with_charging_station()
            else:
                time.sleep(2)

        return True
    
    def is_need_power(self):
        # Are we currently charging at the docking station?
        # If yes only continue if we're not fully charged
        if (self.is_charging and self.is_battery_low):
            rospy.loginfo("I'm charging and will continue when I'm sufficiently charged")
            time.sleep(30)
            return True
        # Are we not currently charging and is either battery low?
        # If yes, go to docking station.
        if (not self.is_charging and self.is_battery_low):
            rospy.loginfo("Battery is low. Going to docking station.")
            self.dock_with_charging_station()
            return True
        return False

    def exit_from_charging_station(self):
        if (self.is_charging):
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

    def dock_with_charging_station(self):
        # Before we can run auto-docking we need to be close to the docking station..
        if (not self.go_to_charging_station()):
            return False

        # We're close to the docking station... so let's dock
        return self.start_docking()

    def start_docking(self):
        self._client = actionlib.SimpleActionClient(
            "/dock_drive_action", AutoDockingAction)
        rospy.loginfo("Waiting for auto_docking server")
        self._client.wait_for_server()
        rospy.loginfo("auto_docking server found")
        goal = AutoDockingGoal()
        rospy.loginfo(
            "Sending auto_docking goal and waiting for result (times out in 180 seconds and will try again if required)")
        self._client.send_goal(goal)

        success = self._client.wait_for_result(rospy.Duration(180))

        if success:
            rospy.loginfo("auto_docking succeeded")
            self.is_charging = True
            return True
        else:
            rospy.loginfo("auto_docking failed")
            return False
    
    def go_to_charging_station(self):
        return True

    def SensorPowerEventCallback(self, msg):
        # kobuki's batttery value tends to bounce up and down 1 constantly so only report if difference greater than 1
        if (math.fabs(int(msg.battery) - self.kobuki_previous_battery_level) > 2):
            rospy.loginfo("Kobuki's battery is now: " + str(
                round(float(msg.battery) / float(self.KOBUKI_BASE_MAX_CHARGE) * 100)) + "%")
            self.kobuki_previous_battery_level = int(msg.battery)

        if (int(msg.charger) == 0):
            if (self.is_charging):
                rospy.loginfo("Stopped charging at docking station")
            self.is_charging = False
        else:
            if (not self.is_charging):
                rospy.loginfo("Charging at docking station")
            self.is_charging = True

        if (round(float(msg.battery) / float(self.KOBUKI_BASE_MAX_CHARGE) * 100) < 50):
            if (not self.is_battery_low):
                rospy.loginfo("Kobuki battery is low")
            self.is_battery_low = True
        # the logic of not using the same value (e.g. 50) for both the battery is low & battery is fine is that
        # it'll leave and immediatly return for more power.
        # The reason why we don't use == 100 is that we hope that proactive charging between coffee deliveries
        # will charge it soon and we don't want people waiting.
        elif (round(float(msg.battery) / float(self.KOBUKI_BASE_MAX_CHARGE) * 100) > 60):
            if (self.is_battery_low):
                rospy.loginfo("Kobuki battery is fine")
            self.is_battery_low = False

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
        toktakbot = ToktakTest()
        while (toktakbot.walking() and not rospy.is_shutdown()):
            delivery_checks = delivery_checks + 1
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")
