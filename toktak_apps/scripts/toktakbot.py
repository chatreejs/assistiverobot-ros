#!/usr/bin/env python3

import rospy
import os
import math
import time
import requests

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import PowerSystemEvent, AutoDockingAction, AutoDockingGoal, SensorState

class Toktakbot():
    ######## CHANGE THE FOLLOWING VALUES #########
    SERVER_END_POINT = os.environ['WEB_SERVICE_END_POINT']
    # x coordinate for pose approx 1 meter from docking station
    NEAR_DOCKING_STATION_X = -0.6
    # y coordinate for pose approx 1 meter from docking station
    NEAR_DOCKING_STATION_Y = 0.0
    ######## END CHANGE THE FOLLOWING VALUES #########

    ####### OPTIONVAL VALUES TO CHANGE ##########
    KOBUKI_BASE_MAX_CHARGE = 160
    ####### END OPTIONVAL VALUES TO CHANGE ##########

    ####### DEFAULT VALUES ##########
    move_base = False
    # is kobuki's battery low?
    is_battery_low = False
    # 1000 isn't possible. Just a large fake # so the script starts believing the battery is fine
    kobuki_previous_battery_level = 1000
    # can't leave docking station until it's full because battery was low
    is_charging = False
    ####### END DEFAULT VALUES ##########

    def __init__(self):
        # Initialize ros node
        rospy.init_node("toktak_prototype", anonymous=False)

        # What to do if shut down (e.g. ctrl + C or failure)
        rospy.on_shutdown(self.shutdown)

        # tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)
        rospy.loginfo("wait for the action server to come up")
        # allow up to 30 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(30))

        # Monitor Kobuki's power and charging status.
        # If an event occurs (low battery, charging, not charging etc) call function SensorPowerEventCallback
        rospy.Subscriber("/mobile_base/sensors/core",
                         SensorState, self.SensorPowerEventCallback)

    def walking(self):
        # Before we deliver the next order... 
        # How is power looking? If low go recharge first at the docking station.
        if (self.is_need_power()):
            return True

        # Power is fine so let's see if anyone needs delivery...
        rospy.loginfo("Anyone need delivery?")

        # We'll send a goal to the robot to tell it to move to a pose that's near the docking station
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Call the server to see the next pending customer's order
        data = None
        result = requests.get(self.SERVER_END_POINT + "/jobs?status=pending&limit=1").json()['result']
        if result is not None:
            data = result[0]

        if (data):
            # If we're at the charging station back up 0.2 meters to avoid collision with dock
            self.exit_from_charging_station()

            # Send api to change job status to running
            requests.patch(self.SERVER_END_POINT + "/jobs/" + str(data['job_id']), json={'status':'running'})

            goal_start = data['goal'][0]
            goal_destination = data['goal'][1]

            # Set start goal pose
            goal.target_pose.pose.position.x = goal_start['position']['x']
            goal.target_pose.pose.position.y = goal_start['position']['y']
            goal.target_pose.pose.position.z = goal_start['position']['z']
            goal.target_pose.pose.orientation.x = goal_start['orientation']['x']
            goal.target_pose.pose.orientation.y = goal_start['orientation']['y']
            goal.target_pose.pose.orientation.z = goal_start['orientation']['z']
            goal.target_pose.pose.orientation.w = goal_start['orientation']['w']

            rospy.loginfo("Going to first goal")
            self.move_base.send_goal(goal)

            success = self.move_base.wait_for_result(rospy.Duration(180))

            if success:
                requests.patch(self.SERVER_END_POINT + "/goals/" + str(goal_start['goal_id']), json={'status':'arrived'})
                while True:
                    rospy.loginfo(
                        "Hooray, reached the desired pose! Waiting for user confirm to continue.")
                    status = requests.get(self.SERVER_END_POINT + "/jobs/" + str(data['job_id'])).json()['result'][0]['goal'][0]['status']
                    if status == "success":
                        break
                    time.sleep(5)
            else:
                # failed to reach goal (e.g. ToktakBot can't find a way to go to the location)
                self.move_base.cancel_goal()
                rospy.loginfo("The base failed to reach the desired pose")
                requests.patch(self.SERVER_END_POINT + "/jobs/" + str(data['job_id']), json={'status':'failed'})
                requests.patch(self.SERVER_END_POINT + "/goals/" + str(goal_start['goal_id']), json={'status':'failed'})
                return True
            
            # Set destination goal pose
            goal.target_pose.pose.position.x = goal_destination['position']['x']
            goal.target_pose.pose.position.y = goal_destination['position']['y']
            goal.target_pose.pose.position.z = goal_destination['position']['z']
            goal.target_pose.pose.orientation.x = goal_destination['orientation']['x']
            goal.target_pose.pose.orientation.y = goal_destination['orientation']['y']
            goal.target_pose.pose.orientation.z = goal_destination['orientation']['z']
            goal.target_pose.pose.orientation.w = goal_destination['orientation']['w']

            rospy.loginfo("Going to destination goal")
            self.move_base.send_goal(goal)

            success = self.move_base.wait_for_result(rospy.Duration(180))

            if success:
                requests.patch(self.SERVER_END_POINT + "/goals/" + str(goal_destination['goal_id']), json={'status':'arrived'})
                while True:
                    rospy.loginfo(
                        "Hooray, reached the desired pose! Waiting for user confirm to continue.")
                    status = requests.get(self.SERVER_END_POINT + "/jobs/" + str(data['job_id'])).json()['result'][0]['goal'][1]['status']
                    if status == "success":
                        requests.patch(self.SERVER_END_POINT + "/jobs/" + str(data['job_id']), json={'status':'success'})
                        break
                    time.sleep(5)
            else:
                # failed to reach goal (e.g. ToktakBot can't find a way to go to the location)
                self.move_base.cancel_goal()
                rospy.loginfo("The base failed to reach the desired pose")
                requests.patch(self.SERVER_END_POINT + "/jobs/" + str(data['job_id']), json={'status':'failed'})
                requests.patch(self.SERVER_END_POINT + "/goals/" + str(goal_destination['goal_id']), json={'status':'failed'})
                return True
        else:
            if (not self.is_charging):
                rospy.loginfo(
                    "Battery is fine but considering no one wants delivery ... Going to docking station.")
                self.dock_with_charging_station()
            else:
                time.sleep(2)

        return True

    def is_need_power(self):
        # Are we currently charging at the docking station?
        # If yes only continue if we're not fully charged
        if (self.is_charging and self.is_battery_low):
            rospy.loginfo(
                "I'm charging and will continue when I'm sufficiently charged")
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
        # If you set a goal while it's docked it tends to run into the docking station while turning.
        # Tell it to back up a little before initiliazing goals.
        if (self.is_charging):
            rospy.loginfo("We're exit from the docking station")
            cmd_vel = rospy.Publisher(
                "/mobile_base/commands/velocity", Twist, queue_size=10)
            # Twist is a datatype for velocity
            move_cmd = Twist()
            # let's go backward at 0.1 m/s
            move_cmd.linear.x = -0.1
            # let's turn at 0 radians/s
            move_cmd.angular.z = 0

            r = rospy.Rate(10)
            # As long as you haven't ctrl + c keeping doing...
            temp_count = 0
            # Go backward at 0.1 m/s for 2 seconds
            while (not rospy.is_shutdown() and temp_count < 50):
                # Publish the velocity
                cmd_vel.publish(move_cmd)
                # Wait for 0.1 seconds (10 HZ) and publish again
                temp_count = temp_count + 1
                r.sleep()
            # Make sure ToktakBot stops by sending a default Twist()
            cmd_vel.publish(Twist())
            return True

    def dock_with_charging_station(self):
        # Before we can run auto-docking we need to be close to the docking station..
        if (not self.go_close_to_charging_station()):
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

    def go_close_to_charging_station(self):
        # the auto docking script works well as long as you are roughly 1 meter from the docking station.
        # So let's get close first...
        rospy.loginfo("Let's go near the docking station")

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        # set a Pose near the docking station
        goal.target_pose.pose.position.x = self.NEAR_DOCKING_STATION_X
        goal.target_pose.pose.position.y = self.NEAR_DOCKING_STATION_Y
        goal.target_pose.pose.orientation.z = 0
        goal.target_pose.pose.orientation.w = 1

        self.move_base.send_goal(goal)

        success = self.move_base.wait_for_result(rospy.Duration(60))

        if success:
            rospy.loginfo("Hooray, reached the desired pose near the charging station")
            return True
        else:
            # failed to reach goal (e.g. TurtleBot can't find a way to go to the location)
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to reach the desired pose near the charging station")
            return False

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

    def shutdown(self):
        rospy.loginfo("Stop")


if __name__ == '__main__':
    delivery_checks = 0
    try:
        toktakbot = Toktakbot()
        # Keep checking for deliver until we shutdown the script with ctrl + c
        while (toktakbot.walking() and not rospy.is_shutdown()):
            delivery_checks = delivery_checks + 1
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")
