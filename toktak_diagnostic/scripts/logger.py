#!/usr/bin/env python

import rospy

from kobuki_msgs.msg import SensorState
from Adafruit_CharLCD import Adafruit_CharLCD

import math

class Logger():
    KOBUKI_BASE_MAX_CHARGE = 160
    kobuki_previous_battery_level = 1000

    lcd = Adafruit_CharLCD(rs=26, en=19, d4=13, d5=6, d6=5, d7=11, cols=16, lines=2)

    def __init__(self):
        self.lcd.clear()
        self.lcd.message("Starting...")

        rospy.init_node("toktak_logger", anonymous=False)

        rospy.on_shutdown(self.shutdown)

        rospy.Subscriber("/mobile_base/sensors/core",
                            SensorState, self.SensorPowerEventCallback)
        rospy.spin()

    def SensorPowerEventCallback(self, msg):
        if (math.fabs(int(msg.battery) - self.kobuki_previous_battery_level) > 2):
            battery_level = round(float(msg.battery) / float(self.KOBUKI_BASE_MAX_CHARGE) * 100)
            rospy.loginfo("Kobuki's battery is now: " + str(battery_level) + "%")
            self.kobuki_previous_battery_level = int(msg.battery)
            self.display_lcd(battery_level)

    def display_lcd(self, msg):
        self.lcd.clear()
        self.lcd.message("Battery: " + str(msg) + "%")

    def shutdown(self):
        self.lcd.clear()
        self.lcd.message("Stop")
        rospy.loginfo("Stop")

if __name__ == '__main__':
    temp = 0
    try:
        logger = Logger()
        while (not rospy.is_shutdown()):
            temp = temp + 1
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")