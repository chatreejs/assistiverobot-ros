#!/usr/bin/env python

import rospy
import actionlib
import requests

from move_base_msgs.msg import *
from geometry_msgs.msg import *

def simple_move(position):
    print position
    start = position['goal'][0]
    destination = position['goal'][1]
    sac = actionlib.SimpleActionClient('move_base',MoveBaseAction)

    # Start 

    #create goal
    goal = MoveBaseGoal()
    goal.target_pose.pose.position.x = start['position']['x']
    goal.target_pose.pose.position.y = start['position']['y']
    goal.target_pose.pose.orientation.w = start['orientation']['w']
    goal.target_pose.pose.orientation.z = start['orientation']['z']
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    #start listner
    sac.wait_for_server()
    #send goal
    sac.send_goal(goal)
    print "Sending goal" ,start['position']['x'],start['position']['y'],start['orientation']['w'],start['orientation']['z']
    #finish
    sac.wait_for_result()
    #print result
    print sac.get_result()

    #Destination
    goal = MoveBaseGoal()
    goal.target_pose.pose.position.x = destination['position']['x']
    goal.target_pose.pose.position.y = destination['position']['y']
    goal.target_pose.pose.orientation.w = destination['orientation']['w']
    goal.target_pose.pose.orientation.z = destination['orientation']['z']
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    #start listner
    sac.wait_for_server()
    #send goal
    sac.send_goal(goal)
    print "Sending goal" ,destination['position']['x'],destination['position']['y'],destination['orientation']['w'],destination['orientation']['z']
    #finish
    sac.wait_for_result()
    #print result
    print sac.get_result()




if __name__=='__main__':
    try: 
        rospy.init_node('simple_move',anonymous=True)
        rate = rospy.Rate(1) # 1 task per 1 second
        while True:
            data = requests.get("http://192.168.31.197:5000/api/v1/jobs?status=pending").json()['result']
            if isinstance(data,dict): # 1 Data
                simple_move(data) 
            elif isinstance(data,list): # 1 more datas
                simple_move(data[0])

            rate.sleep
    except rospy.ROSInterruptException:
        print "keyboard Interrupt"