#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun May  1 21:24:03 2022

@author: bezzo
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker,MarkerArray
import math as m
import numpy as np
import time


class messageClass():
    def __init__(self):
        self.goalx = 0
        self.goaly = 10
        self.velx = 0.0
        self.velz = 0.0
        self.target_velocity = 2.0
        self.posx = 0.0
        self.posy = 0.0
        self.theta = 0.0
        self.scandata = None
        self.ranges = []
        self.angle_inc = None
        self.scanCoords = []
        self.angles = []
        self.gapGoal = None
        self.buffer = 0.5
        self.footpr = 0.8
        self.minind = 0
        self.maxind = 719
        self.angle_min = None
        self.should_max_speed = False
        self.max_speed_started = time.time()

def distance(point1,point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1-point2)

def odomCallback(data):
    message.posx = data.pose.pose.position.x
    message.posy = data.pose.pose.position.y
    message.velx = data.twist.twist.linear.x
    message.velz = data.twist.twist.angular.z

def callback(msg):
    whats_in_front = np.asarray(msg.ranges[300:420])
    nearest_obs = np.min(whats_in_front)
    current_time = time.time()
    print(f"nearest obs: {nearest_obs} and type: {type(nearest_obs)}")

    if nearest_obs > 2.50 and (current_time - message.max_speed_started) < 3.0:
        #set message.max_speed_started to current time only if it was false before
        if message.should_max_speed == False:
            message.max_speed_started = time.time()
        
        print("#########Prinitng whats in front#########")
        print(f"no obstacle in front and time elapsed is : {current_time - message.max_speed_started}" )
        message.should_max_speed = True
        print(f"should max speed: {message.should_max_speed}")

    elif ((message.should_max_speed == True) and (nearest_obs < 2.50)) or ((current_time - message.max_speed_started)) > 3.0:
        message.should_max_speed = False
        print("obsacle in front/ time elapsed is more 3 seconds")
        print(f"should max speed: {message.should_max_speed}")


        
def main():
    global message
    message = messageClass()
    rospy.init_node('scan_values')
    sub = rospy.Subscriber('/front/scan', LaserScan, callback)
    odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odomCallback)
    vel_Pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    dist_to_goal = distance((message.goalx,message.goaly),(message.posx,message.posy))

    while not rospy.is_shutdown():
        if message.should_max_speed:
            vel_msg.linear.x = 2.0
            vel_msg.angular.z = 0.0
            vel_Pub.publish(vel_msg)
    
    if dist_to_goal < 0.5:
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        vel_Pub.publish(vel_msg)
        print("Goal Reached")
        rospy.signal_shutdown("Goal Reached")
        return

    rospy.spin()


if __name__ == '__main__':
    main()

