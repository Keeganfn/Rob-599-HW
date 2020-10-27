#!/usr/bin/env python

import rospy
import sys
import math
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped

def callback(msg):
    transform_listener = tf.TransformListener()
    while True:
        try:
            translation, rotation = transform_listener.lookupTransform("laser_link", "base_link", rospy.Time())
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Not Ready")
            continue    

    in_front = []
    #count = 1
    for i in range(len(msg.ranges)):
        laser_angle = msg.angle_min + (msg.angle_increment * i) 
        y = (msg.ranges[i] * math.sin(laser_angle)) + translation[1]    
    
        #laser_point = PointStamped()
        #laser_point.header.frame_id = "laser_link"
        #laser_point.header.stamp = rospy.Time()    
        #laser_point.point.x = msg.ranges[i] * math.cos(laser_angle)    
        #laser_point.point.y = msg.ranges[i] * math.sin(laser_angle)    
        #laser_point.point.z = 0
        #location = transform_listener.transformPoint("base_link", laser_point) 
    
        if(y <= 0.5 and y >= -0.5):
            #if(count == 1):
                #new_min = laser_angle
                #count += 1

            in_front.append(msg.ranges[i])  
            #new_max = laser_angle
        else:   
            in_front.append(float("inf"))

#   new_laser_msg = LaserScan()
#   new_laser_msg = msg
#   new_laser_msg.angle_min = new_min
#   new_laser_msg.angle_max = new_max
#   new_laser_msg.ranges = in_front
#   new_laser_msg.intensities = []
#   msg.angle_min = new_min
#   msg.angle_max = new_max
    msg.ranges = in_front

    pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("filtered_laser_scan", argv=sys.argv)
    sub = rospy.Subscriber("base_scan", LaserScan, callback, queue_size = None)
    pub = rospy.Publisher("base_scan_filtered", LaserScan, queue_size=1000)
    rospy.spin()
