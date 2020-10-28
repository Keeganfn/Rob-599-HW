#!/usr/bin/env python

import rospy
import sys
import math
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped

def callback(msg):
    #creates transform listener and waits for a response
    transform_listener = tf.TransformListener()
    while True:
        try:
            translation, rotation = transform_listener.lookupTransform("laser_link", "base_link", rospy.Time())
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Not Ready")
            continue    
    #list that will contain our new filtered ranges
    in_front = []
    #turns the y coordinate of each laser out of polar and translates it down to the base 
    #then checks to see if it lies within the 1 meter width of our robot and adds to list, if not fills the new list with an inf
    #doesnt account for rotation, so its not as general as I would like but it will work in most plausible situations including fetch
    for i in range(len(msg.ranges)):
        laser_angle = msg.angle_min + (msg.angle_increment * i) 
        y = (msg.ranges[i] * math.sin(laser_angle)) + translation[1]     
        if(y <= 0.5 and y >= -0.5):
              in_front.append(msg.ranges[i])  
        else:   
            in_front.append(float("inf"))

    #msg now contains our modified ranges
    msg.ranges = in_front

    #published the modified laserscan to base_scan_filtered
    pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("filtered_laser_scan", argv=sys.argv)
    sub = rospy.Subscriber("base_scan", LaserScan, callback, queue_size = None)
    pub = rospy.Publisher("base_scan_filtered", LaserScan, queue_size=1000)
    rospy.spin()
