#!/usr/bin/env python

import rospy
import sys
import actionlib

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rob599_hw1.srv import SetDistance, SetDistanceResponse
from rob599_hw1.msg import MoveRobotAction, MoveRobotGoal, MoveRobotFeedback, MoveRobotResult

class Approach:

    def __init__(self):
        self.distance = 1
        self.closest_obstacle = 1000
        self.distance_reached = False
        self.sub = rospy.Subscriber("base_scan_filtered", LaserScan, self.sub_callback)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.service = rospy.Service("stopping_distance", SetDistance, self.service_callback )
        self.action_server = actionlib.SimpleActionServer("set_goal", MoveRobotAction, self.action_callback, False)
        self.action_server.start()

    def service_callback(self, request):
        if(isinstance(request.distance, float) and request.distance > .5):
            self.distance = request.distance 
            return SetDistanceResponse(True)
        else:
            return SetDistanceResponse(False)

    def action_callback(self, goal):
        self.distance_reached = False
        if(isinstance(goal.distance, float) and goal.distance > .5):
            self.distance = goal.distance 

            while(not self.distance_reached):
                self.action_server.publish_feedback(MoveRobotFeedback(current_distance = self.closest_obstacle))
                if(self.action_server.is_new_goal_available()):
                    self.action_server.set_preempted(MoveRobotResult(result = False))

            self.action_server.set_succeeded(MoveRobotResult(result = True))
        else:
            self.action_server.set_aborted(MoveRobotResult(result = False))




    def set_velocity_message(self):

        if(self.closest_obstacle > self.distance + .07):
            direction = min(self.closest_obstacle * 0.2, 1)
        elif(self.closest_obstacle < self.distance - .07):
            direction = max(min(-self.closest_obstacle * 0.2, -0.1), -1)
        else:
            direction = 0
            self.distance_reached = True

        msg = Twist()
        msg.linear.x = direction 
        msg.linear.y = 0 
        msg.linear.z = 0 
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        return msg

    def sub_callback(self, msg):
        for i in msg.ranges:
            if(i < self.closest_obstacle):
                self.closest_obstacle = i

        rospy.loginfo(self.closest_obstacle)	
        velocity_msg = self.set_velocity_message()
        self.pub.publish(velocity_msg)



if __name__ == '__main__':
    rospy.init_node("drive_forward", argv=sys.argv)
    begin_approach = Approach()
    rospy.spin()
