#!/usr/bin/env python

import rospy
import sys
import actionlib
import tf

from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from rob599_hw1.srv import SetDistance, SetDistanceResponse
from rob599_hw1.msg import MoveRobotAction, MoveRobotGoal, MoveRobotFeedback, MoveRobotResult

class Approach:

    #class initializer for all of the publishers, subscribers, services, action servers and variables needed
    def __init__(self):
        self.distance = 1
        self.closest_obstacle = 1000
        self.distance_reached = False
        self.action_active = False
        self.count = 0

        self.sub = rospy.Subscriber("base_scan", LaserScan, self.sub_callback)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.pub_marker = rospy.Publisher("closest_obstacle_marker", Marker, queue_size=10)
        self.service = rospy.Service("stopping_distance", SetDistance, self.service_callback )

        #Decided to use a goal callback action server so that it would not block my subscriber and feedback is easier
        self.action_server = actionlib.SimpleActionServer("set_goal", MoveRobotAction, auto_start=False)
        self.action_server.register_goal_callback(self.action_callback)
        self.action_server.register_preempt_callback(self.preempt_callback)
        self.action_server.start()

    #error checks to see if a valid distance is entered (I chose any float over .5 to be valid) also makes sure action server isnt active
    def service_callback(self, request):
        if(isinstance(request.distance, float) and request.distance > .5 and self.action_active != True):
            self.distance = request.distance 
            return SetDistanceResponse(True)
        else:
            return SetDistanceResponse(False)

    #accepts new goal and sets the distance we want to go to as well as resetting any flags for other nodes so that it knows when it has stopped
    #also makes sure the service cant be called and error checks the input
    def action_callback(self):
        self.distance_reached = False
        self.action_active = True
        self.count = 0
        self.new_goal = self.action_server.accept_new_goal()
        self.distance = self.new_goal.distance 

        if(isinstance(self.new_goal.distance, float) and self.new_goal.distance > .5):
            self.distance = self.new_goal.distance 
        else:
            self.action_active = False
            self.action_server.set_aborted(MoveRobotResult(result = False))

    #If the action service gets preempted this preempt callback is called resetting the action active flag
    def preempt_callback(self):
        self.action_active = False
        self.action_server.set_preempted(MoveRobotResult(result = False))

    #does all the heavy lifting, takes the filtered laser data and finds the closest obstacle, it then publishes any markers needed and 
    #publishes a velocity message to move the fetch towards its goal. When the action server flag is active it also handles publishing feedback
    def sub_callback(self, msg):
        self.closest_obstacle = 1000
        angle_index = 0
        temp = 0
        #loops through the ranges and finds the closest obstacle as well as the angle its at
        for i in msg.ranges: 
            temp+=1
            if(i < self.closest_obstacle):
                self.closest_obstacle = i
                angle_index = temp

        #finds the angle the closest obstacle is at
        angle = msg.angle_min + (msg.angle_increment * angle_index)

        #When active published feedback of how close it is to the obstacle and once we have reached goal distance returns the result
        if(self.action_active == True):
            self.action_server.publish_feedback(MoveRobotFeedback(current_distance = self.closest_obstacle))
            if(self.distance_reached == True): 
                self.action_active = False
                self.action_server.set_succeeded(MoveRobotResult(result = True))

        #sets the markers and velocity msg to correct values for publishing
        arrow_marker = self.set_arrow_marker(angle)
        text_marker = self.set_text_marker()
        velocity_msg = self.set_velocity_message()

        #publishes the above messages 
        self.pub_marker.publish(arrow_marker)
        self.pub_marker.publish(text_marker)
        self.pub.publish(velocity_msg)

    #controls the gradual acceleration and deceleration and returns a twist message
    def set_velocity_message(self):
        if(self.closest_obstacle > self.distance + .07):
            direction = min(max((self.closest_obstacle-self.distance) * 0.3, .25), .7)
        elif(self.closest_obstacle < self.distance - .07):
            direction = max(min((self.closest_obstacle-self.distance) * 0.3, -0.25), -.7)
        else:
            direction = 0
            self.count += 1
            #makes sure fetch is fully stopped
            if(self.count > 50):
                self.distance_reached = True

        msg = Twist()
        msg.linear.x = direction 
        msg.linear.y = 0 
        msg.linear.z = 0 
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        return msg

    #sets arrow marker for closest object
    def set_arrow_marker(self, angle):
        #normalizes a quaternion for the orientation based on the angle 
        rotation = tf.transformations.quaternion_from_euler(0,0,angle)
        arrow = Marker()
        arrow.header.frame_id = "/laser_link"
        arrow.type = arrow.ARROW
        arrow.id = 0
        arrow.action = arrow.ADD
        arrow.scale.x = self.closest_obstacle
        arrow.scale.y = .05
        arrow.scale.z = .05
        arrow.color.r = 0
        arrow.color.g = 0
        arrow.color.b = 1.0
        arrow.color.a = 1.0
        arrow.pose.position.x = 0.0
        arrow.pose.position.y = 0.0
        arrow.pose.position.z = 0.0
        arrow.pose.orientation.x = rotation[0]
        arrow.pose.orientation.y = rotation[1]
        arrow.pose.orientation.z = rotation[2]
        arrow.pose.orientation.w = rotation[3]
        return arrow

    #sets a text marker to display closest obstacle distance
    def set_text_marker(self):
        distance_text = Marker() 
        distance_text.header.frame_id = "/laser_link"
        distance_text.type = distance_text.TEXT_VIEW_FACING
        distance_text.id = 1 
        distance_text.action = distance_text.ADD
        distance_text.color.r = 0
        distance_text.color.g = 0
        distance_text.color.b = 1.0
        distance_text.color.a = 1.0
        distance_text.scale.z = .25
        distance_text.pose.position.x = self.closest_obstacle
        distance_text.pose.position.y = 0.0
        distance_text.pose.position.z = 0.0
        distance_text.text = str(self.closest_obstacle)
        return distance_text
        


if __name__ == '__main__':
    rospy.init_node("approach", argv=sys.argv)
    #starts everything
    begin_approach = Approach()
    rospy.spin()
