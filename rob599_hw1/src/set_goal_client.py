#!/usr/bin/env python

import rospy
import sys
import actionlib

from rob599_hw1.msg import MoveRobotAction, MoveRobotGoal, MoveRobotFeedback, MoveRobotResult





def complete_callback(status, result):
    rospy.loginfo("RETURNED RESULT: {0}".format(result.result))
    
def feedback_callback(feedback):
    rospy.loginfo("FEEDBACK (DISTANCE TO NEAREST OBJECT): {0}".format(feedback.current_distance))

def active_callback():
    rospy.loginfo("ACTIVE")

if __name__ == '__main__':
    
    try:
        new_distance = float(sys.argv[1])
    except:
        new_distance = 1

    rospy.init_node("set_goal_client", argv=sys.argv)
    client = actionlib.SimpleActionClient("set_goal", MoveRobotAction)
    client.wait_for_server()

    goal = MoveRobotGoal(distance = new_distance)


    client.send_goal(goal, done_cb=complete_callback, active_cb=active_callback, feedback_cb=feedback_callback)
    client.wait_for_result()

