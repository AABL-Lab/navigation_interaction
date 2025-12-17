#! /usr/bin/env python
# Joey Grossman 2025
# Joseph.Grossman@Tufts.edu

import copy
import rospy
import actionlib
import navigation_interaction.msg
from std_msgs.msg import String
import armpy.arm
import pickle
import os


class MoveArmAction(object):
    _feedback = navigation_interaction.msg.MoveArmActionFeedback
    _result = navigation_interaction.msg.MoveArmActionResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, navigation_interaction.msg.MoveArmAction, execute_cb=self.execute_cb, auto_start=False)
        self.arm = armpy.arm.Arm()
        self._as.start()
    
    def execute_cb(self, goal):
        r = rospy.Rate(1)
        success = True

        rospy.loginfo('%s: Moving arm to %s' % (self._action_name, goal.behavior))

        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self.execute_motion_plan('/home/tufts_user/catkin_ws/src/navigation_interaction/src/navigation_interaction/trajectorypickles/point_to_home.pkl')
            self._as.set_aborted()
            success = False

        
        filepath = '/home/tufts_user/catkin_ws/src/navigation_interaction/src/navigation_interaction/trajectorypickles/'+goal.behavior+'.pkl' 
        self.execute_motion_plan(planfilename=filepath)

        self._as.publish_feedback(self._feedback)

        if success:
            self._result.result = "Success"
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

    def execute_motion_plan(self, planfilename="Point.pkl"):
        with open (planfilename, "rb") as f:
            plan = pickle.load(f)
            
        rospy.loginfo("running the loaded trajectory %s" % planfilename)
        rospy.loginfo("Executing the trajectory")
        for i, plan in enumerate(plan):
            rospy.loginfo("executing plan %i" % i) 
            self.arm.move_robot(plan)

if __name__ == '__main__':
    rospy.init_node('move_arm')
    server = MoveArmAction(rospy.get_name())
    rospy.spin()







