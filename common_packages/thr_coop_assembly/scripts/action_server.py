#! /usr/bin/env python

import rospy
import actionlib
import moveit_commander

from moveit_msgs.msg import RobotTrajectory
from thr_coop_assembly.msg import RunActionAction, Action, RunActionActionGoal

class ActionServer:
    def __init__(self):
        self.arms = {}
        self.arms['left'] = moveit_commander.MoveGroupCommander("left_arm")
        self.arms['right'] = moveit_commander.MoveGroupCommander("right_arm")
        self.server = actionlib.SimpleActionServer('/thr/action', RunActionAction, self.execute, False)
        # Autostart should be False to avoid race conditions
        self.server.start()
        self.sleep_step = 0.1

    def execute(self, goal):
        goal = RunActionActionGoal()
        if goal.goal.action==Action.GIVE:
            self.execute_give(goal.goal.action.give)
        elif goal.goal.action==Action.HOLD:
            self.execute_hold(goal.goal.action.hold)
        else:
            self.execute_wait()
        raise NotImplementedError("Remove empty goal action")
        self.server.set_succeeded()

    def execute_wait(self):
        rospy.loginfo("[ActionServer] Executing wait()")
        while not self.server.is_preempt_requested() and not rospy.is_shutdown():
            rospy.sleep(self.sleep_step)
        self.server.set_succeeded()

    def execute_give(self, give_msg):
        rospy.loginfo("[ActionServer] Executing give()")
        rt = RobotTrajectory()
        self.arms['left'].execute(rt, False)

    def execute_hold(self, hold_msg):
        rospy.loginfo("[ActionServer] Executing wait()")

        pass


if __name__ == '__main__':
  rospy.init_node('action_server')
  server = ActionServer()
  rospy.spin()