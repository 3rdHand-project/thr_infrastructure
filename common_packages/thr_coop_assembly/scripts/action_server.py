#! /usr/bin/env python

import rospy
import rospkg
import json
import tf
import actionlib
import moveit_commander
import move_group_extras
import transformations

import baxter_interface  # TODO Use a generic stuff instead for gripper closure (Moveit?)

from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.srv import GetPositionIK
from thr_coop_assembly.msg import RunActionAction, Action

class ActionServer:
    def __init__(self):
        # General attributes
        self.rospack = rospkg.RosPack()

        # Transform/Geometric attributes
        self.tfl = tf.TransformListener()
        self.world = "base"
        with open(self.rospack.get_path("thr_coop_assembly")+"/config/poses.json") as f:
            self.poses = json.load(f)
        with open(self.rospack.get_path("thr_coop_assembly")+"/config/action_params.json") as f:
            self.action_params = json.load(f)

        # Motion/MoveGroup/Grasping attributes
        self.arms = {}
        self.arms['left'] = moveit_commander.MoveGroupCommander("left_arm")
        self.arms['right'] = moveit_commander.MoveGroupCommander("right_arm")
        self.extras = {}
        self.extras['left'] = move_group_extras.MoveGroupExtras(self.arms['left'])
        self.extras['right'] = move_group_extras.MoveGroupExtras(self.arms['right'])
        self.grippers = {}
        self.grippers['left'] = baxter_interface.gripper.Gripper('left')
        self.grippers['right'] = baxter_interface.gripper.Gripper('right')

        # Action server attributes
        self.server = actionlib.SimpleActionServer('/thr/action', RunActionAction, self.execute, False)
        self.server.start()
        rospy.loginfo('Server ready')

    def execute(self, goal):
        if goal.action.type==Action.GIVE:
            self.execute_give(goal.action.parameters)
        elif goal.action.type==Action.HOLD:
            self.execute_hold(goal.action.parameters)
        else:
            self.execute_wait()
        self.server.set_succeeded()

    def execute_wait(self):
        rospy.loginfo("[ActionServer] Executing wait()")
        while not self.server.is_preempt_requested() and not rospy.is_shutdown():
            rospy.sleep(self.action_params['sleep_step'])
        self.server.set_succeeded()

    def object_grasp_pose_to_world(self, poselist, object):
        obj_pose = transformations.list_to_pose(poselist, object)
        world_pose = self.tfl.transformPose(self.world, obj_pose, rospy.Time(0))
        return world_pose

    def should_interrupt(self):
        return rospy.is_shutdown() or self.server.is_preempt_requested()

    def execute_give(self, parameters):
        # Parameters could be "/thr/handle", it asks the robot to give the handle using the "give" pose (only 1 per object atm)

        rospy.loginfo("[ActionServer] Executing give{}", str(parameters))
        object = parameters[0]

        # 0. Trajectories generation
        world_approach_pose = self.object_grasp_pose_to_world(self.poses[object]["give"][0]['approach'], object)
        world_action_pose = self.object_grasp_pose_to_world(self.poses[object]["give"][0]['action'], object)
        goal_approach = self.extras['left'].get_ik(world_approach_pose)
        goal_action = self.extras['left'].get_ik(world_action_pose, goal_approach)
        if not goal_approach:
            rospy.logerr("Unable to reach approach pose")
            return False
        if not goal_action:
            rospy.logerr("Unable to reach give pose")
            return False
        approach_traj = self.extras['left'].interpolate_joint_space(goal_approach, self.action_params['action_duration'], self.action_params['action_num_points'])
        action_traj = self.extras['left'].interpolate_joint_space(goal_action, self.action_params['action_duration'], self.action_params['action_num_points'], goal_approach)

        # 1. Go to approach pose
        if self.should_interrupt() or not self.arms['left'].execute(approach_traj):
            return False

        # 2. Go to "give" pose
        if self.should_interrupt() or not self.arms['left'].execute(action_traj):
            return  False

        # 3. Close gripper to grasp object
        give_traj = None
        if not self.should_interrupt():
            self.grippers['left'].close(True)

        # 4. Wait for human wrist and approach object until we are close enough to release object
        while not self.should_interrupt():
            try:
                wrist = self.tfl.lookupTransform(self.world, "/human/wrist", rospy.Time(0))
            except LookupError:
                rospy.logwarn("Human wrist not found")
                rospy.sleep(self.action_params['sleep_step'])
                continue
            gripper = self.tfl.lookupTransform(self.world, "left_gripper", rospy.Time(0))
            distance_wrist_gripper = transformations.distance(wrist, gripper)
            rospy.loginfo("User wrist at {}m from gripper, limit {}m", distance_wrist_gripper, self.action_params['give']['sphere_radius'])
            if distance_wrist_gripper > self.action_params['give']['sphere_radius']:
                world_give_pose = self.object_grasp_pose_to_world(self.action_params['give']['give_pose'], "/human/wrist")
                goal_give = self.extras['left'].get_ik(world_give_pose)
                give_traj = self.extras['left'].interpolate_joint_space(goal_give, self.action_params['action_duration'], self.action_params['action_num_points'])
                if not self.arms['left'].execute(give_traj):
                    return False
            else:
                break

        # 3. Open gripper to release object
        if not self.should_interrupt():
            self.grippers['left'].open(True)

        # TODO True=>False with asynchronous motion

    def execute_hold(self, parameters):
        # Parameters could be "/thr/handle 0", it asks the robot to hold the handle using its first hold pose
        rospy.loginfo("[ActionServer] Executing hold{}", str(parameters))
        object = parameters[0]
        pose = parameters[1]
        pass


if __name__ == '__main__':
  rospy.init_node('action_server')
  server = ActionServer()
  rospy.spin()