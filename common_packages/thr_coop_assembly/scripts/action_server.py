#! /usr/bin/env python

import rospy
import rospkg
import json
import tf
import actionlib
import moveit_commander
import move_group_extras
import transformations
from moveit_msgs.msg import RobotState
import baxter_interface  # TODO Use a generic stuff instead for gripper closure (Moveit?)

from thr_coop_assembly.msg import RunActionAction, Action, RunActionActionResult

class ActionServer:
    def __init__(self):
        # General attributes
        rospy.loginfo("Starting rospack...")
        self.rospack = rospkg.RosPack()

        # Transform/Geometric attributes
        self.tfl = tf.TransformListener()
        self.world = "base"
        rospy.loginfo("Loading config...")
        with open(self.rospack.get_path("thr_coop_assembly")+"/config/poses.json") as f:
            self.poses = json.load(f)
        with open(self.rospack.get_path("thr_coop_assembly")+"/config/action_params.json") as f:
            self.action_params = json.load(f)

        # Motion/MoveGroup/Grasping attributes
        rospy.loginfo("Connecting to robot...")
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
        rospy.loginfo("Starting server...")
        self.server = actionlib.SimpleActionServer('/thr/action', RunActionAction, self.execute, False)
        self.result = RunActionActionResult()
        self.server.start()
        rospy.loginfo('Server ready')

    def execute(self, goal):
        """
        Dispatches a new goal on the method executing each type of action
        :param goal:
        """
        if goal.action.type==Action.GIVE:
            self.execute_give(goal.action.parameters)
        elif goal.action.type==Action.HOLD:
            self.execute_hold(goal.action.parameters)
        else:
            self.execute_wait()

    def execute_wait(self):
        """
        Executes a WAIT action (always successful)
        """
        rospy.loginfo("[ActionServer] Executing wait()")
        while not self.server.is_preempt_requested() and not rospy.is_shutdown():
            rospy.sleep(self.action_params['sleep_step'])
        self.set_motion_ended(True)

    def object_grasp_pose_to_world(self, poselist, object):
        """
        Returns the transformation from world to the point defined by "poselist attached to object frame"
        :param poselist: A poselist [[x, y, z], [x, y, z, w]]
        :param object: The frame where the pose should be attached
        :return: the transformation from world to the point defined by "poselist attached to object frame"
        """
        obj_pose = transformations.list_to_pose(poselist, object)
        world_pose = self.tfl.transformPose(self.world, obj_pose)
        return world_pose

    def should_interrupt(self):
        """
        :return: True if motion should interrupts at that time for whatever reason
        """
        return rospy.is_shutdown() or self.server.is_preempt_requested()

    def set_motion_ended(self, succeeded):
        self.result.header.stamp = rospy.Time.now()
        self.result.result.succeeded = succeeded
        self.server.set_succeeded(self.result.result)
        return succeeded

    def action_duration(self, start_pose, goal_pose):
        """
        Outputs the duration to go from start_state to goal_state based on the speed in config files.
        If no start_pose is provided the current pose is used as start pose
        :param start_pose: A PoseStamped() defining start pose or the name of the tf
        :param goal_pose: A PoseStamped() defining goal pose or the name of the tf
        :return: duration to go from start_pose to goal_pose in seconds (float)
        """
        speed = self.action_params['action_speed']  # Speed of action in m/s
        if isinstance(start_pose, str):
            start_pose = transformations.list_to_pose(self.tfl.lookupTransform(self.world, start_pose, rospy.Time(0)), self.world)
        if isinstance(goal_pose, str):
            goal_pose = transformations.list_to_pose(self.tfl.lookupTransform(self.world, goal_pose, rospy.Time(0)), self.world)
        return transformations.distance(start_pose, goal_pose)/float(speed)


    def execute_give(self, parameters):
        # Parameters could be "/thr/handle", it asks the robot to give the handle using the "give" pose (only 1 per object atm)

        rospy.loginfo("[ActionServer] Executing give{}".format(str(parameters)))
        object = parameters[0]

        # 0. Trajectories generation
        try:
            world_approach_pose = self.object_grasp_pose_to_world(self.poses[object]["give"][0]['approach'], object)  # Pose of the approach
            world_action_pose = self.object_grasp_pose_to_world(self.poses[object]["give"][0]['action'], object)  # Pose of the pickup (named action)
        except:
            rospy.logerr("Object {} not found".format(object))
            return self.set_motion_ended(False)
        goal_approach = self.extras['left'].get_ik(world_approach_pose)
        goal_action = self.extras['left'].get_ik(world_action_pose, goal_approach)
        if not goal_approach:
            rospy.logerr("Unable to reach approach pose")
            return self.set_motion_ended(False)
        if not goal_action:
            rospy.logerr("Unable to reach give pose")
            return self.set_motion_ended(False)
        approach_traj = self.extras['left'].interpolate_joint_space(goal_approach, self.action_duration('left_gripper', world_approach_pose), self.action_params['action_num_points'])
        action_traj = self.extras['left'].interpolate_joint_space(goal_action, self.action_duration(world_approach_pose, world_action_pose), self.action_params['action_num_points'], goal_approach)

        # 1. Go to approach pose
        if self.should_interrupt():
            return self.set_motion_ended(False)
        print "START"
        self.arms['left'].execute(approach_traj)
        print "END"

        # 2. Go to "give" pose
        if self.should_interrupt():
            return self.set_motion_ended(False)
        self.arms['left'].execute(action_traj)

        # 3. Close gripper to grasp object
        give_traj = None
        if not self.should_interrupt():
            self.grippers['left'].close(True)

        # 4. Wait for human wrist and approach object until we are close enough to release object
        while not self.should_interrupt():
            try:
                wrist = self.tfl.lookupTransform(self.world, "/human/wrist", rospy.Time(0))
            except:
                rospy.logwarn("Human wrist not found")
                rospy.sleep(self.action_params['sleep_step'])
                continue
            gripper = self.tfl.lookupTransform(self.world, "left_gripper", rospy.Time(0))
            distance_wrist_gripper = transformations.distance(wrist, gripper)
            rospy.loginfo("User wrist at {}m from gripper, limit {}m".format(distance_wrist_gripper, self.action_params['give']['sphere_radius']))
            if distance_wrist_gripper > self.action_params['give']['sphere_radius']:
                world_give_pose = self.object_grasp_pose_to_world(self.action_params['give']['give_pose'], "/human/wrist")
                goal_give = self.extras['left'].get_ik(world_give_pose)
                if not goal_give:
                    rospy.logwarn("Human wrist found but not reachable, please move it a little bit...")
                    rospy.sleep(self.action_params['sleep_step'])
                    continue
                give_traj = self.extras['left'].interpolate_joint_space(goal_give, self.action_duration('left_gripper', world_give_pose), self.action_params['action_num_points'])
                self.arms['left'].execute(give_traj)
            else:
                break

        # 3. Open gripper to release object
        if not self.should_interrupt():
            self.grippers['left'].open(True)

        # TODO execute True=>False with asynchronous motion
        return self.set_motion_ended(not self.should_interrupt())

    def execute_hold(self, parameters):
        # Parameters could be "/thr/handle 0", it asks the robot to hold the handle using its first hold pose
        rospy.loginfo("[ActionServer] Executing hold{}".format(str(parameters)))
        object = parameters[0]
        pose = parameters[1]
        pass


if __name__ == '__main__':
  rospy.init_node('action_server')
  server = ActionServer()
  rospy.spin()