#! /usr/bin/env python
import rospy
import rospkg
import json
import tf
import time
import actionlib
import moveit_commander
import move_group_extras
import transformations
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
import numpy
import baxter_interface  # TODO Use a generic stuff instead for gripper closure (Moveit?)

from thr_coop_assembly.msg import RunActionAction, Action, RunActionActionResult

class ActionServer:
    """
    This is the action server that executes actions the robot is capable of.
    It requires two config files:
    * poses.json: poses relative to actions and objects
    * action_params.json: generic parameters for action execution and scenario
    """
    def __init__(self, planning, orientation_matters, allow_replanning):
        """
        :param planning: True if motion should be planned with collision avoidance (where possible), False means joints interpolation
        """
        # General attributes
        self.rospack = rospkg.RosPack()
        self.planning = planning
        self.orientation_matters = orientation_matters
        self.replan = allow_replanning

        # Transform/Geometric attributes
        self.tfl = tf.TransformListener()
        self.world = "base"
        rospy.loginfo("Loading config...")
        with open(self.rospack.get_path("thr_coop_assembly")+"/config/poses.json") as f:
            self.poses = json.load(f)
        with open(self.rospack.get_path("thr_coop_assembly")+"/config/action_params.json") as f:
            self.action_params = json.load(f)

        # Workaround before fully using MoveGroup [to be cleaned and replaced by MoveGroup.execute()]
        rospy.loginfo("Connecting to direct Limb control [TODO to be cleaned]...")
        self.clients = {}
        self.clients['left'] = actionlib.SimpleActionClient("/robot/limb/left/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.clients['right'] = actionlib.SimpleActionClient("/robot/limb/right/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.clients['left'].wait_for_server()
        self.clients['right'].wait_for_server()

        # Motion/MoveGroup/Grasping attributes
        rospy.loginfo("Connecting to MoveIt...")
        self.arms = {}
        self.arms['left'] = moveit_commander.MoveGroupCommander("left_arm")
        self.arms['right'] = moveit_commander.MoveGroupCommander("right_arm")
        self.extras = {}
        self.extras['left'] = move_group_extras.MoveGroupExtras(self.arms['left'])
        self.extras['right'] = move_group_extras.MoveGroupExtras(self.arms['right'])
        self.grippers = {}
        self.grippers['left'] = baxter_interface.gripper.Gripper('left')
        self.grippers['right'] = baxter_interface.gripper.Gripper('right')
        self.arms['left'].set_planner_id(str(self.action_params['planning']['planner_id']))
        self.arms['left'].set_planning_time(self.action_params['planning']['time'])
        self.arms['right'].set_planner_id(str(self.action_params['planning']['planner_id']))
        self.arms['right'].set_planning_time(self.action_params['planning']['time'])
        self.arms['left'].allow_replanning(self.replan)
        self.arms['right'].allow_replanning(self.replan)
        self.scene = moveit_commander.PlanningSceneInterface()

        # Action server attributes
        rospy.loginfo("Starting server...")
        self.server = actionlib.SimpleActionServer('/thr/run_action', RunActionAction, self.execute, False)
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

    def low_level_execute_workaround(self, side, rt, callback_stop=None):
        """
        Since moveit_commander.execute() either blocks threads or does not give feedback on trajectory, this is an
        ugly workaround to send a RobotTrajectory() to a FollowJointTrajectory action client. It comes with the
        :param side: 'right' or 'left'
        :param rt: The RobotTrajectory to execute
        :param callback_stop: the function to call at each step returning True if motion stop has been requested
        :return: True if the full motion has been executed, False if it has been stopped
        """
        ftg = FollowJointTrajectoryGoal()
        ftg.trajectory = rt.joint_trajectory
        self.clients[side].send_goal(ftg)
        stop = False
        while not stop and self.clients[side].simple_state != actionlib.SimpleGoalState.DONE:
            if callback_stop!=None and callback_stop():
                stop = True
            else:
                rospy.sleep(self.action_params['sleep_step'])
        if stop and self.clients[side].simple_state != actionlib.SimpleGoalState.DONE:
            self.clients[side].cancel_goal()
        return not stop

    def extract_perturbation(self, side):
        """
        Sleeps a sleep step and extracts the difference between command state and current state
        CAUTION: Baxter-only method, /reference/* TFs don't exist elsewhere and will trigger LookupExceptions
        :param side: left or right
        :return: The cartesian distance in meters between command and current
        """
        diffs = []
        window = 50
        for i in range(window):
            diffs.append(transformations.distance(self.tfl.lookupTransform(self.world, side+'_gripper', rospy.Time(0)),
                                                  self.tfl.lookupTransform('/reference/'+self.world, '/reference/'+side+'_gripper', rospy.Time(0))))
            rospy.sleep(self.action_params['sleep_step']/window)
        print "max", numpy.max(diffs)
        return numpy.max(diffs)

    def execute_give(self, parameters):
        # Parameters could be "/thr/handle", it asks the robot to give the handle using the "give" pose (only 1 per object atm)

        rospy.loginfo("[ActionServer] Executing give{}".format(str(parameters)))
        object = parameters[0]

        # 0. Trajectories generation
        try:
            world_approach_pose = self.object_grasp_pose_to_world(self.poses[object]["give"][0]['approach'], object)  # Pose of the approach
        except:
            rospy.logerr("Object {} not found".format(object))
            return self.set_motion_ended(False)
        goal_approach = self.extras['left'].get_ik(world_approach_pose)
        if not goal_approach:
            rospy.logerr("Unable to reach approach pose")
            return self.set_motion_ended(False)
        approach_traj = self.extras['left'].interpolate_joint_space(goal_approach, self.action_params['action_num_points'], kv_max=0.8, ka_max=0.8)

        # 1. Go to approach pose
        if self.should_interrupt():
            return self.set_motion_ended(False)
        rospy.loginfo("Approaching {}".format(object))
        #self.arms['left'].execute(approach_traj, False)
        self.low_level_execute_workaround('left', approach_traj)

        # 2. Go to "give" pose
        if self.should_interrupt():
            return self.set_motion_ended(False)
        rospy.loginfo("Grasping {}".format(object))
        #self.arms['left'].execute(action_traj)
        action_traj = self.extras['left'].generate_descent(self.poses[object]["give"][0]['descent'], object, 1.5)
        self.low_level_execute_workaround('left', action_traj)

        # 3. Close gripper to grasp object
        give_traj = None
        if not self.should_interrupt():
            rospy.loginfo("Activating suction for {}".format(object))
            self.grippers['left'].close(True)
            self.scene.attach_box('left_gripper', object)

        # 4. Go to approach pose again with object in-hand (to avoid touching the table)
        if self.should_interrupt():
            return self.set_motion_ended(False)
        rospy.loginfo("Reapproaching {}".format(object))
        #self.arms['left'].execute(approach_traj, False)
        reapproach_traj = self.extras['left'].interpolate_joint_space(goal_approach, self.action_params['action_num_points'], kv_max=0.8, ka_max=0.8)
        self.low_level_execute_workaround('left', reapproach_traj)


        # 4. Wait for human wrist and approach object until we are close enough to release object
        rospy.loginfo("Bringing {} to human wrist".format(object))

        while not self.should_interrupt():
            can_release = False
            try:
                distance_wrist_gripper = transformations.norm(self.tfl.lookupTransform("left_gripper", "/human/wrist", rospy.Time(0)))
            except:
                rospy.logwarn("Human wrist not found")
                rospy.sleep(self.action_params['sleep_step'])
                continue
            rospy.loginfo("User wrist at {}m from gripper, threshold {}m".format(distance_wrist_gripper, self.action_params['give']['sphere_radius']))
            if distance_wrist_gripper > self.action_params['give']['sphere_radius']:
                world_give_pose = self.object_grasp_pose_to_world(self.action_params['give']['give_pose'], "/human/wrist")
                if self.planning:
                    if self.orientation_matters:
                        self.arms['left'].set_pose_target(world_give_pose.pose)
                    else:
                        self.arms['left'].set_position_target([world_give_pose.pose.position.x, world_give_pose.pose.position.y, world_give_pose.pose.position.z])
                    give_traj = self.arms['left'].plan()
                    self.arms['left'].clear_pose_targets()
                    if len(give_traj.joint_trajectory.points)<2:
                        rospy.logwarn("Unable to plan to that pose, please move a little bit...")
                        continue
                else:
                    goal_give = self.extras['left'].get_ik(world_give_pose)
                    if not goal_give:
                        rospy.logwarn("Human wrist found but not reachable, please move it a little bit...")
                        rospy.sleep(self.action_params['sleep_step'])
                        continue
                    give_traj = self.extras['left'].interpolate_joint_space(goal_give, self.action_params['action_num_points'], kv_max=0.8, ka_max=0.8)
                #self.arms['left'].execute(give_traj)

                # The function below returns True if human as moved enough so that we need to replan the trajectory
                def needs_update():
                    p_wrist = transformations.list_to_pose(self.tfl.lookupTransform(self.world, "/human/wrist", rospy.Time(0)))
                    return transformations.distance(world_give_pose, p_wrist)>self.action_params['give']['sphere_radius']
                if not self.low_level_execute_workaround('left', give_traj, needs_update):
                    rospy.logwarn("Human has moved, changing the goal...")
                    rospy.sleep(self.action_params['give']['inter_goal_sleep'])
                    continue
            else:
                can_release = True

            # 5. Wait for position disturbance of the gripper and open it to release object
            if can_release:
                perturbation = self.extract_perturbation('left')
                rospy.loginfo("Perurbation: {}m, threshold: {}m".format(perturbation, self.action_params['give']['releasing_disturbance']))
                if perturbation>self.action_params['give']['releasing_disturbance']:
                    rospy.loginfo("Releasing suction for {}".format(object))
                    self.grippers['left'].open(True)
                    self.scene.remove_attached_object('left_gripper')
                    break
            else:
                rospy.sleep(self.action_params['sleep_step'])

        # 4. Return to pick approach pose in order not to bother human
        if not self.should_interrupt():
            rospy.loginfo("Going back to picking area")
            if self.planning:
                while True:
                    ending_traj = self.arms['left'].plan(world_approach_pose.pose)
                    if len(approach_traj.joint_trajectory.points)>1: break
            else:
                ending_traj = self.extras['left'].interpolate_joint_space(goal_approach, self.action_params['action_num_points'], kv_max=0.8, ka_max=0.8)
            self.low_level_execute_workaround('left', ending_traj)

        rospy.loginfo("[ActionServer] Executed give{} with {}".format(str(parameters), "failure" if self.should_interrupt() else "success"))
        return self.set_motion_ended(not self.should_interrupt())


    def execute_hold(self, parameters):
        # Parameters could be "/thr/handle 0", it asks the robot to hold the handle using its first hold pose
        rospy.loginfo("[ActionServer] Executing hold{}".format(str(parameters)))
        object = parameters[0]
        pose = int(parameters[1])

        # 0. Trajectories generation
        starting_state = self.extras['right'].get_current_state()
        starting_pose = transformations.list_to_pose(self.tfl.lookupTransform(self.world, "right_gripper", rospy.Time(0)))
        try:
            world_approach_pose = self.object_grasp_pose_to_world(self.poses[object]["hold"][pose]['approach'], object)  # Pose of the approach
            #world_action_pose = self.object_grasp_pose_to_world(self.poses[object]["hold"][pose]['action'], object)  # Pose of the pickup (named action)
        except:
            rospy.logerr("Object {} not found".format(object))
            return self.set_motion_ended(False)

        # Comment: goal approach is needed in both interpolate and planning mode (planning = for reapproach)
        goal_approach = self.extras['right'].get_ik(world_approach_pose)
        if not goal_approach:
            rospy.logerr("Unable to reach approach pose")
            return self.set_motion_ended(False)

        if self.planning:
            approach_traj = self.arms['right'].plan(world_approach_pose.pose)
            if len(approach_traj.joint_trajectory.points)<2:
                rospy.logerr("Sorry, object {} is too far away for me".format(object))
                return self.set_motion_ended(False)
        else:
            approach_traj = self.extras['right'].interpolate_joint_space(goal_approach, self.action_params['action_num_points'], kv_max=0.5, ka_max=0.5)

        # 1. Go to approach pose
        if self.should_interrupt():
            return self.set_motion_ended(False)
        rospy.loginfo("Approaching {}".format(object))
        #self.arms['right'].execute(approach_traj, False)
        self.low_level_execute_workaround('right', approach_traj)

        # 2. Go to "hold" pose
        if self.should_interrupt():
            return self.set_motion_ended(False)
        rospy.loginfo("Grasping {}".format(object))
        #self.arms['right'].execute(action_traj)
        action_traj = self.extras['right'].generate_descent(self.poses[object]["hold"][pose]['descent'], object, 3)
        self.low_level_execute_workaround('right', action_traj)

        # 3. Close gripper to grasp object
        if not self.should_interrupt():
            rospy.loginfo("Closing gripper around {}".format(object))
            self.grippers['right'].close(True)

        # 4. Force down
        if self.should_interrupt():
            return self.set_motion_ended(False)
        rospy.loginfo("Forcing down on {}".format(object))
        #self.arms['right'].execute(action_traj)
        force_traj = self.extras['right'].generate_descent(self.poses[object]["hold"][pose]['force'], object, 1)
        self.low_level_execute_workaround('right', force_traj)

        # 5. Wait for interruption
        last_seen_working = -1 # timestamp storing when the human has been last seen in the working area, -1 = never seen
        while not self.should_interrupt():
            try:
                distance_wrist_gripper = transformations.norm(self.tfl.lookupTransform("right_gripper", "/human/wrist", rospy.Time(0)))
            except:
                rospy.logwarn("Human wrist not found")
                distance_wrist_gripper = float('inf')
            if distance_wrist_gripper < self.action_params['hold']['sphere_radius']:
                last_seen_working = time.time()
                rospy.loginfo("Human is currently working with {}... Move your hands away to stop, distance {}m, threshold {}m".format(object, distance_wrist_gripper, self.action_params['hold']['sphere_radius']))
            else:
                break
            rospy.sleep(self.action_params['sleep_step'])

        # 6. Release object
        rospy.loginfo("Human has stopped working with {}, now releasing".format(object))
        if not self.should_interrupt():
            rospy.loginfo("Releasing {}".format(object))
            self.grippers['right'].open(True)

        # Generation of next trajectories starting from the current state
        reapproach_traj = self.extras['right'].interpolate_joint_space(goal_approach, self.action_params['action_num_points'], kv_max=0.5, ka_max=0.5)

        # 7. Go to approach pose again (to avoid touching the fingers)
        if self.should_interrupt():
            return self.set_motion_ended(False)
        rospy.loginfo("Reapproaching {}".format(object))
        #self.arms['right'].execute(approach_traj, False)
        self.low_level_execute_workaround('right', reapproach_traj)

        # 8. Return home
        if self.should_interrupt():
            return self.set_motion_ended(False)
        rospy.loginfo("Returning in idle mode")
        if self.planning:
            while True:
                leaving_traj = self.arms['right'].plan(starting_pose.pose)
                if len(approach_traj.joint_trajectory.points)>1: break
        else:
            leaving_traj = self.extras['right'].interpolate_joint_space(starting_state, self.action_params['action_num_points'], kv_max=0.5, ka_max=0.5, start=goal_approach)
        self.low_level_execute_workaround('right', leaving_traj)

        rospy.loginfo("[ActionServer] Executed hold{} with {}".format(str(parameters), "failure" if self.should_interrupt() else "success"))
        return self.set_motion_ended(not self.should_interrupt())

if __name__ == '__main__':
    rospy.init_node('action_server')
    server = ActionServer(planning=True, orientation_matters=True, allow_replanning=True)
    rospy.spin()