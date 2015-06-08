#! /usr/bin/env python
import rospy
import rospkg
import json
import tf
import sys
import actionlib
import moveit_commander
import move_group_extras
import transformations
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
import numpy
import baxter_interface  # TODO Use a generic stuff instead for gripper closure (Moveit?)

from thr_coop_assembly.msg import RunRobotActionAction, RunRobotActionActionResult

class RobotActionServer:
    """
    This is the action server that executes actions the robot is capable of.
    It requires two config files:
    * poses.json: poses relative to actions and objects
    * action_params.json: generic parameters for action execution and scenario
    """
    def __init__(self, side, planning, orientation_matters, allow_replanning):
        """
        :param planning: True if motion should be planned with collision avoidance (where possible), False means joints interpolation
        """
        # General attributes
        self.rospack = rospkg.RosPack()
        self.planning = planning
        self.orientation_matters = orientation_matters
        self.replan = allow_replanning
        self.side = side
        self.arm_name = side+'_arm'
        self.gripper_name = side+'_gripper'

        # Transform/Geometric attributes
        self.tfl = tf.TransformListener(True, rospy.Duration(5*60)) # TF Interpolation ON and duration of its cache = 5 minutes
        self.world = "base"
        rospy.loginfo("Loading config...")
        with open(self.rospack.get_path("thr_coop_assembly")+"/config/poses.json") as f:
            self.poses = json.load(f)
        with open(self.rospack.get_path("thr_coop_assembly")+"/config/action_params.json") as f:
            self.action_params = json.load(f)

        # Workaround before fully using MoveGroup [to be cleaned and replaced by MoveGroup.execute()]
        rospy.loginfo("Connecting to direct Limb control [TODO to be cleaned]...")
        self.client = actionlib.SimpleActionClient("/robot/limb/{}/follow_joint_trajectory".format(self.side), FollowJointTrajectoryAction)

        # Motion/MoveGroup/Grasping attributes
        rospy.sleep(3) # Moveit should have started first
        rospy.loginfo("Connecting to MoveIt...")
        self.arm = moveit_commander.MoveGroupCommander(self.arm_name)
        self.extras = move_group_extras.MoveGroupExtras(self.arm)
        self.gripper = baxter_interface.gripper.Gripper(self.side)
        self.arm.set_planner_id(str(self.action_params['planning']['planner_id']))
        self.arm.set_planning_time(self.action_params['planning']['time'])
        self.arm.allow_replanning(self.replan)
        self.scene = moveit_commander.PlanningSceneInterface()

        # Home poses are taken when the server starts:
        self.starting_state = self.extras.get_current_state()
        self.tfl.waitForTransform(self.world, self.gripper_name, rospy.Time(0), rospy.Duration(10))
        self.starting_pose = transformations.list_to_pose(self.tfl.lookupTransform(self.world, self.gripper_name, rospy.Time(0)))

        # Action server attributes
        rospy.loginfo("Starting server "+side)
        self.server = actionlib.SimpleActionServer('/thr/robot_run_action/'+side, RunRobotActionAction, self.execute, False)
        self.result = RunRobotActionActionResult()
        self.server.start()
        rospy.loginfo(side+' server ready')

    def execute(self, goal):
        """
        Dispatches a new goal on the method executing each type of action
        :param goal:
        """
        if goal.action.type=='give' and self.side=='left':
            self.execute_give(goal.action.parameters)
        elif goal.action.type=='hold' and self.side=='right':
            self.execute_hold(goal.action.parameters)
        elif goal.action.type=='pick' and self.side=='left':
            self.execute_pick(goal.action.parameters)
        elif goal.action.type == 'go_home_'+self.side :
            self.execute_go_home(goal.action.parameters)
        else:
            rospy.logwarn('{} arm is not capable of action {}'.format(self.side, goal.action.type))
            self.server.set_aborted()
        # Note: do not try/catch: an Exception ends up in an aborted state, ... perfect!

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

    def low_level_execute_workaround(self, side, rt, callback_stop=None):
        """
        Since moveit_commander.execute() either blocks threads or does not give feedback on trajectory, this is an
        ugly workaround to send a RobotTrajectory() to a FollowJointTrajectory action client. It comes with the
        :param side: self.side or self.side
        :param rt: The RobotTrajectory to execute
        :param callback_stop: the function to call at each step returning True if motion stop has been requested
        :return: True if the full motion has been executed, False if it has been stopped
        """
        ftg = FollowJointTrajectoryGoal()
        ftg.trajectory = rt.joint_trajectory
        self.client.send_goal(ftg)
        stop = False
        while not stop and self.client.simple_state != actionlib.SimpleGoalState.DONE:
            if callback_stop!=None and callback_stop():
                stop = True
            else:
                rospy.sleep(self.action_params['sleep_step'])
        if stop and self.client.simple_state != actionlib.SimpleGoalState.DONE:
            self.client.cancel_goal()
        return not stop

    def extract_perturbation(self):
        """
        Sleeps a sleep step and extracts the difference between command state and current state
        CAUTION: Baxter-only method, /reference/* TFs don't exist elsewhere and will trigger LookupExceptions
        :return: The cartesian distance in meters between command and current
        """
        diffs = []
        window = 50
        for i in range(window):
            diffs.append(transformations.distance(self.tfl.lookupTransform(self.world, self.gripper_name, rospy.Time(0)),
                                                  self.tfl.lookupTransform('/reference/'+self.world, '/reference/'+self.gripper_name, rospy.Time(0))))
            rospy.sleep(self.action_params['sleep_step']/window)
        return numpy.max(diffs)

    def execute_pick(self, parameters):
        rospy.loginfo("[ActionServer] Executing pick{}".format(str(parameters)))
        object = parameters[0]

        # 0. Trajectories generation
        try:
            world_approach_pose = self.object_grasp_pose_to_world(self.poses[object]["give"][0]['approach'], object)  # Pose of the approach
        except:
            rospy.logerr("Object {} not found".format(object))
            return self.server.set_aborted()
        goal_approach = self.extras.get_ik(world_approach_pose)
        if not goal_approach:
            rospy.logerr("Unable to reach approach pose")
            return self.server.set_aborted()
        approach_traj = self.extras.interpolate_joint_space(goal_approach, self.action_params['action_num_points'], kv_max=self.action_params['kv_max'], ka_max=self.action_params['ka_max'])

        # 1. Go to approach pose
        if self.should_interrupt():
            return self.server.set_aborted()
        rospy.loginfo("Approaching {}".format(object))
        self.low_level_execute_workaround(self.side, approach_traj)

        # 2. Go to "give" pose
        if self.should_interrupt():
            return self.server.set_aborted()
        rospy.loginfo("Grasping {}".format(object))
        action_traj = self.extras.generate_descent(self.poses[object]["give"][0]['descent'], object, 1.5)
        self.low_level_execute_workaround(self.side, action_traj)

        # 3. Close gripper to grasp object
        if not self.should_interrupt():
            rospy.loginfo("Activating suction for {}".format(object))
            self.gripper.close(True)
            self.scene.attach_box(self.gripper_name, object)

        # 4. Go to approach pose again with object in-hand (to avoid touching the table)
        if self.should_interrupt():
            return self.server.set_aborted()
        rospy.loginfo("Reapproaching {}".format(object))
        reapproach_traj = self.extras.reverse_trajectory(action_traj)
        self.low_level_execute_workaround(self.side, reapproach_traj)

        rospy.loginfo("[ActionServer] Executed pick{} with {}".format(str(parameters), "failure" if self.should_interrupt() else "success"))
        return

    def execute_give(self, parameters):
        # Parameters could be "/thr/handle", it asks the robot to give the handle using the "give" pose (only 1 per object atm)
        rospy.loginfo("[ActionServer] Executing give{}".format(str(parameters)))
        object = parameters[0]

        # 4. Wait for human wrist and approach object until we are close enough to release object
        rospy.loginfo("Bringing {} to human wrist".format(object))
        while not self.should_interrupt():
            can_release = False
            try:
                distance_wrist_gripper = transformations.norm(self.tfl.lookupTransform(self.gripper_name, "/human/wrist", rospy.Time(0)))
            except:
                rospy.logwarn("Human wrist not found")
                rospy.sleep(self.action_params['sleep_step'])
                continue
            rospy.loginfo("User wrist at {}m from gripper, threshold {}m".format(distance_wrist_gripper, self.action_params['give']['sphere_radius']))
            if distance_wrist_gripper > self.action_params['give']['sphere_radius']:
                world_give_pose = self.object_grasp_pose_to_world(self.action_params['give']['give_pose'], "/human/wrist")
                if self.planning:
                    if self.orientation_matters:
                        self.arm.set_pose_target(world_give_pose.pose)
                    else:
                        self.arm.set_position_target([world_give_pose.pose.position.x, world_give_pose.pose.position.y, world_give_pose.pose.position.z])
                    give_traj = self.arm.plan()
                    self.arm.clear_pose_targets()
                    if len(give_traj.joint_trajectory.points)<2:
                        rospy.logwarn("Unable to plan to that pose, please move a little bit...")
                        rospy.sleep(self.action_params['sleep_step']) # TODO Moveit bug, tfl is not updated during planning
                        continue
                else:
                    goal_give = self.extras.get_ik(world_give_pose)
                    if not goal_give:
                        rospy.logwarn("Human wrist found but not reachable, please move it a little bit...")
                        rospy.sleep(self.action_params['sleep_step'])
                        continue
                    give_traj = self.extras.interpolate_joint_space(goal_give, self.action_params['action_num_points'], kv_max=self.action_params['kv_max'], ka_max=self.action_params['ka_max'])

                # The function below returns True if human as moved enough so that we need to replan the trajectory
                def needs_update():
                    p_wrist = transformations.list_to_pose(self.tfl.lookupTransform(self.world, "/human/wrist", rospy.Time(0)))
                    return transformations.distance(world_give_pose, p_wrist)>self.action_params['give']['sphere_radius']
                if not self.low_level_execute_workaround(self.side, give_traj, needs_update):
                    rospy.logwarn("Human has moved, changing the goal...")
                    rospy.sleep(self.action_params['give']['inter_goal_sleep'])
                    continue
            else:
                can_release = True

            # 5. Wait for position disturbance of the gripper and open it to release object
            if can_release:
                perturbation = self.extract_perturbation()
                rospy.loginfo("Perurbation: {}m, threshold: {}m".format(perturbation, self.action_params['give']['releasing_disturbance']))
                if perturbation>self.action_params['give']['releasing_disturbance']:
                    rospy.loginfo("Releasing suction for {}".format(object))
                    self.gripper.open(True)
                    self.scene.remove_attached_object(self.gripper_name)
                    break
            else:
                rospy.sleep(self.action_params['short_sleep_step'])

        rospy.loginfo("[ActionServer] Executed give{} with {}".format(str(parameters), "failure" if self.should_interrupt() else "success"))
        return self.server.set_succeeded()

    def execute_go_home(self, parameters):
        # 4. Return home
        if self.should_interrupt():
            return self.server.set_aborted()
        rospy.loginfo("Returning in idle mode")
        if self.planning:
            while True:
                leaving_traj = self.arm.plan(self.starting_pose.pose)
                if len(leaving_traj.joint_trajectory.points)>1: break
        else:
            leaving_traj = self.extras.interpolate_joint_space(self.starting_state, self.action_params['action_num_points'], kv_max=self.action_params['kv_max'], ka_max=self.action_params['ka_max'])
        self.low_level_execute_workaround(self.side, leaving_traj)
        rospy.loginfo("[ActionServer] Executed go_home{} with {}".format(str(parameters), "failure" if self.should_interrupt() else "success"))
        return self.server.set_succeeded()

    def gripper_action(self, close):
        # Performs blocking calls closing or opening grippers
        success = False
        while not success:
            if close:
                success = self.gripper.close(True)
            else:
                success = self.gripper.open(True)

    def execute_hold(self, parameters):
        # Parameters could be "/thr/handle 0", it asks the robot to hold the handle using its first hold pose
        rospy.loginfo("[ActionServer] Executing hold{}".format(str(parameters)))
        object = parameters[0]
        pose = int(parameters[1])

        # 0. Trajectories generation
        cart_dist = float('inf')
        angular_dist = float('inf')
        while cart_dist>self.action_params['hold']['approach_cartesian_dist'] or angular_dist>self.action_params['hold']['approach_angular_dist']:
            try:
                world_approach_pose = self.object_grasp_pose_to_world(self.poses[object]["hold"][pose]['approach'], object)  # Pose of the approach
            except:
                rospy.logerr("Object {} not found".format(object))
                return self.server.set_aborted()

            # Comment: goal approach is needed in both interpolate and planning mode (planning = for reapproach)
            goal_approach = self.extras.get_ik(world_approach_pose)
            if not goal_approach:
                rospy.logerr("Unable to reach approach pose")
                return self.server.set_aborted()

            if self.planning:
                approach_traj = self.arm.plan(world_approach_pose.pose)
                if len(approach_traj.joint_trajectory.points)<2:
                    rospy.logerr("Sorry, object {} is too far away for me".format(object))
                    return self.server.set_aborted()
            else:
                approach_traj = self.extras.interpolate_joint_space(goal_approach, self.action_params['action_num_points'], kv_max=self.action_params['kv_max'], ka_max=self.action_params['ka_max'])

            # 1. Go to approach pose
            if self.should_interrupt():
                return self.server.set_aborted()
            rospy.loginfo("Approaching {}".format(object))
            self.low_level_execute_workaround(self.side, approach_traj)
            try:
                new_world_approach_pose = self.object_grasp_pose_to_world(self.poses[object]["hold"][pose]['approach'], object)
            except:
                new_world_approach_pose = world_approach_pose
            cart_dist = transformations.distance(world_approach_pose, new_world_approach_pose)
            angular_dist = transformations.distance_quat(world_approach_pose, new_world_approach_pose)

        # 2. Go to "hold" pose
        if self.should_interrupt():
            return self.server.set_aborted()
        rospy.loginfo("Grasping {}".format(object))
        action_traj = self.extras.generate_descent(self.poses[object]["hold"][pose]['descent'], object, 3)
        self.low_level_execute_workaround(self.side, action_traj)

        # 3. Close gripper to grasp object
        if not self.should_interrupt():
            rospy.loginfo("Closing gripper around {}".format(object))
            self.gripper.close(True)

        # 4. Force down
        if self.should_interrupt():
            return self.server.set_aborted()
        rospy.loginfo("Forcing down on {}".format(object))
        try:
            force_traj = self.extras.generate_descent(self.poses[object]["hold"][pose]['force'], object, 1)
        except:
            rospy.logwarn('Cannot compute descent forcing down, but continuing instead')
        else:
            self.low_level_execute_workaround(self.side, force_traj)

        # 5. Wait for interruption
        while not self.should_interrupt():
            try:
                distance_wrist_gripper = transformations.norm(self.tfl.lookupTransform(self.gripper_name, "/human/wrist", rospy.Time(0)))
            except:
                rospy.logwarn("Human wrist not found")
                distance_wrist_gripper = 0
            if distance_wrist_gripper < self.action_params['hold']['sphere_radius']:
                rospy.loginfo("Human is currently working with {}... Move your hands away to stop, distance {}m, threshold {}m".format(object, distance_wrist_gripper, self.action_params['hold']['sphere_radius']))
            else:
                break
            rospy.sleep(self.action_params['sleep_step'])

        # 6. Release object
        rospy.loginfo("Human has stopped working with {}, now releasing".format(object))
        if not self.should_interrupt():
            rospy.loginfo("Releasing {}".format(object))
            self.gripper_action(False)

        # 7. Go to approach pose again (to avoid touching the fingers)
        if self.should_interrupt():
            return self.server.set_aborted()
        reapproach_traj = self.extras.reverse_trajectory(action_traj)
        rospy.loginfo("Reapproaching {}".format(object))
        #self.arm.execute(approach_traj, False)
        self.low_level_execute_workaround(self.side, reapproach_traj)

        rospy.loginfo("[ActionServer] Executed hold{} with {}".format(str(parameters), "failure" if self.should_interrupt() else "success"))
        return self.server.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('robot_action_server')
    server = RobotActionServer(sys.argv[1], planning=False, orientation_matters=True, allow_replanning=True)
    rospy.spin()