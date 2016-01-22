#! /usr/bin/env python
import rospy
import rospkg
import json
import tf
import actionlib
import baxter_commander
import transformations

from thr_coop_assembly.msg import RunMDPActionAction

class SequentialActionServer:
    """
    This is the action server that executes actions the robot is capable of.
    It requires two config files:
    * poses.json: poses relative to actions and objects
    * action_params.json: generic parameters for action execution and scenario
    """
    def __init__(self):
        """
        :param planning: True if motion should be planned with collision avoidance (where possible), False means joints interpolation
        """
        # General attributes
        self.rospack = rospkg.RosPack()

        # Transform/Geometric attributes
        self.tfl = tf.TransformListener(True, rospy.Duration(5*60)) # TF Interpolation ON and duration of its cache = 5 minutes
        self.world = "base"
        rospy.loginfo("Loading config...")
        self.scene = rospy.get_param("/thr/scene")

        with open(self.rospack.get_path("thr_coop_assembly")+"/config/scenes/"+self.scene+"/poses.json") as f:
            self.poses = json.load(f)
        with open(self.rospack.get_path("thr_coop_assembly")+"/config/action_params.json") as f:
            self.action_params = json.load(f)

        # Motion/MoveGroup/Grasping attributes
        rospy.loginfo("Connecting to MoveIt...")
        self.arms = {'l': baxter_commander.ArmCommander('left', kinematics='robot', default_kv_max=self.action_params['limits']['kv'], default_ka_max=self.action_params['limits']['ka']),
                     'r': baxter_commander.ArmCommander('right', kinematics='robot', default_kv_max=self.action_params['limits']['kv'], default_ka_max=self.action_params['limits']['ka'])}

        # Action server attributes
        rospy.loginfo("Starting server...")
        self.server = actionlib.SimpleActionServer('/thr/run_mdp_action', RunMDPActionAction, self.execute, False)
        self.server.start()
        rospy.loginfo('Server ready')

    def execute(self, goal):
        """
        Dispatches a new goal on the method executing each type of action
        :param goal:
        """
        try:
            if goal.action.type=='give':
                self.execute_give(goal.action.parameters)
            elif goal.action.type=='hold':
                self.execute_hold(goal.action.parameters)
            else:
                self.execute_wait()
        except:
            self.server.set_aborted()
            raise

    def execute_wait(self):
        """
        Executes a WAIT action (always successful)
        """
        rospy.loginfo("[ActionServer] Executing wait()")
        t0 = rospy.Time.now()
        while not self.server.is_preempt_requested() and (rospy.Time.now()-t0).to_sec()<self.action_params['wait']['duration'] and not rospy.is_shutdown():
            rospy.sleep(self.action_params['sleep_step'])
        self.server.set_succeeded()

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

    def execute_give(self, parameters):
        # Parameters could be "/thr/handle", it asks the robot to give the handle using the "give" pose (only 1 per object atm)

        rospy.loginfo("[ActionServer] Executing give{}".format(str(parameters)))
        object = parameters[0]
        starting_state = self.arms['l'].get_current_state()
        #starting_pose = transformations.list_to_pose(self.tfl.lookupTransform(self.world, "left_gripper", rospy.Time(0)))

        # 0. Trajectories generation
        try:
            world_approach_pose = self.object_grasp_pose_to_world(self.poses[object]["pick"][0]['approach'], object)  # Pose of the approach
        except:
            rospy.logerr("Object {} not found".format(object))
            return self.server.set_aborted()
        goal_approach = self.arms['l'].get_ik(world_approach_pose)
        if not goal_approach:
            rospy.logerr("Unable to reach approach pose")
            return self.server.set_aborted()

        # 1. Go to approach pose
        if not self.arms['l'].move_to_controlled(goal_approach):
            return self.abort(str(parameters))

        # 2. Go to "give" pose
        if self.should_interrupt():
            return self.server.set_aborted()
        rospy.loginfo("Grasping {}".format(object))
        action_traj = self.arms['l'].generate_cartesian_path(self.poses[object]["pick"][0]['descent'], object, 1.5)
        if not self.arms['l'].execute(action_traj[0]):
            return self.abort(str(parameters))

        # 3. Close gripper to grasp object
        if not self.should_interrupt():
            rospy.loginfo("Activating suction for {}".format(object))
            self.arms['l'].close()
            #self.scene.attach_box('left_gripper', object)

        # 4. Go to approach pose again with object in-hand (to avoid touching the table)
        if self.should_interrupt():
            return self.server.set_aborted()
        rospy.loginfo("Reapproaching {}".format(object))
        reapproach_traj = self.arms['l'].generate_reverse_trajectory(action_traj[0])
        if not self.arms['l'].execute(reapproach_traj):
            return self.abort(str(parameters))

        if not self.arms['l'].gripping():
            return self.abort(str(parameters))

        # 5. Wait for human wrist and approach object until we are close enough to release object
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
                world_give_pose = self.object_grasp_pose_to_world(self.poses[object]['give']["/human/wrist"], "/human/wrist")
                goal_give = self.arms['l'].get_ik(world_give_pose)
                if not goal_give:
                    rospy.logwarn("Human wrist found but not reachable, please move it a little bit...")
                    rospy.sleep(self.action_params['sleep_step'])
                    continue

                # The function below returns True if human as moved enough so that we need to replan the trajectory
                def needs_update():
                    p_wrist = transformations.list_to_pose(self.tfl.lookupTransform(self.world, "/human/wrist", rospy.Time(0)))
                    return transformations.distance(world_give_pose, p_wrist)>self.action_params['give']['sphere_radius']
                if not self.arms['l'].move_to_controlled(goal_give, test=needs_update):
                    return self.abort(str(parameters))
                #if not self.arms['l'].move_to_controlled(goal_give, test=needs_update):
                #    rospy.logwarn("Human has moved, changing the goal...")
                #    rospy.sleep(self.action_params['give']['inter_goal_sleep'])
                #    continue
            else:
                can_release = True

            # 5. Wait for position disturbance of the gripper and open it to release object
            if can_release:
                perturbation = self.arms['l'].extract_perturbation()
                rospy.loginfo("Perurbation: {}m, threshold: {}m".format(perturbation, self.action_params['give']['releasing_disturbance']))
                if perturbation>self.action_params['give']['releasing_disturbance']:
                    rospy.loginfo("Releasing suction for {}".format(object))
                    self.arms['l'].open()
                    #self.scene.remove_attached_object('left_gripper')
                    break
            else:
                rospy.sleep(self.action_params['short_sleep_step'])

        # 4. Return home
        if self.should_interrupt():
            return self.server.set_aborted()
        rospy.loginfo("Returning in idle mode")
        if not self.arms['l'].move_to_controlled(starting_state):
            return self.abort(str(parameters))

        rospy.loginfo("[ActionServer] Executed give{} with success".format(str(parameters)))
        self.server.set_succeeded()

    def execute_hold(self, parameters):
        # Parameters could be "/thr/handle 0", it asks the robot to hold the handle using its first hold pose
        rospy.loginfo("[ActionServer] Executing hold{}".format(str(parameters)))
        object = parameters[0]
        pose = int(parameters[1])

        starting_state = self.arms['r'].get_current_state()
        starting_pose = transformations.list_to_pose(self.tfl.lookupTransform(self.world, "right_gripper", rospy.Time(0)))

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
            goal_approach = self.arms['r'].get_ik(world_approach_pose)
            if not goal_approach:
                rospy.logerr("Unable to reach approach pose")
                return self.server.set_aborted()

            # 1. Go to approach pose
            if self.should_interrupt():
                return self.server.set_aborted()
            rospy.loginfo("Approaching {}".format(object))
            if not self.arms['r'].move_to_controlled(goal_approach):
                return self.abort(str(parameters))
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
        action_traj = self.arms['r'].generate_cartesian_path(self.poses[object]["hold"][pose]['descent'], object, 3)
        if not self.arms['r'].execute(action_traj[0]):
            return self.abort(str(parameters))

        # 3. Close gripper to grasp object
        if not self.should_interrupt():
            rospy.loginfo("Closing gripper around {}".format(object))
            self.arms['r'].close()

        # 4. Force down
        if self.should_interrupt():
            return self.server.set_aborted()
        rospy.loginfo("Forcing down on {}".format(object))
        try:
            force_traj = self.arms['r'].generate_cartesian_path(self.poses[object]["hold"][pose]['force'], object, 1)
        except:
            rospy.logwarn('Cannot compute descent forcing down, but continuing instead')
        else:
            if not self.arms['r'].execute(force_traj[0]):
                return self.abort(str(parameters))

        # 5. Wait for interruption
        while not self.should_interrupt():
            try:
                distance_wrist_gripper = transformations.norm(self.tfl.lookupTransform("right_gripper", "/human/wrist", rospy.Time(0)))
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
            self.arms['r'].open()

        # 7. Go to approach pose again (to avoid touching the fingers)
        if self.should_interrupt():
            return self.server.set_aborted()
        reapproach_traj = self.arms['r'].generate_reverse_trajectory(action_traj[0])
        rospy.loginfo("Reapproaching {}".format(object))
        #self.arms['right'].execute(approach_traj, False)
        if not self.arms['r'].execute(reapproach_traj):
            return self.abort(str(parameters))

        # 8. Return home
        if self.should_interrupt():
            return self.server.set_aborted()
        rospy.loginfo("Returning in idle mode")
        if not self.arms['r'].move_to_controlled(starting_state):
            return self.abort(str(parameters))
        rospy.loginfo("[ActionServer] Executed hold{} with success".format(str(parameters)))
        self.server.set_succeeded()

    def abort(self, action_name):
        rospy.logwarn("[ActionServer] Executed hold{} with failure".format(action_name))
        return self.server.set_aborted()

if __name__ == '__main__':
    rospy.init_node('sequential_action_server')
    server = SequentialActionServer()
    rospy.spin()