from . action import Action
from baxter_commander.persistence import dicttostate
import rospy
import transformations

class Hold(Action):
    def __init__(self, commander, tf_listener, action_params, poses, seeds, should_interrupt=None):
        super(Hold, self).__init__(commander, tf_listener, action_params, poses, seeds, should_interrupt)

    def run(self, parameters=None):
        # Parameters could be "/thr/handle 0", it asks the robot to hold the handle using its first hold pose
        rospy.loginfo("[ActionServer] Executing hold{}".format(str(parameters)))
        object = parameters[0]
        pose = int(parameters[1])

        cart_dist = float('inf')
        angular_dist = float('inf')
        while cart_dist > self.action_params['hold']['approach_cartesian_dist'] or angular_dist>self.action_params['hold']['approach_angular_dist']:
            try:
                world_approach_pose = self._object_grasp_pose_to_world(self.poses[object]["hold"][pose]['approach'], object)  # Pose of the approach
            except:
                rospy.logerr("Object {} not found".format(object))
                return False

            goal_approach = self.commander.get_ik(world_approach_pose, dicttostate(self.seeds['hold']))
            if not goal_approach:
                rospy.logerr("Unable to reach approach pose")
                return False

            # 1. Go to approach pose
            if self._should_interrupt():
                return False

            rospy.loginfo("Approaching {}".format(object))
            if not self.commander.move_to_controlled(goal_approach):
                return False

            # 1.bis. Double motion to improve precision
            #rospy.sleep(4)
            #self.commander.move_to_controlled(goal_approach)

            try:
                new_world_approach_pose = self._object_grasp_pose_to_world(self.poses[object]["hold"][pose]['approach'], object)
            except:
                new_world_approach_pose = world_approach_pose

            cart_dist = transformations.distance(world_approach_pose, new_world_approach_pose)
            angular_dist = transformations.distance_quat(world_approach_pose, new_world_approach_pose)

        # 1.ter. Sleep
        rospy.sleep(1)

        # 2. Go to "hold" pose
        if self._should_interrupt():
            return False

        rospy.loginfo("Grasping {}".format(object))
        action_traj = self.commander.generate_cartesian_path(self.poses[object]["hold"][pose]['descent'], object, 2)

        if action_traj[1]<0.9:
            rospy.logerr("Unable to generate hold descent")
            return False

        if not self.commander.execute(action_traj[0]):
            return False

        # 3. Close gripper to grasp object
        if not self._should_interrupt():
            rospy.loginfo("Closing gripper around {}".format(object))
            self.commander.close()

        # 4. Force down
        if self._should_interrupt():
            return False

        rospy.loginfo("Forcing down on {}".format(object))
        force = self.commander.generate_cartesian_path(self.poses[object]["hold"][pose]['force'], object, 1)

        if not self.commander.execute(force[0]):
            return False

        # 5. Wait for interruption
        while not self._should_interrupt():
            try:
                distance_wrist_obj = transformations.norm(self.tfl.lookupTransform(object, "/human/wrist", rospy.Time(0)))
            except:
                rospy.logwarn("Human wrist not found")
                distance_wrist_obj = 0
            if distance_wrist_obj < self.action_params['hold']['sphere_radius']:
                rospy.loginfo("Human is currently working with {}... Move your hands away to stop, distance {}m, threshold {}m".format(object, distance_wrist_obj, self.action_params['hold']['sphere_radius']))
            else:
                break
            rospy.sleep(self.action_params['sleep_step'])

        # 6. Release object
        rospy.loginfo("Human has stopped working with {}, now releasing".format(object))
        if not self._should_interrupt():
            rospy.loginfo("Releasing {}".format(object))
            self.commander.open()

        # 7. Go to approach pose again (to avoid touching the fingers)
        if self._should_interrupt():
            return False

        reapproach_traj = self.commander.generate_reverse_trajectory(action_traj[0])
        rospy.loginfo("Reapproaching {}".format(object))
        if not self.commander.execute(reapproach_traj):
            return False

        rospy.loginfo("[ActionServer] Executed hold{} with success".format(str(parameters)))
        return True
