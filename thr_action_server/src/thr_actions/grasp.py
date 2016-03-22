from . action import Action
from baxter_commander.persistence import dicttostate
import rospy
import transformations
import numpy as np

class Grasp(Action):
    def __init__(self, commander, tf_listener, action_params, poses, seeds, should_interrupt=None):
        super(Grasp, self).__init__(commander, tf_listener, action_params, poses, seeds, should_interrupt)
        self.gripper = commander.name+'_gripper'

    def run(self, parameters=None):
        # Parameters could be "/thr/handle 0", it asks the robot to grasp the handle using its first grasp pose
        rospy.loginfo("[ActionServer] Executing grasp{}".format(str(parameters)))
        object = parameters[0]
        pose = 0  # At the moment we have only 1 grasp pose

        self.commander.open()

        cart_dist = float('inf')
        angular_dist = float('inf')
        while cart_dist > self.action_params['grasp']['approach_cartesian_dist'] or angular_dist>self.action_params['grasp']['approach_angular_dist']:
            try:
                world_approach_pose = self._object_grasp_pose_to_world(self.poses[object]["grasp"][pose]['approach'], object)  # Pose of the approach
            except:
                rospy.logerr("Object {} not found".format(object))
                return False

            goal_approach = self.commander.get_ik(world_approach_pose) #, dicttostate(self.seeds['grasp'])) # No seed provided
            if not goal_approach:
                rospy.logerr("Unable to reach approach pose")
                return False

            # 1. Go to approach pose
            if self._should_interrupt():
                return False

            rospy.loginfo("Approaching {}".format(object))
            if not self.commander.move_to_controlled(goal_approach, pause_test=self.pause_test, stop_test=self.stop_test):
                return False

            try:
                new_world_approach_pose = self._object_grasp_pose_to_world(self.poses[object]["grasp"][pose]['approach'], object)
            except:
                new_world_approach_pose = world_approach_pose

            cart_dist = transformations.distance(world_approach_pose, new_world_approach_pose)
            angular_dist = transformations.distance_quat(world_approach_pose, new_world_approach_pose)

        # 1.ter. Sleep
        rospy.sleep(1)

        # 2. Go to "grasp" pose
        if self._should_interrupt():
            return False

        rospy.loginfo("Grasping {}".format(object))
        grasp = np.array(self.poses[object]["grasp"][pose]['grasp'])
        approach = self.tfl.lookupTransform(object, self.gripper, rospy.Time(0))[0]
        descent = list(grasp - approach)
        rospy.loginfo("Generated descent vector {}".format(str(descent)))
        if not self.commander.translate_to_cartesian(descent, object, 2., pause_test=self.pause_test, stop_test=self.stop_test):
            rospy.logerr("Unable to generate grasp descent")
            return False

        # 2.bis. Sleep
        rospy.sleep(2)

        # 3. Close gripper to grasp object
        if not self._should_interrupt():
            rospy.loginfo("Closing gripper around {}".format(object))
            self.commander.close()

        rospy.loginfo("Rising {}".format(object))
        if self._should_interrupt():
            return False

        if not self.commander.translate_to_cartesian(self.poses[object]['grasp'][0]['rise'], self.world, 1.5, pause_test=self.pause_test, stop_test=self.stop_test):
            rospy.logerr("Unable to generate rising for {}".format(object))
            return False

        if not self.commander.gripping():
            rospy.logerr('Object {} is not gripped'.format(object))
            return False

        rospy.loginfo("[ActionServer] Executed grasp{} with success".format(str(parameters)))
        return True
