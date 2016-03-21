from . action import Action
from baxter_commander.persistence import dicttostate
import rospy
import numpy as np
import transformations

class Turn(Action):
    def __init__(self, commander, tf_listener, action_params, poses, seeds, should_interrupt=None):
        super(Turn, self).__init__(commander, tf_listener, action_params, poses, seeds, should_interrupt)
        self.gripper = commander.name+'_gripper'

    def run(self, parameters=None):
        # Parameters could be "/thr/handle 0 0 0 1", it asks the robot to turn the handle so that it reaches the specified quaternion 0 0 0 1
        # This quaternion is given wrt the world frame (base)
        rospy.loginfo("[ActionServer] Executing turn{}".format(str(parameters)))
        object = parameters[0]
        quaternion = map(float, parameters[1:4])

        def goal_reached():
            current_quat = self.tfl.lookupTransform(self.world, object, rospy.Time(0))[1]
            distance_goal = transformations.distance_quat(current_quat, quaternion)
            return distance_goal < self.action_params['turn']['goal_angular_dist']

        cart_dist = float('inf')
        angular_dist = float('inf')
        while cart_dist > self.action_params['turn']['approach_cartesian_dist'] or angular_dist > self.action_params['turn']['approach_angular_dist']:
            try:
                world_approach_pose = self._object_grasp_pose_to_world(self.poses[object]["turn"]['approach'], object)
            except:
                rospy.logerr("Object {} not found".format(object))
                return False

            # 1. Go to approach pose
            if self._should_interrupt():
                return False

            rospy.loginfo("Approaching {}".format(object))
            if not self.commander.move_to_controlled(world_approach_pose, pause_test=self.pause_test, stop_test=self.stop_test):
                rospy.logerr("Unable to reach approach pose")
                return False

            try:
                new_world_approach_pose = self._object_grasp_pose_to_world(self.poses[object]["turn"]['approach'], object)
            except:
                new_world_approach_pose = world_approach_pose

            cart_dist = transformations.distance(world_approach_pose, new_world_approach_pose)
            angular_dist = transformations.distance_quat(world_approach_pose, new_world_approach_pose)

        while not goal_reached():
            rospy.sleep(1)

            if self._should_interrupt():
                return False

            state_before_rotation = self.commander.get_current_state()

            rospy.loginfo("Grasping {}".format(object))
            grasp = np.array(self.poses[object]["turn"]['contact'])
            approach = np.array(self.poses[object]["turn"]['approach'][0])
            descent = list(grasp - approach)
            if not self.commander.translate_to_cartesian(descent, object, 1., pause_test=self.pause_test, stop_test=self.stop_test):
                return False

            rospy.loginfo("Turning {}".format(object))
            # We want to rotate, no matter the goal angle since we'll stop once object has reached the goal quaternion
            self.commander.rotate_joint(self.commander.name + '_w2', 6.29, pause_test=self.pause_test, stop_test=lambda: self.stop_test() or goal_reached())

            # 7. Go to approach pose again (to avoid touching the fingers)
            if self._should_interrupt():
                return False

            rospy.loginfo("Leaving {}".format(object))
            rising = list(-np.array(descent))
            if not self.commander.translate_to_cartesian(rising, object, 1., pause_test=self.pause_test, stop_test=self.stop_test):
                return False

            if not goal_reached():
                rospy.loginfo("Restoring gripper")
                self.commander.move_to_controlled(state_before_rotation, pause_test=self.pause_test, stop_test=self.stop_test)

        if goal_reached():
            rospy.loginfo("[ActionServer] Executed turn{} with success".format(str(parameters)))
            return True
        else:
            rospy.logerr("[ActionServer] Executed turn{} with failure, goal not reached".format(str(parameters)))
            return False
