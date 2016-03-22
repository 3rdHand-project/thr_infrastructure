from . action import Action
from baxter_commander.persistence import dicttostate
from tf import LookupException
from transformations import distance
import rospy
import numpy as np

class Pick(Action):
    def __init__(self, commander, tf_listener, action_params, poses, seeds, should_interrupt=None):
        super(Pick, self).__init__(commander, tf_listener, action_params, poses, seeds, should_interrupt)
        self.gripper = commander.name + '_gripper'

    def run(self, parameters=None):
        rospy.loginfo("[ActionServer] Executing pick{}".format(str(parameters)))
        object = parameters[0]

        # 1. Go to approach pose
        try:
            world_approach_pose = self._object_grasp_pose_to_world(self.poses[object]["pick"][0]['approach'], object)
        except LookupException:
            rospy.logerr("Object {} not found".format(object))
            return False

        goal_approach = self.commander.get_ik(world_approach_pose, dicttostate(self.seeds['pick']))
        if not goal_approach:
            rospy.logerr("Unable to reach approach pose")
            return False

        while True:
            if self._should_interrupt():
                return False
            rospy.loginfo("Approaching {}".format(object))
            if not self.commander.move_to_controlled(goal_approach, pause_test=self.pause_test, stop_test=self.stop_test):
                return False

            # We just check that motion was precise enough, no target recomputation if object moves (needs another IK)
            actual_world_approach_pose = self.tfl.lookupTransform(self.world, self.gripper, rospy.Time(0))  # Cannot fail
            if distance(actual_world_approach_pose, world_approach_pose) < self.action_params['pick']['approach_cartesian_dist']:
                break

        # 2. Go to "pick" pose
        if self._should_interrupt():
            return False
        rospy.loginfo("Grasping {}".format(object))

        # Selecting descent vector mode or grasp point mode
        if 'descent' in self.poses[object]["pick"][0]:
            if not self.commander.translate_to_cartesian(self.poses[object]["pick"][0]['descent'], object, 1., pause_test=self.pause_test, stop_test=self.stop_test):
                rospy.logerr("Unable to generate picking descent")
                return False
        elif 'contact' in self.poses[object]["pick"][0]:
            grasp = np.array(self.poses[object]["pick"][0]['contact'])
            approach = self.tfl.lookupTransform(object, self.gripper, rospy.Time(0))[0]
            descent = list(grasp - approach)
            if not self.commander.translate_to_cartesian(descent, object, 1, pause_test=self.pause_test, stop_test=self.stop_test):
                rospy.logerr("Unable to generate picking descent")
                return False
        else:
            rospy.logerr("No 'contact' nor 'descent' attribute defined for picking object {}".format(object))
            return False


        # 3. Close gripper to grasp object
        if not self._should_interrupt():
            rospy.loginfo("Activating suction for {}".format(object))
            self.commander.close()
            #self.scene.attach_box(self.gripper_name, object)

        # 4. Rise the object to avoid touching the table
        if self._should_interrupt():
            return False
        rospy.loginfo("Rising {} with respect to the world".format(object))
        if not self.commander.translate_to_cartesian(self.poses[object]["pick"][0]['rise'], self.world, 1., pause_test=self.pause_test, stop_test=self.stop_test):
            rospy.logerr("Unable to generate picking rising")
            return False

        rospy.loginfo("[ActionServer] Executed pick{} with {}".format(str(parameters), "success" if self.commander.gripping() else "failure"))
        return self.commander.gripping()
