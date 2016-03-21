from . action import Action
from baxter_commander.persistence import dicttostate
import rospy
import numpy as np
import transformations

class Turn(Action):
    def __init__(self, commander, tf_listener, action_params, poses, seeds, should_interrupt=None):
        super(Turn, self).__init__(commander, tf_listener, action_params, poses, seeds, should_interrupt)
        self.gripper = commander.name+'_gripper'
        self.rotating_joint = self.commander.name + '_w2'

    def run(self, parameters=None):
        # Parameters could be "/thr/handle 0 0 0 1", it asks the robot to turn the handle so that it reaches the specified quaternion 0 0 0 1
        # This quaternion is given wrt the world frame (base)
        rospy.loginfo("[ActionServer] Executing turn{}".format(str(parameters)))
        object = parameters[0]
        quaternion = map(float, parameters[2:5])
        backward = parameters[1] == "1"

        def goal_reached():
            # Returns True if object has reached the required angles
            current_quat = self.tfl.lookupTransform(self.world, object, rospy.Time(0))[1]
            distance_goal = transformations.distance_quat(current_quat, quaternion)
            return distance_goal < self.action_params['turn']['goal_angular_dist']

        def check_and_get_approach_pose(object, pose_id):
            # Returns None if object is not found, else True if approach pose is reached, the approach pose in world frame otherwise
            try:
                world_approach_pose = self._object_grasp_pose_to_world(self.poses[object]["turn"][best_pose_id]['approach'], object)
            except:
                rospy.logerr("Object {} not found".format(object))
                return None

            world_current_pose = self.tfl.lookupTransform(self.world, self.gripper, rospy.Time(0))

            cart_dist = transformations.distance(world_approach_pose, world_current_pose)
            angular_dist = transformations.distance_quat(world_approach_pose, world_current_pose)
            if cart_dist > self.action_params['turn']['approach_cartesian_dist'] or angular_dist > self.action_params['turn']['approach_angular_dist']:
                return world_approach_pose
            else:
                return True

        while not goal_reached():
            # 1. Choose the best grasp pose
            #    Strategy: if we have to go forward (resp backward), pick the grasp pose whose IK returns the minimum (resp max) angle of w2
            #
            #    Example for a forward rotation:
            #    -PI -----------------> 0 -----------------> +PI
            #               |                           |
            #         good_w2_angle                bad_w2_angle
            #
            best_pose_id = None
            best_angle = float('inf') * (-1 if backward else 1)
            for pose_id, pose in enumerate(self.poses[object]["turn"]):
                ik = self.commander.get_ik(self._object_grasp_pose_to_world(pose['approach'], object))
                if ik is not None:
                    joint = ik.joint_state.name.index(self.rotating_joint)
                    if backward and ik.joint_state.position[joint] > best_angle or \
                       not backward and ik.joint_state.position[joint] < best_angle:
                        best_pose_id = pose_id
                        best_angle = ik.joint_state.position[joint]

            if best_pose_id is None:
                rospy.logerr("Unable to reach approach pose (preliminary pose selection)")

            # 2. Go to the approach pose of the selected grasp pose, loop in case object moves
            while True:
                approach_pose = check_and_get_approach_pose(object, best_pose_id)
                if approach_pose is None or self._should_interrupt():
                    return False
                elif approach_pose is True:
                    break
                else:
                    rospy.loginfo("Approaching {}".format(object))
                    if not self.commander.move_to_controlled(approach_pose, pause_test=self.pause_test, stop_test=self.stop_test):
                        rospy.logerr("Unable to reach approach pose (motion execution)")
                        return False
            rospy.sleep(1)

            if self._should_interrupt():
                return False

            rospy.loginfo("Grasping {}".format(object))
            grasp = np.array(self.poses[object]["turn"][best_pose_id]['contact'])
            approach = np.array(self.poses[object]["turn"][best_pose_id]['approach'][0])
            descent = list(grasp - approach)
            if not self.commander.translate_to_cartesian(descent, object, 1., pause_test=self.pause_test, stop_test=self.stop_test):
                return False

            rospy.loginfo("Turning {}".format(object))
            # We want to rotate, no matter the goal angle since we'll stop once object has reached the goal quaternion
            goal = -6.29 if backward else 6.29
            self.commander.rotate_joint(self.rotating_joint, goal, pause_test=self.pause_test, stop_test=lambda: self.stop_test() or goal_reached())

            if self._should_interrupt():
                return False

            rospy.loginfo("Leaving {}".format(object))
            rising = list(-np.array(descent))
            if not self.commander.translate_to_cartesian(rising, object, 1., pause_test=self.pause_test, stop_test=self.stop_test):
                return False

        if goal_reached():
            rospy.loginfo("[ActionServer] Executed turn{} with success".format(str(parameters)))
            return True
        else:
            rospy.logerr("[ActionServer] Executed turn{} with failure, goal not reached".format(str(parameters)))
            return False
