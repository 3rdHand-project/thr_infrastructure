from . action import Action
import rospy
import transformations

class Carry(Action):
    def __init__(self, commander, tf_listener, action_params, poses, seeds, should_interrupt=None):
        super(Carry, self).__init__(commander, tf_listener, action_params, poses, seeds, should_interrupt)
        self.gripper_name = self.commander.name + '_gripper'

    def get_place_pose(self, pose_list, frame_id, object):
        world_T_object = pose_list
        try:
            object_T_gripper = self.tfl.lookupTransform(object, self.gripper_name, rospy.Time(0))
        except:
            return None
        else:
            world_T_gripper = transformations.multiply_transform(world_T_object, object_T_gripper)
            return world_T_gripper

    def run(self, parameters=None):
        # Parameters could be "/shapeo ellipse x y z qx qy qz qw frame_id True"
        rospy.loginfo("[ActionServer] Executing carry{}".format(str(parameters)))
        object = parameters[0]
        shape = parameters[1]
        pose = transformations.raw_list_to_list(map(float, parameters[2:9]))
        frame_id = parameters[9]
        success = parameters[10] == 'True'

        if not success:
            rospy.logerr("REBA service could not compute the carry pose")
            return False

        if frame_id.strip('/') != 'base':
            raise NotImplementedError("Carry.get_place_pose doesn't accept constraint wrt another frame than /base, you sent {}".format(frame_id))

        success = False
        while not success and not self._should_interrupt():
            if not self.commander.gripping():
                rospy.logerr('Object {} is no longer gripped'.format(object))
                return False

            world_T_gripper = self.get_place_pose(pose, frame_id, object)
            if world_T_gripper is None:
                rospy.logerr("Object {} is no longer visible".format(object))
                return False

            try:
                success = self.commander.move_to_controlled(world_T_gripper, pause_test=self.pause_test, stop_test=self.stop_test)
            except ValueError:
                rospy.logerr("Goal not reachable, please move it a little bit...")
                # We decide to fail immediately in case IK fails because REBA parameters need to be updated (so a new Decision must be executed)
                return False
                # rospy.sleep(self.action_params['sleep_step'])
            else:
                if success:
                    break

        rospy.loginfo("[ActionServer] Executed carry{} with {}".format(str(parameters), "failure" if self._should_interrupt() else "success"))
        return True
