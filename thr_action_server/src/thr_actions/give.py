from . action import Action
from baxter_commander.persistence import dicttostate
import rospy
import transformations


class Give(Action):
    def __init__(self, commander, tf_listener, action_params, poses, seeds, should_interrupt=None):
        super(Give, self).__init__(commander, tf_listener, action_params, poses, seeds, should_interrupt)
        self.starting_state = self.commander.get_current_state()
        self.gripper_name = self.commander.name + '_gripper'
        self.default_give_pose = dicttostate(self.action_params['give']['default']['joints'])

    def run(self, parameters=None):
        # Parameters could be "/thr/handle", it asks the robot to give the handle using the "give" pose (only 1 per object atm)
        rospy.loginfo("[ActionServer] Executing give{}".format(str(parameters)))
        object = parameters[0]

        if not self.commander.gripping():
            rospy.logerr('Object {} is no longer gripped'.format(object))
            return False

        rospy.loginfo("Giving {} to default give pose".format(object))

        def stop_test():
            return not self.commander.gripping() or self.stop_test()

        # Give to default arm pose
        if not self._should_interrupt():
            self.commander.move_to_controlled(self.default_give_pose, stop_test=stop_test, pause_test=self.pause_test)

        if not self.commander.gripping():
            rospy.logwarn("[ActionServer] Executed give{} failed: object dropped".format(str(parameters)))
            return False

        # Wait for position disturbance of the gripper and open it to release object
        rospy.loginfo("Waiting for perturbation of the gripper...")
        self.commander.wait_for_human_grasp(1.4, ignore_gripping=False)  # Blocking call

        if not self._should_interrupt():
            rospy.loginfo("Releasing suction for {}".format(object))
            self.commander.open()

        rospy.loginfo("[ActionServer] Executed give{} with {}".format(str(parameters), "failure" if self._should_interrupt() else "success"))
        return True
