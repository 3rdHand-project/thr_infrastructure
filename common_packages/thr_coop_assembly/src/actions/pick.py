from . action import Action
from baxter_commander.persistence import dicttostate
import rospy

class Pick(Action):
    def __init__(self, commander, tf_listener, action_params, poses, seeds, should_interrupt=None):
        super(Pick, self).__init__(commander, tf_listener, action_params, poses, seeds, should_interrupt)

    def run(self, parameters=None):
        rospy.loginfo("[ActionServer] Executing pick{}".format(str(parameters)))
        object = parameters[0]

        # 0. Trajectories generation
        try:
            world_approach_pose = self._object_grasp_pose_to_world(self.poses[object]["give"][0]['approach'], object)  # Pose of the approach
        except:
            rospy.logerr("Object {} not found".format(object))
            return False
        goal_approach = self.commander.get_ik(world_approach_pose, dicttostate(self.seeds['pick']))
        if not goal_approach:
            rospy.logerr("Unable to reach approach pose")
            return False

        # 1. Go to approach pose
        if self._should_interrupt():
            return False
        rospy.loginfo("Approaching {}".format(object))
        if not self.commander.move_to_controlled(goal_approach):
            return False

        # 2. Go to "give" pose
        if self._should_interrupt():
            return False
        rospy.loginfo("Grasping {}".format(object))
        action_traj = self.commander.generate_cartesian_path(self.poses[object]["give"][0]['descent'], object, 1.5)
        if action_traj[1]<0.9:
            raise RuntimeError("Unable to generate descent")
        if not self.commander.execute(action_traj[0]):
            return False

        # 3. Close gripper to grasp object
        if not self._should_interrupt():
            rospy.loginfo("Activating suction for {}".format(object))
            self.commander.close()
            #self.scene.attach_box(self.gripper_name, object)

        # 4. Go to approach pose again with object in-hand (to avoid touching the table)
        if self._should_interrupt():
            return False
        rospy.loginfo("Reapproaching {}".format(object))
        reapproach_traj = self.commander.generate_reverse_trajectory(action_traj[0])
        if not self.commander.execute(reapproach_traj):
            return False

        rospy.loginfo("[ActionServer] Executed pick{} with {}".format(str(parameters), "success" if self.commander.gripping() else "failure"))
        return self.commander.gripping()
