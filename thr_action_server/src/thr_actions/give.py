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
        self.init_p_wrist = None

        if not self.commander.gripping():
            rospy.logerr('Object {} is no longer gripped'.format(object))
            return False

        # 4. Wait for human wrist and handover the object to the default pose if the wrist is far away from it, close to the wrist if it's near the default pose
        rospy.loginfo("Giving {} to human wrist".format(object))
        while not self._should_interrupt():
            if not self.commander.gripping():
                break

            can_release = False
            distance_wrist_gripper = float('inf')
            world_T_wrist = None
            try:
                distance_wrist_gripper = transformations.norm(self.tfl.lookupTransform(self.gripper_name, "/human/wrist", rospy.Time(0)))
                world_T_wrist = self.tfl.lookupTransform(self.world, "/human/wrist", rospy.Time(0))
            except:
                pass

            # The function below returns True if human as moved enough so that we need to replan the trajectory
            def needs_update():
                if not self.tfl.frameExists("/human/wrist"):
                    return False
                else:
                    p_wrist = transformations.list_to_pose(self.tfl.lookupTransform(self.world, "/human/wrist", rospy.Time(0)))
                    if self.init_p_wrist is None:
                        self.init_p_wrist = p_wrist
                    distance = transformations.distance(self.init_p_wrist, p_wrist)
                    requires_update = distance > self.action_params['give']['threshold_radius']
                    if requires_update:
                        self.init_p_wrist = p_wrist
                    return requires_update

            if distance_wrist_gripper < self.action_params['give']['release_radius']:
                can_release = True
            elif world_T_wrist is not None and transformations.distance(world_T_wrist, self.action_params['give']['default']['eef']) < self.action_params['give']['give_radius']:
                # Give to the wrist
                obj_gripper = [self.poses[object]['pick'][0]['contact'], self.poses[object]['pick'][0]['approach'][1]]  # TODO: ugly
                wrist_gripper = transformations.multiply_transform(self.poses[object]['give']["/human/wrist"], obj_gripper)
                world_give_pose = self._object_grasp_pose_to_world(wrist_gripper, "/human/wrist")
                try:
                    self.commander.move_to_controlled(world_give_pose, stop_test=lambda: needs_update() or self.stop_test(), pause_test=self.pause_test)
                except ValueError:
                    # rospy.logwarn("Human wrist found but not reachable, please move it a little bit...")
                    rospy.sleep(self.action_params['sleep_step'])
                    continue
            else:
                # Give to default arm pose
                self.commander.move_to_controlled(self.default_give_pose, stop_test=lambda: needs_update() or self.stop_test(), pause_test=self.pause_test)
                can_release = True

            # 5. Wait for position disturbance of the gripper and open it to release object
            if can_release:
                rospy.loginfo("Waiting for perturbation of the gripper...")
                self.commander.wait_for_human_grasp(1.4, ignore_gripping=False)  # Blocking call
                if not rospy.is_shutdown():
                    rospy.loginfo("Releasing suction for {}".format(object))
                    self.commander.open()
                    break
            else:
                rospy.sleep(self.action_params['short_sleep_step'])

        rospy.loginfo("[ActionServer] Executed give{} with {}".format(str(parameters), "failure" if self._should_interrupt() else "success"))
        return True
