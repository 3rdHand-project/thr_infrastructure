from . action import Action
import rospy
import transformations

class Give(Action):
    def __init__(self, commander, tf_listener, action_params, poses, seeds, should_interrupt=None):
        super(Give, self).__init__(commander, tf_listener, action_params, poses, seeds, should_interrupt)
        self.starting_state = self.commander.get_current_state()
        self.gripper_name = self.commander.name + '_gripper'

    def run(self, parameters=None):
        # Parameters could be "/thr/handle", it asks the robot to give the handle using the "give" pose (only 1 per object atm)
        rospy.loginfo("[ActionServer] Executing give{}".format(str(parameters)))
        object = parameters[0]

        # 4. Wait for human wrist and approach object until we are close enough to release object
        rospy.loginfo("Bringing {} to human wrist".format(object))
        while not self._should_interrupt():
            can_release = False
            try:
                distance_wrist_gripper = transformations.norm(self.tfl.lookupTransform(self.gripper_name, "/human/wrist", rospy.Time(0)))
            except:
                rospy.logwarn("Human wrist not found")
                rospy.sleep(self.action_params['sleep_step'])
                continue
            rospy.loginfo("User wrist at {}m from gripper, threshold {}m".format(distance_wrist_gripper, self.action_params['give']['sphere_radius']))
            if distance_wrist_gripper > self.action_params['give']['sphere_radius']:
                world_give_pose = self._object_grasp_pose_to_world(self.action_params['give']['give_pose'], "/human/wrist")

                # The function below returns True if human as moved enough so that we need to replan the trajectory
                def needs_update():
                    p_wrist = transformations.list_to_pose(self.tfl.lookupTransform(self.world, "/human/wrist", rospy.Time(0)))
                    return transformations.distance(world_give_pose, p_wrist)>self.action_params['give']['sphere_radius']

                try:
                    success = self.commander.move_to_controlled(world_give_pose, test=needs_update)
                except ValueError:
                    rospy.logwarn("Human wrist found but not reachable, please move it a little bit...")
                    rospy.sleep(self.action_params['sleep_step'])
                    continue
                else:
                    if not success:
                        return False

                #if not self.low_level_execute_workaround(self.side, give_traj, needs_update):
                #    rospy.logwarn("Human has moved, changing the goal...")
                #    rospy.sleep(self.action_params['give']['inter_goal_sleep'])
                #    continue
            else:
                can_release = True

            # 5. Wait for position disturbance of the gripper and open it to release object
            if can_release:
                rospy.loginfo("Waiting for perturbation of the gripper...")
                self.commander.wait_for_human_grasp(1.9)  # Blocking call
                if not rospy.is_shutdown():
                    rospy.loginfo("Releasing suction for {}".format(object))
                    self.commander.open()
                    break
            else:
                rospy.sleep(self.action_params['short_sleep_step'])

        rospy.loginfo("[ActionServer] Executed give{} with {}".format(str(parameters), "failure" if self._should_interrupt() else "success"))
        return True