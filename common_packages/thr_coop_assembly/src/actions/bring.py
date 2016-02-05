from . action import Action
import rospy
import transformations

class Bring(Action):
    def __init__(self, commander, tf_listener, action_params, poses, seeds, should_interrupt=None):
        super(Bring, self).__init__(commander, tf_listener, action_params, poses, seeds, should_interrupt)
        self.starting_state = self.commander.get_current_state()
        self.gripper_name = self.commander.name + '_gripper'

    def run(self, parameters=None):
        rospy.loginfo("[ActionServer] Executing bring{}".format(str(parameters)))
        object = parameters[0]
        wrist = '/human/wrist'

        if not self.commander.gripping():
            rospy.logerr('Object {} is no longer gripped'.format(object))
            return False

        rospy.loginfo("Bringing {} to the human".format(object))
        while not self._should_interrupt():
            try:
                distance_object_location = transformations.distance(self.tfl.lookupTransform(wrist, object, rospy.Time(0)), self.poses[object]['bring'][wrist])
                object_T_gripper = self.tfl.lookupTransform(object, self.gripper_name, rospy.Time(0))
                world_T_location = self.tfl.lookupTransform(self.world, wrist, rospy.Time(0))
            except KeyError:
                rospy.logerr("No declared pose to bring {}".format(object))
                return False
            except:
                rospy.logwarn("{} or {} not found".format(object, wrist))
                rospy.sleep(self.action_params['sleep_step'])
                continue

            rospy.loginfo("{} at {}m from {}, threshold {}m".format(object, distance_object_location, wrist, self.action_params['bring']['sphere_radius']))
            if distance_object_location > self.action_params['bring']['sphere_radius']:

                location_T_gripper = transformations.multiply_transform(self.poses[object]['bring'][wrist], object_T_gripper)
                world_T_gripper = transformations.multiply_transform(world_T_location, location_T_gripper)

                try:
                    success = self.commander.move_to_controlled(world_T_gripper, rpy=[1, 1, 0], pause_test=self.pause_test)
                except ValueError:
                    rospy.logwarn("Human wrist found but not reachable, please move it a little bit...".format(wrist))
                    rospy.sleep(self.action_params['sleep_step'])
                    continue
                else:
                    if not success:
                        return False
            else:
                break

        rospy.loginfo("[ActionServer] Executed bring{} with {}".format(str(parameters), "failure" if self._should_interrupt() else "success"))
        return True
