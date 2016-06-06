from . action import Action
import rospy
import transformations
from os.path import join
from os.path import exists
from os import makedirs
import glob
from vrep_ik_bridge.srv import VrepIK, VrepIKRequest


class Carry(Action):
    def __init__(self, commander, tf_listener, action_params, poses, seeds, should_interrupt=None):
        super(Carry, self).__init__(commander, tf_listener, action_params, poses, seeds, should_interrupt)
        self.gripper_name = self.commander.name + '_gripper'
        # wait for vrep ik service
        rospy.wait_for_service('/vrep/ik')
        self.ik_srv = rospy.ServiceProxy('/vrep/ik', VrepIK)

    def get_place_pose(self, pose_list, frame_id, object):
        world_T_object = pose_list
        try:
            object_T_gripper = self.tfl.lookupTransform(object, self.gripper_name, rospy.Time(0))
        except:
            return None
        else:
            world_T_gripper = transformations.multiply_transform(world_T_object, object_T_gripper)
            return world_T_gripper

    def write_to_txt(self, joint_state):
        method = rospy.get_param('/thr_experiment/current_method')
        # create the string of the pose to write in the textfile
        state_str = ''
        for value in joint_state:
            state_str += str(value) + '\t'
        state_str += '\n'
        # write the string to the file
        directory = join('/tmp', 'grid_reba')
        if not exists(directory):
            makedirs(directory)
        index = rospy.get_param('/thr_experiment/current_index')
        with open(directory + '/joints_' + method + '_' + index + '.txt', 'w') as f:
            f.write(state_str)

    def get_trajectory(self, pose):
        # get the trajectory to handover
        try:
            req = VrepIKRequest()
            req.use_viapoint = False
            req.target = pose
            return self.ik_srv(req).trajectory_to_handover
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def run(self, parameters=None, debug=True):
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

        # if frame_id.strip('/') != 'vrep_frame':
        #     raise NotImplementedError("Carry.get_place_pose doesn't accept constraint wrt another frame than /vrep_frame, you sent {}".format(frame_id))

        success = False
        rate = rospy.Rate(10)
        while not success and not self._should_interrupt():
            if rospy.get_param("/vrep/use"):
                # convert the pose list in pose stamped
                world_T_gripper = transformations.list_to_pose(pose)
                # get the trajectory from vrep
                trajectory = self.get_trajectory(world_T_gripper)
                if trajectory is None:
                    return False
                success = self.commander.execute(trajectory)
            else:
                world_T_gripper = self.get_place_pose(pose, frame_id, object)
                if world_T_gripper is None:
                    rospy.logerr("Object {} is no longer visible".format(object))
                    return False

                seed = self.commander.get_current_state()
                ik = self.commander.get_ik(world_T_gripper, source='trac', seeds=seed, params={'end_tolerance': 0.1, 'num_attempts':10})

                if ik is None:
                    rospy.logerr("Goal not reachable, please move it a little bit...")
                    # We decide to fail immediately in case IK fails because REBA parameters need to be updated (so a new Decision must be executed)
                    return False
                rospy.sleep(self.action_params['sleep_step'])

                # write the ik to a text file
                if debug:
                    self.write_to_txt(ik.joint_state.position)

                ik.joint_state.name = seed.joint_state.name  # TODOOOOOOOOOO REMOVE THIS HACK
                success = self.commander.move_to_controlled(ik, pause_test=self.pause_test, stop_test=self.stop_test)

            rate.sleep()

        rospy.loginfo("[ActionServer] Executed carry{} with {}".format(str(parameters), "failure" if self._should_interrupt() else "success"))
        return True
