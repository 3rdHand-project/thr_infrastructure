from . action import Action
import rospy
import transformations
from os.path import join
from os.path import exists
from os import makedirs
from vrep_ik_bridge.srv import VrepIK, VrepIKRequest
import json
import numpy as np
import rospkg
import tf


class Carry(Action):
    def __init__(self, commander, tf_listener, action_params, poses, seeds, should_interrupt=None):
        super(Carry, self).__init__(commander, tf_listener, action_params, poses, seeds, should_interrupt)
        self.gripper_name = self.commander.name + '_gripper'
        # wait for vrep ik service
        rospy.wait_for_service('/vrep/ik')
        self.ik_srv = rospy.ServiceProxy('/vrep/ik', VrepIK)
        self.rospack = rospkg.RosPack()
        self.tfb = tf.TransformBroadcaster()

    def get_place_pose(self, pose_list, frame_id, object, shape, fixed, debug=True):
        try:
            ObjectTGripper = self.tfl.lookupTransform(object, self.gripper_name, rospy.Time(0))
        except:
            return None
        gripper_poses = []
        if fixed:
            BaseTGripper = transformations.multiply_transform(pose_list, ObjectTGripper)
            gripper_poses.append(transformations.list_to_pose(BaseTGripper))
        else:
            # get the user hand pose
            BaseTHand = pose_list
            # open file of poses for shapeo object
            pkg_dir = self.rospack.get_path("thr_scenes")
            with open(pkg_dir + '/config/' + object + '/poses.json') as datafile:
                data = json.load(datafile)
            # get the supposed pose of the hand
            ObjTHand_list = data[object]['carry'][rospy.get_param('/human/laterality')][shape]
            for p in ObjTHand_list:
                HandTObj = transformations.inverse_transform(p)
                BaseTObject = transformations.multiply_transform(BaseTHand, HandTObj)
                BaseTGripper = transformations.multiply_transform(BaseTObject, ObjectTGripper)
                gripper_poses.append(transformations.list_to_pose(BaseTGripper))

                if debug:
                    self.tfb.sendTransform(BaseTHand[0], BaseTHand[1], rospy.Time.now(), '/tmp/hand', '/base')
                    self.tfb.sendTransform(p[0], p[1], rospy.Time.now(), '/tmp/hand2', '/shapeo')
                    self.tfb.sendTransform(BaseTObject[0], BaseTObject[1], rospy.Time.now(), '/tmp/shapeo', '/base')
                    self.tfb.sendTransform(BaseTGripper[0], BaseTGripper[1], rospy.Time.now(), '/tmp/gripper', '/base')
        return gripper_poses

    def pose_in_vrep(self, pose):
        BaseTGripper = transformations.pose_to_list(pose)
        # express transformation in VREP frame
        VrepTBase = self.tfl.lookupTransform('vrep_frame', 'base', rospy.Time(0))
        GripperTVrep = self.tfl.lookupTransform('right_gripper', 'vrep_tip', rospy.Time(0))

        # apply chain rule of transformation
        robot_pose = transformations.multiply_transform(VrepTBase, BaseTGripper)
        robot_pose = transformations.multiply_transform(robot_pose, GripperTVrep)
        return robot_pose

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

    def distance_to_seed(self, joint_state, seed):
        return np.linalg.norm(np.array(joint_state) - np.array(seed))

    def get_best_pose(self, pose_list):
        seed = self.commander.get_current_state()
        best_pose = pose_list[0]
        best_ik = None
        min_dist = float('inf')
        for p in pose_list:
                ik = self.commander.get_ik(p, source='trac', seeds=seed, params={'end_tolerance': 0.1, 'num_attempts':10})
                if ik is not None:
                    dist = self.distance_to_seed(ik.joint_state.position, seed.joint_state.position)
                    if dist < min_dist:
                        best_pose = p
                        best_ik = ik
                        best_ik.joint_state.name = seed.joint_state.name  # TODOOOOOOOOOO REMOVE THIS HACK
        return best_pose, best_ik

    def run(self, parameters=None, debug=True):
        # Parameters could be "/shapeo ellipse x y z qx qy qz qw frame_id True"
        rospy.loginfo("[ActionServer] Executing carry{}".format(str(parameters)))
        object = parameters[0]
        shape = parameters[1]
        pose = transformations.raw_list_to_list(map(float, parameters[2:9]))
        frame_id = parameters[9]
        success = parameters[10] == 'True'
        fixed = parameters[11] == 'True'

        if not success:
            rospy.logerr("REBA service could not compute the carry pose")
            return False

        if frame_id.strip('/') != 'base':
            raise NotImplementedError("Carry.get_place_pose doesn't accept constraint wrt another frame than base, you sent {}".format(frame_id))

        success = False
        rate = rospy.Rate(10)
        while not success and not self._should_interrupt():
            # get the object pose
            gripper_poses = self.get_place_pose(pose, frame_id, object, shape, fixed)
            # find the best pose and ik
            best_pose, best_ik = self.get_best_pose(gripper_poses)
            # execute the trajectory to the pose either with VREP or normal IK
            if rospy.get_param('/vrep/use'):
                vrep_pose = self.pose_in_vrep(best_pose)
                # get the trajectory from vrep
                trajectory = self.get_trajectory(vrep_pose)
                if trajectory is None:
                    return False
                success = self.commander.execute(trajectory)
            else:
                if best_ik is None:
                    rospy.logerr("Goal not reachable, please move it a little bit...")
                    # We decide to fail immediately in case IK fails because REBA parameters need to be updated (so a new Decision must be executed)
                    return False
                rospy.sleep(self.action_params['sleep_step'])

                # write the ik to a text file
                if debug:
                    self.write_to_txt(best_ik.joint_state.position)

                success = self.commander.move_to_controlled(best_ik, pause_test=self.pause_test, stop_test=self.stop_test)

            rate.sleep()

        rospy.loginfo("[ActionServer] Executed carry{} with {}".format(str(parameters), "failure" if self._should_interrupt() else "success"))
        return True
