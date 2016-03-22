from . action import Action
from baxter_commander.persistence import dicttostate
from thr_infrastructure_msgs.srv import GetSceneStateRequest, GetSceneState
from numpy import array
import rospy
import numpy as np
import transformations

class Hold(Action):
    def __init__(self, commander, tf_listener, action_params, poses, seeds, should_interrupt=None):
        super(Hold, self).__init__(commander, tf_listener, action_params, poses, seeds, should_interrupt)
        self.gripper = commander.name+'_gripper'
        self.scene = None
        self.scene_state_service = '/thr/scene_state'

    def update_scene(self):
        request = GetSceneStateRequest()
        try:
            getscene = rospy.ServiceProxy(self.scene_state_service, GetSceneState)
            self.scene = getscene(request).state
        except rospy.ServiceException, e:
            rospy.logerr("Cannot update scene {}:".format(e.message))

    def run(self, parameters=None):
        # Parameters could be "/thr/handle 0", it asks the robot to hold the handle using its first hold pose
        rospy.loginfo("[ActionServer] Executing hold{}".format(str(parameters)))
        object = parameters[0]
        pose = int(parameters[1])

        cart_dist = float('inf')
        angular_dist = float('inf')
        while cart_dist > self.action_params['hold']['approach_cartesian_dist'] or angular_dist>self.action_params['hold']['approach_angular_dist']:
            try:
                world_approach_pose = self._object_grasp_pose_to_world(self.poses[object]["hold"][pose]['approach'], object)  # Pose of the approach
            except:
                rospy.logerr("Object {} not found".format(object))
                return False

            goal_approach = self.commander.get_ik(world_approach_pose, dicttostate(self.seeds['hold']))
            if not goal_approach:
                rospy.logerr("Unable to reach approach pose")
                return False

            # 1. Go to approach pose
            if self._should_interrupt():
                return False

            rospy.loginfo("Approaching {}".format(object))
            if not self.commander.move_to_controlled(goal_approach, pause_test=self.pause_test, stop_test=self.stop_test):
                return False

            try:
                new_world_approach_pose = self._object_grasp_pose_to_world(self.poses[object]["hold"][pose]['approach'], object)
            except:
                new_world_approach_pose = world_approach_pose

            cart_dist = transformations.distance(world_approach_pose, new_world_approach_pose)
            angular_dist = transformations.distance_quat(world_approach_pose, new_world_approach_pose)

        # 1.ter. Sleep
        rospy.sleep(1)

        # 2. Go to "hold" pose
        if self._should_interrupt():
            return False

        rospy.loginfo("Grasping {}".format(object))
        grasp = np.array(self.poses[object]["hold"][pose]['contact'])
        approach = self.tfl.lookupTransform(object, self.gripper, rospy.Time(0))[0]
        descent = list(grasp - approach)
        if not self.commander.translate_to_cartesian(descent, object, 1., pause_test=self.pause_test, stop_test=self.stop_test):
            return False

        # 3. Close gripper to grasp object
        if not self._should_interrupt():
            rospy.loginfo("Closing gripper around {}".format(object))
            self.commander.close()

        # 4. Force down
        if self._should_interrupt():
            return False

        rospy.loginfo("Forcing down on {}".format(object))
        force_vector = self.poses[object]["hold"][pose]['force'] * array(descent)
        if not self.commander.translate_to_cartesian(force_vector, object, 1, pause_test=self.pause_test, stop_test=self.stop_test):
            return False

        # 5. Wait for interruption
        while not self._should_interrupt():
            try:
                distance_wrist_gripper = transformations.norm(self.tfl.lookupTransform(self.gripper, "/human/wrist", rospy.Time(0)))
            except:
                rospy.logwarn("Human wrist not found")
                distance_wrist_gripper = 0
            self.update_scene()
            attached = len([pred for pred in self.scene.predicates if pred.type == 'attached' and pred.parameters[0] == object and int(pred.parameters[2]) == pose]) > 0
            if distance_wrist_gripper < self.action_params['hold']['sphere_radius'] or attached:
                # rospy.loginfo("Human is currently working with {}... Move your hands away to stop, distance {}m, threshold {}m".format(object, distance_wrist_gripper, self.action_params['hold']['sphere_radius']))
                pass
            else:
                break
            rospy.sleep(self.action_params['sleep_step'])

        # 6. Release object
        rospy.loginfo("Human has stopped working with {}, now releasing".format(object))
        if not self._should_interrupt():
            rospy.loginfo("Releasing {}".format(object))
            self.commander.open()

        # 7. Go to approach pose again (to avoid touching the fingers)
        if self._should_interrupt():
            return False

        rospy.loginfo("Leaving {}".format(object))
        rising = list(-array(descent))
        if not self.commander.translate_to_cartesian(rising, object, 1., pause_test=self.pause_test, stop_test=self.stop_test):
            return False

        rospy.loginfo("[ActionServer] Executed hold{} with success".format(str(parameters)))
        return True
