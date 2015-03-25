#!/usr/bin/env python

import rospy
import tf
from thr_coop_assembly.srv import GetSceneState, GetSceneStateResponse
from thr_coop_assembly.msg import SceneState, Predicate

class SceneStateManager(object):
    def __init__(self):
        self.state = SceneState()
        self.world = "base"
        try:
            self.objects = rospy.get_param('/optitrack/objects')
        except KeyError:
            raise KeyError("Unable to read /optitrack/objects, have you used roslaunch?")
        self.poses = {}
        self.tfl = tf.TransformListener()

    def generate_predicate(self):
        self.state.header.stamp = rospy.Time.now()
        raise NotImplementedError("scene_state_manager.generate_predicate() should update self.state.predicates "
                                  "based on the self.poses map {'/human/wrist':[[x, y, z], [x, y, z, w]], ...}")

    def update_state(self):
        for obj in self.objects:
            self.poses[obj] = self.tfl.lookupTransform(self.world, obj, rospy.Time(0))
        self.generate_predicate()

    def handle_request(self, req):
        self.update_state()
        resp = GetSceneStateResponse()
        resp.commands = self.reader.get_user_commands()
        return resp

    def run(self):
        s = rospy.Service('/thr/scene_state', GetSceneState, self.handle_request)
        rospy.loginfo('[SceneStateManager] server ready to track {}...'.format(str(self.objects)))
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('scene_state_manager')
    SceneStateManager().run()