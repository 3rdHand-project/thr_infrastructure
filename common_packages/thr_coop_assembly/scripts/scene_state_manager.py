#!/usr/bin/env python

import rospy, rospkg, tf, transformations
from thr_coop_assembly.srv import GetSceneState, GetSceneStateResponse
from thr_coop_assembly.msg import SceneState, Predicate
from itertools import combinations
import json

class SceneStateManager(object):
    def __init__(self):
        self.state = SceneState()
        self.world = "base"

        try:
            self.objects = rospy.get_param('/optitrack/objects')  # TODO get from something specific to the scene instead
        except KeyError:
            raise KeyError("Unable to read /optitrack/objects, have you used roslaunch?")
        self.scene = rospy.get_param('/thr/scene')

        self.rospack = rospkg.RosPack()
        with open(self.rospack.get_path("thr_coop_assembly")+"/config/constraints.json") as f:
            self.constraints = json.load(f)[self.scene]

        with open(self.rospack.get_path("thr_coop_assembly")+"/config/perception.json") as f:
            self.config = json.load(f)

        self.predicates = []
        self.tfl = tf.TransformListener()

    def update_state(self):
        """
        This function updates the scene state by creating predicates observed from tf and
        :return:
        """
        self.state.header.stamp = rospy.Time.now()

        # Predicate Attached()
        for o1, o2 in combinations(self.objects, 2):
            try:
                human_relative = self.tfl.lookupTransform(o1, o2, rospy.Time(0))
            except:
                pass
            else:
                key = o1+'_'+o2 if o1<o2 else o2+'_'+o1
                try:
                    cart_dist = transformations.distance(self.constraints[key], human_relative)
                except KeyError:
                    pass
                else:
                    # TODO this measures only cartesian distance, missing quaternions
                    if cart_dist<self.config['position_tolerance']:
                        p = Predicate()
                        p.type = Predicate.ATTACHED
                        p.objects = [o1, o2]
                        self.predicates.append(p)

        # InHumanHand, InRobotHand
        # TODO InRobotHand missing, need to check both arms
        for object in self.objects:
            try:
                human_relative = self.tfl.lookupTransform('/human/wrist', object, rospy.Time(0))
                #robot_relative = self.tfl.lookupTransform('', o2, rospy.Time(0))
            except:
                pass
            else:
                d = transformations.norm(human_relative)
                if d<self.config['in_human_hand_tolerance']:
                    p = Predicate()
                    p.type = Predicate.HOLDBYHUMAN
                    p.objects = [object]
                    self.predicates.append(p)


    def handle_request(self, req):
        self.predicates = []
        self.update_state()
        resp = GetSceneStateResponse()
        resp.state.header.stamp = rospy.Time.now()
        resp.state.predicates = self.predicates
        return resp

    def run(self):
        s = rospy.Service('/thr/scene_state', GetSceneState, self.handle_request)
        rospy.loginfo('[SceneStateManager] server ready to track {}...'.format(str(self.objects)))
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('scene_state_manager')
    SceneStateManager().run()