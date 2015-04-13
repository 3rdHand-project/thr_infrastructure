#!/usr/bin/env python

import rospy, rospkg, tf, transformations
from thr_coop_assembly.srv import GetSceneState, GetSceneStateResponse
from thr_coop_assembly.msg import SceneState, Predicate
from itertools import combinations
from threading import Lock
import json

class SceneStateManager(object):
    def __init__(self, rate):
        self.state = SceneState()
        self.rate = rospy.Rate(rate)
        self.world = "base"
        self.state_lock = Lock()
        self.screwdriver_close_to = {}  # dict {object: [timestamp_first_seen, timestamp_last_seen], ...} storing the timestamps where each object as been seen close to the screwdriver

        try:
            self.objects = rospy.get_param('/thr/objects')[rospy.get_param('/thr/scene')]
        except KeyError:
            raise KeyError("Unable to read /thr/objects, have you used roslaunch to start this script?")
        self.scene = rospy.get_param('/thr/scene')

        self.rospack = rospkg.RosPack()
        with open(self.rospack.get_path("thr_coop_assembly")+"/config/constraints.json") as f:
            self.constraints = json.load(f)[self.scene]

        with open(self.rospack.get_path("thr_coop_assembly")+"/config/perception.json") as f:
            self.config = json.load(f)

        self.attached = [] # Pairs of attached objects on the form o1_o2 with o1<o2
        self.tfl = tf.TransformListener()

    def pred_attached(self, o1, o2):
        return False

    def pred_positioned(self, o1, o2):
        """
        :param o1: name of the first object SUCH AS o1 < o2 !!!
        :param o2: name of the second object SUCH AS o1 < o2 !!!
        :return: True if predicate POSITIONED(o1, o2) is True
        """
        try:
            relative = self.tfl.lookupTransform(o1, o2, rospy.Time(0))
        except:
            pass
        else:
            key = o1+'_'+o2 if o1<o2 else o2+'_'+o1
            cart_dist = transformations.distance(self.constraints[key], relative)
            # TODO this measures only cartesian distance, missing quaternions
            return cart_dist<self.config['position_tolerance']
        return False

    def pred_attached(self, o1, o2):
        attached = False
        duration = rospy.Duration(self.config['time_working_screwdriver'])
        if self.pred_positioned(o1, o2):
            if self.screwdriver_close_to.has_key(o1):
                attached = self.screwdriver_close_to[o1][1]-self.screwdriver_close_to[o1][0]>duration
            if not attached and self.screwdriver_close_to.has_key(o2):
                attached =  self.screwdriver_close_to[o2][1]-self.screwdriver_close_to[o2][0]>duration
        return attached

    def pred_in_human_ws(self, obj):
        return False

    def handle_request(self, req):
        resp = GetSceneStateResponse()
        self.state_lock.acquire()
        try:
            resp.state = self.state # TODO deepcopy?
        finally:
            self.state_lock.release()
        return resp

    def run(self):
        while not rospy.is_shutdown():
            for o in self.objects:
                try:
                    dist = transformations.norm(self.tfl.lookupTransform("/tools/screwdriver", o, rospy.Time(0)))
                except tf.LookupException:
                    pass
                else:
                    if dist<self.config['dist_working_screwdriver'] and not self.screwdriver_close_to.has_key(o):
                        # The screwdriver is inside the area for the first time
                        self.screwdriver_close_to[o] = [rospy.Time.now(), rospy.Time.now()]
                    elif dist<self.config['dist_working_screwdriver']:
                        # The screwdriver has already seen in the area in the past
                        self.screwdriver_close_to[o][1] = rospy.Time.now()
                    elif self.screwdriver_close_to.has_key(o):
                        # The screwdriver has just quit the area
                        del self.screwdriver_close_to[o]
                    else:
                        # The screwdriver is only not found
                        pass

            self.state_lock.acquire()
            try:
                self.state.predicates = []
                self.state.header.stamp = rospy.Time.now()
                for o1, o2 in combinations(self.objects, 2):
                    if self.pred_attached(o1, o2):
                        p = Predicate()
                        p.type = Predicate.ATTACHED
                        p.objects = [o1, o2]
                        self.state.predicates.append(p)
                    if self.pred_positioned(o1, o2):
                        p = Predicate()
                        p.type = Predicate.POSITIONED
                        p.objects = [o1, o2]
                        self.state.predicates.append(p)
            finally:
                self.state_lock.release()
            print self.screwdriver_close_to
            self.rate.sleep()

    def start(self):
        rospy.Service('/thr/scene_state', GetSceneState, self.handle_request)
        rospy.loginfo('[SceneStateManager] server ready to track {}...'.format(str(self.objects)))
        self.run()

if __name__ == "__main__":
    rospy.init_node('scene_state_manager')
    SceneStateManager(2).start()