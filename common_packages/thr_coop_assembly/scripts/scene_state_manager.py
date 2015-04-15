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
        with open(self.rospack.get_path("thr_coop_assembly")+"/config/poses.json") as f:
            self.poses = json.load(f) # TODO separate with scene name [self.scene]

        with open(self.rospack.get_path("thr_coop_assembly")+"/config/perception.json") as f:
            self.config = json.load(f)

        self.attached = [] # Pairs of attached objects on the form o1_o2 with o1<o2
        self.tfl = tf.TransformListener()

    def pred_attached(self, o1, o2):
        return False

    def pred_positioned(self, o1, o2):
        """
        Checks if any constraint between o1 and o2 exists and returns True if at least one constraint is within the tolerance
        :return: True if predicate POSITIONED(o1, o2) is True
        """
        try:
            relative = self.tfl.lookupTransform(o1, o2, rospy.Time(0))
            inv_relative = self.tfl.lookupTransform(o2, o1, rospy.Time(0))
        except:
            pass
        else:
            possible_constraints = []
            possible_relative = []
            if self.poses[o1].has_key('constraints'):  # o1 is the master, o2 the slave, same than constraints
                for c in self.poses[o1]['constraints']:
                    if c.has_key(o2):
                        possible_constraints.append(c[o2])
                        possible_relative.append(relative)
            if self.poses[o2].has_key('constraints'): # o1 is the slave, o2 the master, the opposite of the constraints
                for c in self.poses[o2]['constraints']:
                    if c.has_key(o1):
                        possible_constraints.append(c[o1])
                        possible_relative.append(inv_relative)
            for c in range(len(possible_constraints)):
                cart_dist = transformations.distance(possible_constraints[c], possible_relative[c])
                quat_dist = transformations.distance_quat(possible_constraints[c], possible_relative[c])
                if cart_dist<self.config['position_tolerance'] and quat_dist<self.config['orientation_tolerance']:
                    return True
        return False

    def pred_attached(self, o1, o2):
        return False

    def pred_in_human_ws(self, obj):
        try:
            return transformations.norm(self.tfl.lookupTransform(obj, "/table", rospy.Time(0)))<self.config['in_human_ws_distance']
        except:
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
                except:
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
                for o in self.objects:
                    if self.pred_in_human_ws(o):
                        p = Predicate()
                        p.type = Predicate.IN_HUMAN_WS
                        p.objects = [o]
                        self.state.predicates.append(p)
            finally:
                self.state_lock.release()
            self.rate.sleep()

    def start(self):
        rospy.Service('/thr/scene_state', GetSceneState, self.handle_request)
        rospy.loginfo('[SceneStateManager] server ready to track {}...'.format(str(self.objects)))
        self.run()

if __name__ == "__main__":
    rospy.init_node('scene_state_manager')
    SceneStateManager(2).start()