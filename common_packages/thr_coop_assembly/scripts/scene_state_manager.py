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
        self.screwdriver = '/tools/screwdriver'
        self.state_lock = Lock()
        self.attached = set()

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

    def pred_positioned(self, master, slave, atp):
        """
        Checks if any constraint between master and slave exists at attach point atp and returns True if the constraint is within the tolerance
        :param master:
        :param slave:
        :param atp: (int)
        :return: True if predicate POSITIONED(master, slave, atp) is True
        """
        if master+slave+str(atp) in self.attached:
            return True
        try:
            relative = self.tfl.lookupTransform(master, slave, rospy.Time(0))
        except:
            pass
        else:
            constraint = self.poses[master]['constraints'][atp][slave]
            cart_dist = transformations.distance(constraint, relative)
            quat_dist = transformations.distance_quat(constraint, relative)
            return cart_dist<self.config['position_tolerance'] and quat_dist<self.config['orientation_tolerance']
        return False

    def pred_attached(self, master, slave, atp):
        if master+slave+str(atp) in self.attached:
            return True
        elif self.pred_positioned(master, slave, atp):
            try:
                relative = self.tfl.lookupTransform(master, self.screwdriver, rospy.Time(0))
            except:
                pass
            else:
                cart_dist = transformations.distance(relative, self.poses[master]['constraints'][atp][self.screwdriver])
                if cart_dist<self.config['tool_position_tolerance']:
                    self.attached.append(master+slave+str(atp))
                    return True
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
            self.state_lock.acquire()
            try:
                self.state.predicates = []
                self.state.header.stamp = rospy.Time.now()
                for o in self.objects:
                    if self.pred_in_human_ws(o):
                        p = Predicate()
                        p.type = Predicate.IN_HUMAN_WS
                        p.objects = [o]
                        self.state.predicates.append(p)
                for o1, o2 in combinations(self.objects, 2):
                    if self.poses[o1].has_key('constraints') and len([c for c in self.poses[o1]['constraints'] if o2 in c])>0:
                        master = o1
                        slave = o2
                    elif self.poses[o2].has_key('constraints') and len([c for c in self.poses[o2]['constraints'] if o1 in c])>0:
                        slave = o1
                        master = o2
                    else:
                        continue
                    for atp in range(len(self.poses[master]['constraints'])):
                        if self.pred_positioned(master, slave, atp):
                            p = Predicate()
                            p.type = Predicate.POSITIONED
                            p.objects = [master, slave, str(atp)]
                            self.state.predicates.append(p)
                        if self.pred_attached(master, slave, atp):
                            p = Predicate()
                            p.type = Predicate.ATTACHED
                            p.objects = [master, slave, str(atp)]
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