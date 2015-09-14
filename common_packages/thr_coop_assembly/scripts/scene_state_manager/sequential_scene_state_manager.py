#!/usr/bin/env python

import rospy, rospkg, tf, transformations
from thr_coop_assembly.srv import GetSceneState, GetSceneStateResponse
from thr_coop_assembly.msg import SceneState, Predicate
from itertools import combinations
from threading import Lock
import json, cv2, cv_bridge
from numpy import zeros, uint8
from time import time
from sensor_msgs.msg import Image
from copy import deepcopy

class SequentialSceneStateManager(object):
    def __init__(self, rate):
        self.state = SceneState()
        self.rate = rospy.Rate(rate)
        self.world = "base"
        self.screwdriver = '/tools/screwdriver'
        self.state_lock = Lock()
        self.attached = set()
        self.attaching_stamps = {}
        self.old_state = None
        self.old_display_state = None
        self.logs = {}

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
        self.tfl = tf.TransformListener(True, rospy.Duration(5*60)) # TF Interpolation ON and duration of its cache = 5 minutes
        self.image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)


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
            # WARNING: Do not ask the relative tf directly, it is outdated!
            tf_slave = self.tfl.lookupTransform(self.world, slave, rospy.Time(0))
            tf_master = self.tfl.lookupTransform(self.world, master, rospy.Time(0))
        except Exception, e:
            pass
        else:
            relative = transformations.multiply_transform(transformations.inverse_transform(tf_master), tf_slave)
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
                # WARNING: Do not ask the relative tf directly, it is outdated!
                screwdriver = self.tfl.lookupTransform(self.world, self.screwdriver, rospy.Time(0))
                tf_master = self.tfl.lookupTransform(self.world, master, rospy.Time(0))
            except:
                rospy.logwarn("screwdriver or {} not found, I consider that you're NOT attaching {} and {}".format(master, master, slave))
            else:
                relative = transformations.multiply_transform(transformations.inverse_transform(tf_master), screwdriver)
                cart_dist = transformations.distance(relative, self.poses[master]['constraints'][atp][self.screwdriver])
                if cart_dist<self.config['tool_position_tolerance']:
                    try:
                        if time()-self.attaching_stamps[master][slave]>self.config['screwdriver_attaching_time']:
                            rospy.logwarn("[Scene state manager] User has attached {} and {}".format(master, slave))
                            self.attached.append(master+slave+str(atp))
                            return True
                    except KeyError:
                        if not self.attaching_stamps.has_key(master):
                            self.attaching_stamps[master] = {}
                        self.attaching_stamps[master][slave] = time()
                        return False
        return False

    def pred_in_human_ws(self, obj):
        try:
            return transformations.norm(self.tfl.lookupTransform(obj, "/table", rospy.Time(0)))<self.config['in_human_ws_distance']
        except:
            return False

    def handle_request(self, req):
        resp = GetSceneStateResponse()
        with self.state_lock:
            resp.state = self.state # TODO deepcopy?
        return resp

    def display_image(self, width, height):
        with self.state_lock:
            refresh = not self.old_display_state or self.state.predicates != self.old_display_state.predicates
        if refresh:
            img = zeros((height,width, 3), uint8)
            preds = {"attached": [], "in_hws": [], "positioned": []}
            with self.state_lock:
                for p in self.state.predicates:
                    if p.type=='in_human_ws':
                        preds["in_hws"].append(p)
                    elif p.type=='positioned':
                        preds["positioned"].append(p)
                    elif p.type=='attached':
                        preds["attached"].append(p)

            # Now draw the image with opencv
            line = 1
            for i_pred, pred in preds.iteritems():
                cv2.putText(img, '#'+i_pred.upper()+' ['+str(len(pred))+']', (10, 20*line), cv2.FONT_HERSHEY_SIMPLEX, 0.55, [255]*3)
                line+=1
                for i, p in enumerate(pred):
                    cv2.putText(img, str(p.parameters), (50, 20*line), cv2.FONT_HERSHEY_SIMPLEX, 0.5, [180]*3)
                    line += 1
            #cv2.imshow("Predicates", img)
            #cv2.waitKey(1)
            msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
            self.image_pub.publish(msg)
            self.old_display_state = deepcopy(self.state)

    def record_state(self):
        with self.state_lock:
            if not self.old_state or self.state.predicates != self.old_state.predicates:
                predicates = []
                for p in self.state.predicates:
                    predicates.append({'type': p.type, 'parameters': p.parameters})
                self.logs.append({'timestamp': rospy.get_time(),
                                  'scene': predicates })
                self.old_state = deepcopy(self.state)

    def run(self):
        while not rospy.is_shutdown():
            with self.state_lock:
                self.state.predicates = []
                self.state.header.stamp = rospy.Time.now()
                for o in self.objects:
                    if self.pred_in_human_ws(o):
                        p = Predicate()
                        p.type = 'in_human_ws'
                        p.parameters = [o]
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
                            p.type = 'positioned'
                            p.parameters = [master, slave, str(atp)]
                            self.state.predicates.append(p)
                        if self.pred_attached(master, slave, atp):
                            p = Predicate()
                            p.type = 'attached'
                            p.parameters = [master, slave, str(atp)]
                            self.state.predicates.append(p)
            display = rospy.get_param('/thr/display')
            if display == "debug":
                self.display_image(1024, 600)
            self.rate.sleep()

        logs_name = rospy.get_param('/thr/logs_name')
        if logs_name != "none":
            with open('scenes_'+logs_name+'.json', 'w') as f:
                json.dump(self.logs, f)

    def start(self):
        rospy.Service('/thr/scene_state', GetSceneState, self.handle_request)
        rospy.loginfo('[SceneStateManager] server ready to track {}...'.format(str(self.objects)))
        self.run()

if __name__ == "__main__":
    rospy.init_node('sequential_scene_state_manager')
    SequentialSceneStateManager(20).start()
