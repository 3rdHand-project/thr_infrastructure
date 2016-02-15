#!/usr/bin/env python

import rospy, rospkg, tf, transformations
from thr_infrastructure_msgs.srv import GetSceneState, GetSceneStateResponse, UpdateRelationalState, UpdateRelationalStateResponse,\
    UpdateRelationalStateRequest, StartStopEpisode, StartStopEpisodeRequest, StartStopEpisodeResponse
from thr_infrastructure_msgs.msg import SceneState, Predicate, ActionHistoryEvent
from itertools import combinations
from threading import Lock
import json
from sensor_msgs.msg import Image
from copy import deepcopy

class ConcurrentSceneStateManager(object):
    def __init__(self, rate):
        self.state = SceneState()
        self.old_state = None
        self.rate = rospy.Rate(rate)
        self.world = "base"
        self.screwdriver = '/tools/screwdriver'
        self.state_lock = Lock()
        self.history_lock = Lock()
        self.persistent_predicates = []
        self.attaching_stamps = {}
        self.action_history_name = '/thr/action_history'
        self.service_update_name = '/thr/update_relational_state'
        self.logs = []
        self.running = False

        # Action History
        # Stores some info about previously executed actions, useful to produce the predicates AT_HOME, BUSY, HELD, PICKED
        self.at_home = {'left': True, 'right': True}
        self.busy = {'left': False, 'right': False}
        self.held = []
        self.picked = []
        self.activity = {'left': None, 'right': None}

        try:
            self.objects = rospy.get_param('/thr/objects')[rospy.get_param('/thr/scene')]
        except KeyError:
            raise KeyError("Unable to read /thr/objects, have you used roslaunch to start this script?")
        self.scene = rospy.get_param('/thr/scene')

        self.rospack = rospkg.RosPack()
        with open(self.rospack.get_path("thr_scenes")+"/config/"+self.scene+"/poses.json") as f:
            self.poses = json.load(f)

        with open(self.rospack.get_path("thr_scene_state_manager")+"/config/perception.json") as f:
            self.config = json.load(f)

        with open(self.rospack.get_path("thr_action_server")+"/config/abilities.json") as f:
            self.abilities = json.load(f)

        self.attached = [] # Pairs of attached objects on the form o1_o2 with o1<o2
        self.screwed = [] # Pairs of screwed objects (screwdriver 7 seconds => screwed ; screw + wrist > 0.6 m => attached)
        self.tfl = tf.TransformListener(True, rospy.Duration(5*60)) # TF Interpolation ON and duration of its cache = 5 minutes
        self.image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
        rospy.Subscriber(self.action_history_name, ActionHistoryEvent, self.cb_action_event_received)
        rospy.Service(self.service_update_name, UpdateRelationalState, self.cb_update_relational_state)

        self.start_stop_service_name = '/thr/scene_state_manager/start_stop'
        rospy.Service(self.start_stop_service_name, StartStopEpisode, self.cb_start_stop)

    def cb_start_stop(self, request):
        if request.command == StartStopEpisodeRequest.START:
            with self.state_lock:
                self.attached = []
                self.attaching_stamps = {}
                self.persistent_predicates = []
                self.picked = []
                self.held = []
                self.at_home['left'] = True
                self.at_home['right'] = True
                self.busy['left'] = False
                self.busy['right'] = False
                self.running = True

        elif request.command == StartStopEpisodeRequest.STOP:
            self.running = False
        return StartStopEpisodeResponse()

    def cb_update_relational_state(self, request):
        if not self.running:
            return UpdateRelationalStateResponse(success=False)
        with self.state_lock:
            if request.command == UpdateRelationalStateRequest.ADD:
                if request.predicate in self.persistent_predicates:
                    return UpdateRelationalStateResponse(success=False)
                else:
                    self.persistent_predicates.append(request.predicate)
                    return UpdateRelationalStateResponse(success=True)

            elif request.command == UpdateRelationalStateRequest.REMOVE:
                if request.predicate in self.persistent_predicates:
                    self.persistent_predicates.remove(request.predicate)
                    return UpdateRelationalStateResponse(success=True)
                else:
                    return UpdateRelationalStateResponse(success=False)

    def cb_action_event_received(self, msg):
            with self.state_lock:
            # Listening action history for predicate AT_HOME
                if msg.side in ['left', 'right']:
                    if msg.type==ActionHistoryEvent.FINISHED_SUCCESS and msg.action.type=='go_home_left':
                        self.at_home['left'] = True
                    elif msg.type==ActionHistoryEvent.FINISHED_SUCCESS and msg.action.type=='go_home_right':
                        self.at_home['right'] = True
                    elif msg.type==ActionHistoryEvent.STARTING and self.abilities[msg.action.type]=='right' and msg.action.type!='go_home_right':
                        self.at_home['right'] = False
                    elif msg.type==ActionHistoryEvent.STARTING and self.abilities[msg.action.type]=='left' and msg.action.type!='go_home_left':
                        self.at_home['left'] = False

                    # Listening action events for predicate BUSY
                    if self.abilities[msg.action.type]=='left':
                        self.busy['left'] = msg.type==ActionHistoryEvent.STARTING
                    elif self.abilities[msg.action.type]=='right':
                        self.busy['right'] = msg.type==ActionHistoryEvent.STARTING
                    else:
                        rospy.logerr("[Scene state manager] No arm is capable of {}{}, event ignored".format(msg.action.type, str(msg.action.parameters)))

                    # Listening action events for predicates PICKED + HELD
                    if msg.type==ActionHistoryEvent.STARTING and msg.action.type=='hold':
                        self.held = msg.action.parameters
                    elif msg.type==ActionHistoryEvent.FINISHED_SUCCESS and msg.action.type=='pick':
                        self.picked = msg.action.parameters
                    elif msg.type==ActionHistoryEvent.FINISHED_SUCCESS and msg.action.type=='hold':
                        self.held = []
                    elif msg.type==ActionHistoryEvent.FINISHED_SUCCESS and msg.action.type=='give':
                        self.picked = []

                    # Listening action events for activity predicates
                    if msg.type==ActionHistoryEvent.STARTING:
                        self.activity[self.abilities[msg.action.type]] = msg.action
                    else:
                        self.activity[self.abilities[msg.action.type]] = None

    def pred_held(self, obj):
        #with self.history_lock:
            return obj in self.held

    def pred_picked(self, obj):
        #with self.history_lock:
            return obj in self.picked

    def pred_at_home(self, side):
        #with self.history_lock:
            return self.at_home[side]

    def pred_busy(self, side):
        #with self.history_lock:
            return self.busy[side]

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
            return cart_dist<self.config['positioned']['position_tolerance'] and quat_dist<self.config['positioned']['orientation_tolerance']
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
                pass
            else:
                if self.screwdriver in self.poses[master]['constraints'][atp]:  # For objects that need to be screwed
                    relative = transformations.multiply_transform(transformations.inverse_transform(tf_master), screwdriver)
                    cart_dist = transformations.distance(relative, self.poses[master]['constraints'][atp][self.screwdriver])
                    if cart_dist < self.config['attached']['tool_position_tolerance']:
                        try:
                            if rospy.Time.now() - self.attaching_stamps[master][slave] > rospy.Duration(self.config['attached']['screwdriver_attaching_time']):
                                rospy.logwarn("[Scene state manager] User has attached {} and {}".format(master, slave))
                                # self.screwed.append(master+slave+str(atp))
                                self.attached.append(master+slave+str(atp))
                        except KeyError:
                            if not self.attaching_stamps.has_key(master):
                                self.attaching_stamps[master] = {}
                            self.attaching_stamps[master][slave] = rospy.Time.now()
                else: # For objects that only need to be inserted
                    # self.screwed.append(master+slave+str(atp))
                    self.attached.append(master+slave+str(atp))
        return False

    def record_state(self):
        with self.state_lock:
            if not self.old_state or self.state.predicates != self.old_state.predicates:
                predicates = []
                for p in self.state.predicates:
                    predicates.append({'type': p.type, 'parameters': p.parameters})
                self.logs.append({'timestamp': rospy.get_time(),
                                  'scene': predicates })
                self.old_state = deepcopy(self.state)

    def pred_in_human_ws(self, obj):
        try:
            return rospy.Time.now() - self.tfl.getLatestCommonTime(obj, "/table") < rospy.Duration(self.config['in_human_ws']['in_human_ws_time']) \
                    and transformations.norm(self.tfl.lookupTransform(obj, "/table", rospy.Time(0)))<self.config['in_human_ws']['in_human_ws_distance']
        except:
            return False

    def cb_scene_state(self, req):
        with self.state_lock:
            return GetSceneStateResponse(self.state)

    def run(self):
        while not rospy.is_shutdown():
            if self.running:
                with self.state_lock:
                    self.state.predicates = [] + self.persistent_predicates
                    self.state.header.stamp = rospy.Time.now()
                    for o in self.objects:
                        if self.pred_in_human_ws(o):
                            p = Predicate()
                            p.type = 'in_human_ws'
                            p.parameters = [o]
                            self.state.predicates.append(p)
                        elif self.pred_picked(o):
                            p = Predicate()
                            p.type = 'picked'
                            p.parameters = [o]
                            self.state.predicates.append(p)
                        elif self.pred_picked(o):
                            p = Predicate()
                            p.type = 'held'
                            p.parameters = [o]
                            self.state.predicates.append(p)
                    for o1, o2 in combinations(self.objects, 2):
                        if o1 in self.poses and 'constraints' in self.poses[o1] and len([c for c in self.poses[o1]['constraints'] if o2 in c])>0:
                            master = o1
                            slave = o2
                        elif o2 in self.poses and 'constraints' in self.poses[o2] and len([c for c in self.poses[o2]['constraints'] if o1 in c])>0:
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
                    with self.history_lock:
                        for side in ['left', 'right']:
                            if self.pred_busy(side):
                                p = Predicate()
                                p.type = 'busy'
                                p.parameters.append(side)
                                self.state.predicates.append(p)
                            if self.pred_at_home(side):
                                p = Predicate()
                                p.type = 'at_home'
                                p.parameters.append(side)
                                self.state.predicates.append(p)
                            if self.activity[side] is not None:
                                p = Predicate()
                                p.type = self.activity[side].type
                                p.parameters = deepcopy(self.activity[side].parameters)
                                p.parameters.append('eq2' if p.type=='hold' else 'eq1')
                                self.state.predicates.append(p)

                self.record_state()
            self.rate.sleep()

        logs_name = rospy.get_param('/thr/logs_name')
        if logs_name != "none":
            with open('scenes_'+logs_name+'.json', 'w') as f:
                json.dump(self.logs, f)

    def start(self):
        rospy.Service('/thr/scene_state', GetSceneState, self.cb_scene_state)
        rospy.loginfo('[SceneStateManager] server ready to track {}...'.format(str(self.objects)))
        self.run()

if __name__ == "__main__":
    rospy.init_node('concurrent_scene_state_manager')
    ConcurrentSceneStateManager(20).start()