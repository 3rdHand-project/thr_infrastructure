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
        self.action_history_name = '/thr/action_history'
        self.service_update_name = '/thr/update_relational_state'
        self.logs = []
        self.running = False

        # Action History
        # Stores some info about previously executed actions, useful to produce the predicates AT_HOME, BUSY, HELD, PICKED
        self.at_home = {'left': True, 'right': True}
        self.busy = {'left': False, 'right': False}
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

        self.tfl = tf.TransformListener(True, rospy.Duration(5*60)) # TF Interpolation ON and duration of its cache = 5 minutes
        self.image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
        rospy.Subscriber(self.action_history_name, ActionHistoryEvent, self.cb_action_event_received)
        rospy.Service(self.service_update_name, UpdateRelationalState, self.cb_update_relational_state)

        self.start_stop_service_name = '/thr/scene_state_manager/start_stop'
        rospy.Service(self.start_stop_service_name, StartStopEpisode, self.cb_start_stop)

    def cb_start_stop(self, request):
        if request.command == StartStopEpisodeRequest.START:
            with self.state_lock:
                self.persistent_predicates = []
                self.picked = []
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

                    # Listening action events for predicate PICKED
                    if msg.type==ActionHistoryEvent.FINISHED_SUCCESS and msg.action.type=='pick':
                        self.picked = msg.action.parameters
                    elif msg.type==ActionHistoryEvent.FINISHED_SUCCESS and msg.action.type=='give':
                        self.picked = []

                    # Listening action events for activity predicates
                    if msg.type==ActionHistoryEvent.STARTING:
                        self.activity[self.abilities[msg.action.type]] = msg.action
                    else:
                        self.activity[self.abilities[msg.action.type]] = None

    def pred_picked(self, obj):
        return obj in self.picked

    def pred_at_home(self, side):
        return self.at_home[side]

    def pred_busy(self, side):
        return self.busy[side]

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
