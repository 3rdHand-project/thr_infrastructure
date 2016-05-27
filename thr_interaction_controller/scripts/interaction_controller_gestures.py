#!/usr/bin/env python
import os
import rospy
import rospkg
import actionlib
import json
from random import choice
from threading import Lock

from thr_infrastructure_msgs.msg import *
from thr_infrastructure_msgs.srv import *
from actionlib_msgs.msg import *
from collections import deque
from kinect2.client import Kinect2Client

class InteractionController(object):
    def __init__(self):
        self.lock = Lock()
        self.running = True
        self.waiting = False
        self.last_predicition_confidence = None
        self.last_predicted_decision_str = None
        self.last_human_decision = None

        self.current_scene = None
        self.last_scene = None

        self.logs = []

        # Kinect controls
        self.kinect = Kinect2Client('BAXTERFLOWERS.local')
        self.last_skeleton = None
        self.skeleton_id = ''
        self.hand_state = {'filtered_state': ''}
        self.speech = deque()
        self.speech_lock = Lock()
        self.last_sentence = ''
        self.last_speak = rospy.Time(0)

        # Parameters to be tweaked
        self.interaction_loop_rate = rospy.Rate(1)
        self.reward_service = '/thr/learner'
        self.predictor_service = 'thr/predictor'
        self.scene_state_service = '/thr/scene_state'
        self.run_decision_name = '/thr/run_decision'

        # Initiating topics ands links to services/actions
        self.run_decision_client = actionlib.SimpleActionClient(self.run_decision_name, RunDecisionAction)
        rospy.loginfo("Waiting action client {}...".format(self.run_decision_name))
        self.run_decision_client.wait_for_server()
        for service in [self.reward_service, self.predictor_service, self.scene_state_service]:
            rospy.loginfo("Waiting service {}...".format(service))
            rospy.wait_for_service(service)

        self.rospack = rospkg.RosPack()

        with open(self.rospack.get_path("thr_action_server")+"/config/decision_action_mapping.json") as config_file:
            self.decision_action_mapping = json.load(config_file)

        rospy.loginfo("Starting Kinect 2 services...")
        self.start_kinect_services()

        rospy.loginfo("IC for interaction via gestures ready!")

    #################################################
    # KINECT SERVICES ###############################
    #################################################

    def say(self, sentence, interval=15):
        if self.last_sentence != sentence or rospy.get_time() - self.last_speak > interval:
            self.kinect.tts.say(sentence)
            self.last_sentence = sentence
            self.last_speak = rospy.get_time()

    def cb_skeleton(self, msg):
        num_skeletons = len(msg.keys())
        if self.skeleton_id == '':
            if num_skeletons > 1:
                rospy.logwarn("{} skeletons are visible, ignoring frame".format(num_skeletons))
            elif num_skeletons == 0:
                rospy.logwarn("No skeleton visible, ignoring frame")
            else:
                self.skeleton_id = msg.keys()[0]
                rospy.loginfo("Selected skeleton {}".format(self.skeleton_id))

        if num_skeletons > 0 and self.skeleton_id != '':
            try:
                self.last_skeleton = msg[self.skeleton_id]
            except KeyError:
                # Skeleton ID has changed
                rospy.logwarn("Skeleton ID has changed")
                self.say("I'm tracking someone else")
                self.skeleton_id = ''
                self.last_skeleton = None

        ####### Hand state filtering
        # If you're looking for the end filtered state this is self.hand_state['filtered_state']
        activating_duration = 3
        stopping_duration = 2
        timeout_duration = 3.5  # The gesture expires within 3.5 sec if it hasn't been activated, must be >  other durations
        if self.last_skeleton is not None:
            hand_state = self.last_skeleton['HandRight']['HandState']
            if hand_state not in self.hand_state:
                self.hand_state[hand_state] = {}
            if 'filtered_state' not in self.hand_state:
                self.hand_state['filtered_state'] = ''
            if hand_state in ['NotTracked', 'Unknown'] or self.hand_state['filtered_state'] != '' and hand_state != self.hand_state['filtered_state']:
                if self.hand_state['filtered_state'] != "":
                    if 'disappeared' not in self.hand_state[self.hand_state['filtered_state']]:
                        self.hand_state[self.hand_state['filtered_state']]['disappeared'] = rospy.get_time()
                    elif rospy.get_time() - self.hand_state[self.hand_state['filtered_state']]['disappeared'] > stopping_duration:
                        del self.hand_state[self.hand_state['filtered_state']]['disappeared']
                        self.hand_state['filtered_state'] = ""
                        rospy.loginfo("Switch to {}".format(self.hand_state['filtered_state']))
            else:
                if self.hand_state['filtered_state'] == "":
                    if 'appeared' not in self.hand_state[hand_state]:
                        self.hand_state[hand_state]['appeared'] = rospy.get_time()
                    elif rospy.get_time() - self.hand_state[hand_state]['appeared'] > activating_duration:
                        self.hand_state['filtered_state'] = hand_state
                        rospy.loginfo("Switch to {}".format(self.hand_state['filtered_state']))
                        del self.hand_state[hand_state]['appeared']

            for gesture in self.hand_state:
                if 'appeared' in self.hand_state[gesture] and rospy.get_time() - self.hand_state[gesture]['appeared'] > timeout_duration:
                    del self.hand_state[gesture]['appeared']
                if 'disappeared' in self.hand_state[gesture] and rospy.get_time() - self.hand_state[gesture]['disappeared'] > timeout_duration:
                    del self.hand_state[gesture]['disappeared']

    def cb_speech(self, msg):
        with self.speech_lock:
            try:
                self.speech.append(msg['semantics'])
            except KeyError as e:
                rospy.logerr("Malformed speech message: no key {}".format(e.message))

    def start_kinect_services(self):
        self.kinect.skeleton.set_callback(self.cb_skeleton)
        self.kinect.speech.set_callback(self.cb_speech)
        with open('{}/config/{}/grammar_en.xml'.format(self.rospack.get_path('thr_scenes'), rospy.get_param('/thr/scene'))) as f:
            self.kinect.speech.params.set_grammar(f, "Grammar {}".format(rospy.get_param('/thr/scene')))
        self.kinect.tts.params.set_language('english')
        self.kinect.speech.params.semantic_on()
        self.kinect.tts.params.queue_on()
        msg = self.kinect.skeleton.start()
        msg += self.kinect.tts.start()
        msg += self.kinect.speech.start()
        if len(msg) > 0:
            rospy.logerr(msg)

    #################################################
    # SERVICE CALLERS ###############################
    #################################################

    def set_new_training_example(self, scene, decision, prediction_confidence, predicted_action, corrected):
        request = SetNewTrainingExampleRequest()
        request.decision = decision
        request.predicted_decision = predicted_action
        request.scene_state = scene
        request.prediction_confidence = prediction_confidence
        request.corrected = corrected

        try:
            reward = rospy.ServiceProxy(self.reward_service, SetNewTrainingExample)
            reward(request)
        except rospy.ServiceException as e:
            rospy.logerr("Cannot set training example: {}".format(e.message))

    def update_scene(self):
        request = GetSceneStateRequest()
        try:
            getscene = rospy.ServiceProxy(self.scene_state_service, GetSceneState)
            self.last_scene = self.current_scene
            self.current_scene = getscene(request).state
        except rospy.ServiceException as e:
            rospy.logerr("Cannot update scene {}:".format(e.message))

    def predict(self):
        request = GetNextDecisionRequest()
        request.scene_state = self.current_scene
        try:
            predict = rospy.ServiceProxy(self.predictor_service, GetNextDecision)
            return predict(request)
        except rospy.ServiceException as e:
            rospy.logerr("Cannot call predictor:".format(e.message))
            decision = Decision(type='wait')
            return GetNextDecisionResponse(decisions=[decision], probas=[1.])

    def start_or_stop_episode(self, start=True):
        for node in ['scene_state_manager', 'scene_state_updater', 'action_server', 'learner_predictor']:
            url = '/thr/{}/start_stop'.format(node)
            rospy.wait_for_service(url)
            rospy.ServiceProxy(url, StartStopEpisode).call(StartStopEpisodeRequest(
                command=StartStopEpisodeRequest.START if start else
                StartStopEpisodeRequest.STOP))

    ###################################################################################################################

    def process_speech(self):
        with self.speech_lock:
            decisions = [Decision(type=speech[0], parameters=speech[1:]) for speech in self.speech]
            self.speech.clear()
        return decisions

    def run_decision(self, decision):
        if decision.type == 'wait':
            self.waiting = True
            return
        os.system('beep')
        goal = RunDecisionGoal()
        goal.decision = decision
        self.run_decision_client.send_goal(goal)
        while self.run_decision_client.get_state() in [GoalStatus.PENDING, GoalStatus.ACTIVE] and not rospy.is_shutdown():
            self.interaction_loop_rate.sleep()
        self.current_decision = decision
        return False

    def run(self):
        def decision_to_tts(decision):
            obj = decision.parameters[0].split('/')[-1].replace('_', ' ')
            action = decision.type.split('_')[-1]
            return action, obj

        rospy.loginfo('Manual interaction starting from gestures!')
        self.start_or_stop_episode(True)

        def filter(actions, action):
            return [decision for index, decision in enumerate(actions.decisions) if
                        action in decision.type and all_decisions.probas[index] > 0.]

        if not rospy.is_shutdown():
            try:
                while self.running and not rospy.is_shutdown():
                    self.update_scene()
                    all_decisions = self.predict()

                    go_homes = filter(all_decisions, 'go_home')
                    if len(go_homes) > 0:
                        decision = choice(go_homes)
                    else:
                        gives = filter(all_decisions, 'give')
                        picks = filter(all_decisions, 'pick')
                        holds = filter(all_decisions, 'hold')
                        if len(gives) > 0:
                            decision = choice(gives)
                            decision.parameters = decision.parameters #+ map(str, flattened_gesture_pose) + [
                        else:
                            self.say("Please tell or show me what to do")
                            gesture = self.hand_state['filtered_state']
                            wait = False

                            speech_decisions = self.process_speech()
                            if len(speech_decisions) > 0:
                                if speech_decisions[0] in holds or speech_decisions[0] in picks:
                                    decision = speech_decisions[0]
                                else:
                                    action, obj = decision_to_tts(speech_decisions[0])
                                    self.say("I cannot {} {} now".format(action, obj))
                                    wait = True
                            else:
                                if gesture == 'Lasso':
                                    if len(holds) > 0:
                                        decision = choice(holds)
                                    else:
                                        self.say("I cannot hold now")
                                        wait = True
                                elif gesture == 'Open':
                                    if len(picks) > 0:
                                        decision = choice(picks)
                                    else:
                                        rospy.logwarn("I cannot pick now")
                                        wait = True
                                else:
                                    wait = True

                            if wait:
                                self.interaction_loop_rate.sleep()
                                continue

                            rospy.logwarn("You showed a {} gesture corresponding to a {} action".format(gesture, decision.type))
                            action, obj = decision_to_tts(decision)
                            self.say("I'm {}ing {}".format(action, obj))

                    type, params = decision.type, decision.parameters
                    rospy.logwarn("Choosing decision {}{}".format(type, params))

                    self.logs.append({'timestamp': rospy.get_time(),
                                      'type': type,
                                      'parameters': params})
                    self.run_decision(decision)
                    self.interaction_loop_rate.sleep()
            finally:
                logs_name = rospy.get_param('/thr/logs_name')
                if logs_name != "none":
                    with open('action_decisions_' + logs_name + '.json', 'w') as f:
                        json.dump(self.logs, f)

if __name__ == '__main__':
    rospy.init_node("interaction_controller")
    InteractionController().run()
