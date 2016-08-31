#!/usr/bin/env python
import os
import rospy
import rospkg
import actionlib
import json
from thr_infrastructure_msgs.msg import *
from thr_infrastructure_msgs.srv import *
from actionlib_msgs.msg import *


class InteractionController(object):
    def __init__(self, interaction_rate=20):
        self.running = True
        self.current_scene = None

        self.logs = []

        # Parameters to be tweaked
        self.interaction_loop_rate = rospy.Rate(interaction_rate)
        self.reward_service = '/thr/learner'
        self.predictor_service = 'thr/predictor'
        self.scene_state_service = '/thr/scene_state'
        self.run_decision_name = '/thr/run_decision'
        self.action_history_name = '/thr/action_history'

        # Initiating topics ands links to services/actions
        self.run_decision_client = actionlib.SimpleActionClient(self.run_decision_name, RunDecisionAction)
        rospy.loginfo("Waiting action client {}...".format(self.run_decision_name))
        self.run_decision_client.wait_for_server()
        for service in [self.reward_service, self.predictor_service, self.scene_state_service]:
            rospy.loginfo("Waiting service {}...".format(service))
            rospy.wait_for_service(service)

        self.rospack = rospkg.RosPack()
        rospy.Subscriber(self.action_history_name, ActionHistoryEvent, self.cb_action_event_received)

        with open(self.rospack.get_path("thr_action_server")+"/config/decision_action_mapping.json") as config_file:
            self.decision_action_mapping = json.load(config_file)

    def cb_action_event_received(self, event):
        #if event.type in [ActionHistoryEvent.FINISHED_SUCCESS, ActionHistoryEvent.FINISHED_FAILURE]:
        #    self.set_new_training_example()
        pass

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
        except rospy.ServiceException as e:
            rospy.logerr("Cannot set training example: {}".format(e.message))
        else:
            reward(request)

    def update_scene(self):
        request = GetSceneStateRequest()
        try:
            getscene = rospy.ServiceProxy(self.scene_state_service, GetSceneState)
        except rospy.ServiceException as e:
            rospy.logerr("Cannot update scene {}:".format(e.message))
        else:
            self.current_scene = getscene(request).state

    def predict(self, current_scene):
        request = GetNextDecisionRequest()
        request.scene_state = current_scene
        try:
            predict = rospy.ServiceProxy(self.predictor_service, GetNextDecision)
        except rospy.ServiceException as e:
            rospy.logerr("Cannot call predictor:".format(e.message))
            decision = Decision(type='wait')
            return GetNextActionResponse(decisions=[decision], probas=[1.])
        else:
            return predict(request)

    @staticmethod
    def get_most_probable_decision(prediction):
        max_probability = -1
        most_probable_decision_id = 0
        for decision_id, probability in enumerate(prediction.probas):
            if probability > max_probability:
                max_probability = probability
                most_probable_decision_id = decision_id
        return prediction.decisions[most_probable_decision_id]

    @staticmethod
    def start_or_stop_episode(start=True):
        for node in ['scene_state_manager', 'scene_state_updater', 'action_server', 'learner_predictor']:
            url = '/thr/{}/start_stop'.format(node)
            rospy.wait_for_service(url)
            rospy.ServiceProxy(url, StartStopEpisode).call(StartStopEpisodeRequest(
                command=StartStopEpisodeRequest.START if start else
                StartStopEpisodeRequest.STOP))

    ###################################################################################################################
    def run_decision(self, decision):
        if decision.type == 'wait':
            return
        os.system('beep')
        goal = RunDecisionGoal()
        goal.decision = decision
        self.run_decision_client.send_goal(goal)
        while self.run_decision_client.get_state() in [GoalStatus.PENDING, GoalStatus.ACTIVE] and not rospy.is_shutdown():
            self.interaction_loop_rate.sleep()

    def init(self):
        rospy.loginfo('Interaction starting in independent mode, without GUI nor speech')
        rospy.set_param("/thr/paused", False)
        self.start_or_stop_episode(True)      # Start signal

    def loop(self):
        self.update_scene()
        prediction = self.predict(self.current_scene)
        decision = self.get_most_probable_decision(prediction)
        self.run_decision(decision)
        self.interaction_loop_rate.sleep()

    def run(self):
        try:
            self.init()
            while self.running and not rospy.is_shutdown():
                self.loop()
        finally:
            logs_name = rospy.get_param('/thr/logs_name')
            if logs_name != "none":
                with open('action_decisions_'+logs_name+'.json', 'w') as f:
                    json.dump(self.logs, f)


if __name__ == '__main__':
    rospy.init_node("interaction_controller")
    InteractionController().run()
