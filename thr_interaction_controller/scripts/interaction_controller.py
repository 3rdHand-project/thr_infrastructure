#!/usr/bin/env python
import os
import rospy
import rospkg
import actionlib
import numpy as np
from copy import deepcopy
import json

import web_asker

from thr_infrastructure_msgs.msg import *
from thr_infrastructure_msgs.srv import *
from actionlib_msgs.msg import *
from baxter_commander import Halo
from baxter_interface import Head


class HeadSignal():
    def __init__(self):
        self.head = Head()
        self.halo = Halo()
        self.halo.set_off()

    def request_attention(self, enabled=False):
        if enabled:
            self.halo.start_flashing()
            self.head.command_nod()
        else:
            self.halo.stop_flashing()

    def show_result(self, success=True):
        if success:
            self.halo.set_green()
        else:
            self.halo.set_red()

    def reset_signal(self):
        self.halo.set_off()


class ConfirmQuestion(object):
    def __init__(self, web_asker, action_str, decision_str_list, state, confidence, head):
        self.web_asker = web_asker
        self.decision_str = action_str
        self.decision_str_list = decision_str_list
        self.state = state
        self.confidence = confidence
        self.head = head
        self.head.request_attention(True)

        self.first = self.web_asker.ask("I think i should do {} :".format(self.decision_str),
                                        ["Do it!", "Don't do that"], color="blue")
        self.second = None
        self.correct_decision = None

    def update(self):
        if self.first is not None and self.first.answered():
            self.head.request_attention(False)
            if self.first.get_answer() == "Do it!":
                self.correct_decision = self.decision_str
            else:
                self.second = self.web_asker.ask("I wanted to do {}, What should I do?".format(self.decision_str),
                                                 self.decision_str_list, color="blue")
                self.first = None

        if self.second is not None and self.second.answered():
            self.correct_decision = self.second.get_answer()

    def is_answered(self):
        return self.correct_decision is not None

    def get_correct_decision(self):
        assert self.correct_decision is not None
        return self.correct_decision

    def get_predicted_decision(self):
        return self.decision_str

    def get_state(self):
        return self.state

    def remove(self):
        if self.first is not None:
            self.first.remove()
        if self.second is not None:
            self.second.remove()


class FeedbackQuestion(object):
    def __init__(self, web_asker, decision_str, decision_str_list, state, confidence, delete_key):
        self.web_asker = web_asker
        self.decision_str = decision_str
        self.decision_str_list = decision_str_list
        self.state = state
        self.confidence = confidence
        self.delete_key = delete_key

        self.first = self.web_asker.ask("I'm doing {} :".format(self.decision_str),
                                        ["Don't do that"])
        self.second = None
        self.correct_decision = None

    def update(self):
        if self.first is not None and self.first.answered():
            self.second = self.web_asker.ask("I did {}, What should have I done?".format(self.decision_str),
                                             self.decision_str_list)
            self.first.remove()
            self.first = None

        if self.second is not None and self.second.answered():
            self.correct_decision = self.second.get_answer()

    def is_answered(self):
        return self.correct_decision is not None

    def get_correct_decision(self):
        if self.correct_decision is not None:
            return self.correct_decision
        else:
            return self.decision_str

    def get_predicted_decision(self):
        return self.decision_str

    def get_state(self):
        return self.state

    def get_delete_key(self):
        return self.delete_key

    def is_user_engaged(self):
        return self.second is not None

    def remove(self):
        if self.first is not None:
            self.first.remove()
        if self.second is not None:
            self.second.remove()


class InteractionController(object):
    def __init__(self):
        self.running = True
        self.waiting = False
        self.last_predicition_confidence = None
        self.last_predicted_decision_str = None
        self.confirm_question = None
        self.feedback_question_list = []

        self.current_scene = None
        self.last_scene = None

        self.logs = []
        self.head = HeadSignal()

        # Parameters to be tweaked
        self.interaction_loop_rate = rospy.Rate(20)
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
        self.init_webasker()
        rospy.Subscriber(self.action_history_name, ActionHistoryEvent, self.cb_action_event_received)

        with open(self.rospack.get_path("thr_action_server")+"/config/decision_action_mapping.json") as config_file:
            self.decision_action_mapping = json.load(config_file)

    def init_webasker(self):
        with open(self.rospack.get_path("thr_interaction_controller")+"/config/mongo_adress_list.json") as adress_file:
            adress_list = json.load(adress_file)
        for adress in adress_list:
            try:
                self.web_asker = web_asker.WebAsker(adress)
            except Exception, e:
                rospy.loginfo(e)
                rospy.loginfo("Cannot reach {}.".format(adress))
                rospy.loginfo("Trying next adress...")
            else:
                break

        if self.web_asker is None:
            rospy.logerr("Cannot reach any adress.")
        else:
            self.web_asker.clear_all()

    def cb_action_event_received(self, event):
        if event.type in [ActionHistoryEvent.FINISHED_SUCCESS, ActionHistoryEvent.FINISHED_FAILURE]:
            key = (event.action.type, tuple(event.action.parameters))
            for i, question in enumerate(self.feedback_question_list):
                if question.get_delete_key() == key and not question.is_user_engaged():
                    correct_decision = self.str_to_Decision(question.get_correct_decision())
                    predicted_decision = self.str_to_Decision(question.get_predicted_decision())
                    if event.type == ActionHistoryEvent.FINISHED_SUCCESS:
                        self.head.show_result(True)
                        self.set_new_training_example(question.get_state(), correct_decision,
                                                      question.confidence, predicted_decision)
                        self.waiting = False
                    else:
                        self.head.show_result(False)
                    del self.feedback_question_list[i]
                    question.remove()

        if event.side == 'human' and event.type == ActionHistoryEvent.STARTING:
            if self.current_scene is not None:
                predicted_decision = self.str_to_Decision(self.last_predicted_decision_str)
                self.set_new_training_example(self.current_scene, event.action, self.last_predicition_confidence,
                                              predicted_decision)

    #################################################
    # SERVICE CALLERS ###############################
    #################################################

    def set_new_training_example(self, scene, decision, prediction_confidence, predicted_action):
        request = SetNewTrainingExampleRequest()
        request.decision = decision
        request.predicted_decision = predicted_action
        request.scene_state = scene
        request.prediction_confidence = prediction_confidence

        try:
            reward = rospy.ServiceProxy(self.reward_service, SetNewTrainingExample)
            reward(request)
        except rospy.ServiceException, e:
            rospy.logerr("Cannot set training example: {}".format(e.message))

    def update_scene(self):
        request = GetSceneStateRequest()
        try:
            getscene = rospy.ServiceProxy(self.scene_state_service, GetSceneState)
            self.last_scene = self.current_scene
            self.current_scene = getscene(request).state
        except rospy.ServiceException, e:
            rospy.logerr("Cannot update scene {}:".format(e.message))

    def predict(self):
        request = GetNextDecisionRequest()
        request.scene_state = self.current_scene
        try:
            predict = rospy.ServiceProxy(self.predictor_service, GetNextDecision)
            return predict(request)
        except rospy.ServiceException, e:
            rospy.logerr("Cannot call predictor:".format(e.message))
            decision = Decision(type='wait')
            return GetNextActionResponse(decisions=[decision], probas=[1.])

    def start_or_stop_episode(self, start=True):
        self.head.reset_signal()
        for node in ['scene_state_manager', 'human_activity_recognizer', 'action_server', 'learner_predictor']:
            url = '/thr/{}/start_stop'.format(node)
            rospy.wait_for_service(url)
            rospy.ServiceProxy(url, StartStopEpisode).call(StartStopEpisodeRequest(
                command=StartStopEpisodeRequest.START if start else
                StartStopEpisodeRequest.STOP))

    ###################################################################################################################

    def Decision_to_str(self, decision):
        return decision.type + "(" + ", ".join(decision.parameters) + ")"

    def str_to_Decision(self, string):
        decision = Decision()
        decision.type = string.split("(")[0]
        if len(string.split("(")[1][:-1]) > 0:
            decision.parameters = string.split("(")[1][:-1].split(", ")
        else:
            decision.parameters = []
        return decision

    def run_decision(self, decision):
        os.system('beep')
        if decision.type == 'wait':
            self.waiting = True
            return
        self.head.reset_signal()
        goal = RunDecisionGoal()
        goal.decision = decision
        self.run_decision_client.send_goal(goal)
        while self.run_decision_client.get_state() in [GoalStatus.PENDING, GoalStatus.ACTIVE] and not rospy.is_shutdown():
            self.interaction_loop_rate.sleep()
        self.current_decision = decision

    def run(self):
        try:
            print 'Interaction starting!'

            is_running = False
            start_stop_question = self.web_asker.ask("Start ?", ["Start !"], priority=30)

            rospy.set_param("/thr/paused", False)

            while self.running and not rospy.is_shutdown():
                if not is_running:
                    if start_stop_question.answered():
                        start_stop_question.remove()
                        is_running = True
                        self.start_or_stop_episode(True)
                        start_stop_question = self.web_asker.ask("Stop ?", ["Stop !"], priority=30)

                        rospy.set_param("/thr/paused", False)
                        pause_unpause_question = self.web_asker.ask("Pause ?", ["Pause !"], priority=20)

                elif start_stop_question.answered():
                    start_stop_question.remove()
                    pause_unpause_question.remove()
                    is_running = False

                    if self.confirm_question is not None:
                        self.confirm_question.remove()
                        self.confirm_question = None
                    for question in self.feedback_question_list:
                        question.remove()
                    self.feedback_question_list = []

                    self.start_or_stop_episode(False)
                    start_stop_question = self.web_asker.ask("Restart ?", ["Restart !"], priority=30)

                else:

                    self.update_scene()

                    for i, question in enumerate(self.feedback_question_list):
                        question.update()
                        if question.is_answered():
                            correct_decision = self.str_to_Decision(question.get_correct_decision())
                            predicted_decision = self.str_to_Decision(question.get_predicted_decision())
                            self.set_new_training_example(question.get_state(), correct_decision,
                                                          question.confidence, predicted_decision)
                            del self.feedback_question_list[i]
                            question.remove()

                    if self.confirm_question is not None:
                        self.confirm_question.update()
                        if self.confirm_question.is_answered():
                            correct_decision = self.str_to_Decision(self.confirm_question.get_correct_decision())
                            predicted_decision = self.str_to_Decision(self.confirm_question.get_predicted_decision())
                            if (correct_decision.type != "wait" or
                                    self.current_scene.predicates == self.confirm_question.get_state().predicates):
                                self.run_decision(correct_decision)
                            self.set_new_training_example(self.confirm_question.get_state(), correct_decision,
                                                          self.confirm_question.confidence, predicted_decision)
                            self.confirm_question = None
                            self.interaction_loop_rate.sleep()
                            continue

                    if rospy.get_param("/thr/paused"):
                        if pause_unpause_question.answered():
                            pause_unpause_question.remove()
                            pause_unpause_question = self.web_asker.ask("Pause ?", ["Pause !"], priority=20)
                            rospy.set_param("/thr/paused", False)

                    elif pause_unpause_question.answered():
                        pause_unpause_question.remove()
                        pause_unpause_question = self.web_asker.ask("Unpause ?", ["Unpause !"], priority=20)
                        rospy.set_param("/thr/paused", True)

                    else:
                        if self.confirm_question is not None:
                            self.interaction_loop_rate.sleep()
                            continue

                        if self.waiting:
                            if self.current_scene.predicates == self.last_scene.predicates:
                                self.interaction_loop_rate.sleep()
                                continue
                            else:
                                self.waiting = False
                                for i, question in enumerate(self.feedback_question_list):
                                    if question.get_delete_key() == "wait":
                                        correct_decision = self.str_to_Decision(question.get_correct_decision())
                                        predicted_decision = self.str_to_Decision(question.get_predicted_decision())
                                        self.set_new_training_example(question.get_state(), correct_decision,
                                                                      question.confidence, predicted_decision)
                                        del self.feedback_question_list[i]
                                        question.remove()

                        prediction = self.predict()
                        str_action_list = [self.Decision_to_str(a) for a in prediction.decisions]
                        predicted_decision = np.random.choice(prediction.decisions, p=prediction.probas)
                        action_str = self.Decision_to_str(predicted_decision)

                        self.last_predicted_decision_str = action_str
                        self.last_predicition_confidence = prediction.confidence

                        if prediction.mode == prediction.SURE:
                            predicted_decision = self.str_to_Decision(action_str)
                            if action_str != "wait()" or len(str_action_list) > 1:
                                if predicted_decision.type != "wait":
                                    key = (self.decision_action_mapping[predicted_decision.type]["type"],
                                           tuple(predicted_decision.parameters))
                                else:
                                    key = "wait"

                                self.feedback_question_list.append(
                                    FeedbackQuestion(self.web_asker, action_str, str_action_list,
                                                     self.current_scene, prediction.confidence, key))

                            self.run_decision(predicted_decision)

                        elif prediction.mode == prediction.CONFIRM:
                            if action_str == "wait()" and len(str_action_list) == 1:
                                self.run_decision(self.str_to_Decision(action_str))
                            else:
                                self.confirm_question = ConfirmQuestion(self.web_asker, action_str,
                                                                        str_action_list, self.current_scene,
                                                                        prediction.confidence, head=self.head)

                        elif prediction.mode == prediction.NO_IDEA:
                            raise NotImplementedError()

                        else:
                            rospy.logerr("Received unknown prediction confidence mode {}".format(prediction.mode))
                            assert False

                self.interaction_loop_rate.sleep()
        finally:
            logs_name = rospy.get_param('/thr/logs_name')
            if logs_name != "none":
                with open('action_decisions_'+logs_name+'.json', 'w') as f:
                    json.dump(self.logs, f)

if __name__ == '__main__':
    rospy.init_node("interaction_controller")
    InteractionController().run()
