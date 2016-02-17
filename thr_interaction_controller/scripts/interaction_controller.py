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

class ConfirmQuestion(object):
    def __init__(self, web_asker, action_str, action_str_list, state, halo):
        self.web_asker = web_asker
        self.action_str = action_str
        self.action_str_list = action_str_list
        self.state = state
        self.halo = halo
        self.halo.start_flashing('yellow')

        self.first = self.web_asker.ask("I'm think i should do {} :".format(self.action_str),
                                        ["Do it!", "Don't do that"], color="blue")
        self.second = None
        self.correct_action = None

    def update(self):
        if self.first is not None and self.first.answered():
            self.halo.stop_flashing()
            if self.first.get_answer() == "Do it!":
                self.correct_action = self.action_str
            else:
                self.second = self.web_asker.ask("I wanted to do {}, What should I do?".format(self.action_str),
                                                 self.action_str_list, color="blue")
                self.first = None

        if self.second is not None and self.second.answered():
            self.correct_action = self.second.get_answer()

    def is_answered(self):
        return self.correct_action is not None

    def get_correct_action(self):
        assert self.correct_action is not None
        return self.correct_action

    def get_state(self):
        return self.state

    def remove(self):
        if self.first is not None:
            self.first.remove()
        if self.second is not None:
            self.second.remove()


class FeedbackQuestion(object):
    def __init__(self, web_asker, action_str, action_str_list, state, delete_key):
        self.web_asker = web_asker
        self.action_str = action_str
        self.action_str_list = action_str_list
        self.state = state
        self.delete_key = delete_key

        self.first = self.web_asker.ask("I'm doing {} :".format(self.action_str),
                                        ["Don't do that"])
        self.second = None
        self.correct_action = None

    def update(self):
        if self.first is not None and self.first.answered():
            self.second = self.web_asker.ask("I did {}, What should have I done?".format(self.action_str),
                                             self.action_str_list)
            self.first.remove()
            self.first = None

        if self.second is not None and self.second.answered():
            self.correct_action = self.second.get_answer()

    def is_answered(self):
        return self.correct_action is not None

    def get_correct_action(self):
        if self.correct_action is not None:
            return self.correct_action
        else:
            return self.action_str

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
        self.confirm_question = None
        self.feedback_question_list = []

        self.current_scene = None
        self.scene_before_action = None

        self.logs = []
        self.halo = Halo()

        # Parameters to be tweaked
        self.interaction_loop_rate = rospy.Rate(20)
        self.reward_service = '/thr/learner'
        self.predictor_service = 'thr/predictor'
        self.scene_state_service = '/thr/scene_state'
        self.run_action_name = '/thr/run_mdp_action'
        self.action_history_name = '/thr/action_history'

        # Initiating topics ands links to services/actions
        self.run_action_client = actionlib.SimpleActionClient(self.run_action_name, RunMDPActionAction)
        rospy.loginfo("Waiting action client {}...".format(self.run_action_name))
        self.run_action_client.wait_for_server()
        for service in [self.reward_service, self.predictor_service, self.scene_state_service]:
            rospy.loginfo("Waiting service {}...".format(service))
            rospy.wait_for_service(service)

        self.rospack = rospkg.RosPack()
        self.init_webasker()
        rospy.Subscriber(self.action_history_name, ActionHistoryEvent, self.cb_action_event_received)

        with open(self.rospack.get_path("thr_action_server")+"/config/mdp_robot_mapping.json") as config_file:
            self.mdp_robot_mapping = json.load(config_file)

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
                    correct_action = self.str_to_MDPAction(question.get_correct_action())
                    if event.type == ActionHistoryEvent.FINISHED_SUCCESS:
                        self.halo.set_green()
                        self.set_new_training_example(question.get_state(), correct_action, True)
                    else:
                        self.halo.set_red()
                    del self.feedback_question_list[i]
                    question.remove()

        if event.side == 'human' and event.type == ActionHistoryEvent.STARTING:
            if self.current_scene is not None:
                self.set_new_training_example(self.current_scene,
                                              event.action,
                                              True)

    ################################################# SERVICE CALLERS #################################################
    def set_new_training_example(self, scene, action, good):
        request = SetNewTrainingExampleRequest()
        request.action = action

        request.scene_state = scene
        request.good = good
        try:
            reward = rospy.ServiceProxy(self.reward_service, SetNewTrainingExample)
            reward(request)
        except rospy.ServiceException, e:
            rospy.logerr("Cannot send reward {}:".format('good' if good else 'bad', e.message))

    def update_scene(self):
        request = GetSceneStateRequest()
        try:
            getscene = rospy.ServiceProxy(self.scene_state_service, GetSceneState)
            self.current_scene = getscene(request).state
        except rospy.ServiceException, e:
            rospy.logerr("Cannot update scene {}:".format(e.message))

    def predict(self):
        request = GetNextActionRequest()
        request.scene_state = self.current_scene
        try:
            predict = rospy.ServiceProxy(self.predictor_service, GetNextAction)
            return predict(request)
        except rospy.ServiceException, e:
            rospy.logerr("Cannot call predictor:".format(e.message))
            return MDPAction(type='wait')

    def start_or_stop_episode(self, start=True):
        self.halo.set_off()
        for node in ['scene_state_manager', 'human_activity_recognizer', 'action_server', 'learner_predictor']:
            url = '/thr/{}/start_stop'.format(node)
            rospy.wait_for_service(url)
            rospy.ServiceProxy(url, StartStopEpisode).call(StartStopEpisodeRequest(
                command=StartStopEpisodeRequest.START if start else
                StartStopEpisodeRequest.STOP))

    ###################################################################################################################

    def MDPAction_to_str(self, action):
        return action.type + "(" + ", ".join(action.parameters) + ")"

    def str_to_MDPAction(self, string):
        action = MDPAction()
        action.type = string.split("(")[0]
        if len(string.split("(")[1][:-1]) > 0:
            action.parameters = string.split("(")[1][:-1].split(", ")
        else:
            action.parameters = []
        return action

    def run_action(self, action):
        self.scene_before_action = deepcopy(self.current_scene)
        os.system('beep')
        if action.type == 'wait':
            self.waiting = True
            return
        self.halo.set_off()
        goal = RunMDPActionGoal()
        goal.action = action
        self.run_action_client.send_goal(goal)
        while self.run_action_client.get_state() in [GoalStatus.PENDING, GoalStatus.ACTIVE] and not rospy.is_shutdown():
            self.interaction_loop_rate.sleep()
        self.current_action = action

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
                            correct_action = self.str_to_MDPAction(question.get_correct_action())
                            self.set_new_training_example(question.get_state(), correct_action, True)
                            del self.feedback_question_list[i]
                            question.remove()

                    if self.confirm_question is not None:
                        self.confirm_question.update()
                        if self.confirm_question.is_answered():
                            correct_action = self.str_to_MDPAction(self.confirm_question.get_correct_action())
                            if (correct_action.type != "wait" or
                                    self.current_scene.predicates == self.scene_before_action.predicates):
                                self.run_action(correct_action)
                            self.set_new_training_example(self.confirm_question.get_state(), correct_action, True)
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
                            if self.current_scene.predicates == self.scene_before_action.predicates:
                                self.interaction_loop_rate.sleep()
                                continue
                            else:
                                self.waiting = False
                                for i, question in enumerate(self.feedback_question_list):
                                    if question.get_delete_key() == "wait":
                                        correct_action = self.str_to_MDPAction(question.get_correct_action())
                                        self.set_new_training_example(question.get_state(), correct_action, True)
                                        del self.feedback_question_list[i]
                                        question.remove()

                        prediction = self.predict()
                        str_action_list = [self.MDPAction_to_str(a) for a in prediction.actions]
                        predicted_action = np.random.choice(prediction.actions, p=prediction.probas)
                        action_str = self.MDPAction_to_str(predicted_action)

                        if prediction.confidence == prediction.SURE:
                            predicted_action = self.str_to_MDPAction(action_str)
                            if action_str != "wait()" or len(str_action_list) > 1:
                                if predicted_action.type != "wait":
                                    key = (self.mdp_robot_mapping[predicted_action.type]["type"],
                                           tuple(predicted_action.parameters))
                                else:
                                    key = "wait"

                                self.feedback_question_list.append(
                                    FeedbackQuestion(self.web_asker, action_str, str_action_list,
                                                     self.current_scene, key))

                            self.run_action(predicted_action)

                        elif prediction.confidence == prediction.CONFIRM:
                            if action_str == "wait()" and len(str_action_list) == 1:
                                self.run_action(self.str_to_MDPAction(action_str))
                            else:
                                self.confirm_question = ConfirmQuestion(self.web_asker, action_str,
                                                                        str_action_list, self.current_scene,
                                                                        halo=self.halo)

                        elif prediction.confidence == prediction.NO_IDEA:
                            raise NotImplemented()

                        else:
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
