#!/usr/bin/env python
import os, rospy, rospkg
import actionlib
import numpy as np
from copy import deepcopy
import json

import web_asker

from thr_coop_assembly.msg import *
from thr_coop_assembly.srv import *
from actionlib_msgs.msg import *

class InteractionController(object):
    def __init__(self):
        self.running = True
        self.waiting = False
        self.current_scene = None
        self.scene_before_action = None

        self.logs = []

        # Parameters to be tweaked
        self.interaction_loop_rate = rospy.Rate(20)  # Rate of the interaction loop in Hertz
                                                    # Shouldn't go too fast, it will trigger actions at that speed!
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


    def init_webasker(self):
        with open(self.rospack.get_path("thr_coop_assembly")+"/config/mongo_adress_list.json") as adress_file:
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
        if event.side == 'human':
            print event

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
        for node in ['scene_state_manager', 'human_activity_recognizer', 'action_server']:
            url = '/thr/{}/start_stop'.format(node)
            rospy.wait_for_service(url)
            rospy.ServiceProxy(url, StartStopEpisode).call(StartStopEpisodeRequest(command=StartStopEpisodeRequest.START if start else
                                                                                           StartStopEpisodeRequest.STOP))


    ###################################################################################################################

    def MDPAction_to_str(self, action):
        return action.type + "(" + ", ".join(action.parameters) + ")"

    def str_to_MDPAction(self, string):
        action = MDPAction()
        action.type = string.split("(")[0]
        action.parameters = string.split("(")[1][:-1].split(", ")
        return action

    def run_action(self, action):
        self.scene_before_action = deepcopy(self.current_scene)
        os.system('beep')
        if action.type == 'wait':
            self.waiting = True
            return
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
                    self.start_or_stop_episode(False)
                    start_stop_question = self.web_asker.ask("Start ?", ["Start !"], priority=30)

                elif rospy.get_param("/thr/paused"):
                    if pause_unpause_question.answered():
                        pause_unpause_question.remove()
                        pause_unpause_question = self.web_asker.ask("Pause ?", ["Pause !"], priority=20)
                        rospy.set_param("/thr/paused", False)

                elif pause_unpause_question.answered():
                    pause_unpause_question.remove()
                    pause_unpause_question = self.web_asker.ask("Unpause ?", ["Unpause !"], priority=20)
                    rospy.set_param("/thr/paused", True)

                else:
                    self.update_scene()

                    if self.waiting:
                        if self.current_scene.predicates == self.scene_before_action.predicates:
                            self.interaction_loop_rate.sleep()
                            continue

                    prediction = self.predict()
                    str_action_list = [self.MDPAction_to_str(a) for a in prediction.actions]

                    if prediction.confidence == prediction.SURE:
                        predicted_action = np.random.choice(prediction.actions, p=prediction.probas)
                        # question = self.web_asker.ask(
                        #    "I'm doing {} :".format(self.MDPAction_to_str(predicted_action)),
                        #    ["Don't do that"])

                        self.logs.append({'timestamp': rospy.get_time(),
                                          'type': predicted_action.type,
                                          'parameters': predicted_action.parameters})

                        self.run_action(np.random.choice(prediction.actions, p=prediction.probas))

                        # if question.answered():
                        #     question.remove()
                        #     correct_action = self.str_to_MDPAction(self.web_asker.ask(
                        #         "What should have been done ?", str_action_list).get_answer())
                        #     self.set_new_training_example(self.scene_before_action, correct_action, True)
                        #     self.set_new_training_example(self.scene_before_action, predicted_action, False)
                        # else:
                        #     question.remove()
                        #     self.set_new_training_example(self.scene_before_action, predicted_action, True)
                        self.set_new_training_example(self.scene_before_action, predicted_action, True)

                    elif prediction.confidence == prediction.CONFIRM:
                        predicted_action = np.random.choice(prediction.actions, p=prediction.probas)

                        question = self.web_asker.ask(
                            "Can I do {} :".format(self.MDPAction_to_str(predicted_action)),
                            ["Ok", "Don't do that"])
                        if question.get_answer() == "Ok":
                            correct_action = predicted_action
                        else:
                            self.set_new_training_example(self.current_scene, predicted_action, False)
                            correct_action = self.str_to_MDPAction(self.web_asker.ask(
                                "Pick an action :", str_action_list).get_answer())
                        

                        self.logs.append({'timestamp': rospy.get_time(),
                              'type': correct_action.type,
                              'parameters': correct_action.parameters})
                        self.run_action(correct_action)

                        self.set_new_training_example(self.scene_before_action, correct_action, True)

                    elif prediction.confidence == prediction.NO_IDEA:
                        correct_action = self.str_to_MDPAction(self.web_asker.ask(
                            "Pick an action :", str_action_list).get_answer())
                        self.logs.append({'timestamp': rospy.get_time(),
                              'type': correct_action.type,
                              'parameters': correct_action.parameters})
                        self.run_action(correct_action)


                        self.set_new_training_example(self.scene_before_action, correct_action, True)

                    else:
                        assert False

                self.interaction_loop_rate.sleep()
        finally:
            logs_name = rospy.get_param('/thr/logs_name')
            if logs_name != "none":
                with open('action_decisions_'+logs_name+'.json', 'w') as f:
                    json.dump(self.logs, f)

if __name__=='__main__':
    rospy.init_node("interaction_controller")
    InteractionController().run()