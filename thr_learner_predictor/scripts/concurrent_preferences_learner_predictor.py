#!/usr/bin/env python

import rospy
import rospkg
import random
import numpy as np
import os

from RBLT.domains import domain_dict
# from RBLT.learning.boosted_policy_learning import BoostedPolicyLearning

from thr_infrastructure_msgs.srv import GetNextAction, GetNextActionResponse,\
    StartStopEpisode, StartStopEpisodeRequest, StartStopEpisodeResponse
from thr_infrastructure_msgs.srv import SetNewTrainingExample, SetNewTrainingExampleResponse
from thr_infrastructure_msgs.msg import MDPAction


class Server(object):
    def __init__(self):
        self.learner_name = '/thr/learner'
        self.predictor_name = '/thr/predictor'
        self.rospack = rospkg.RosPack()

        self.tmp_dir_name = "/tmp/"
        self.i_tree = 0

        self.dataset = []

        self.domain = domain_dict["multi_agent_box_coop"].Domain({"random_start": False}, "/tmp")
        self.reward = self.domain.rewards["reward_built"]
        self.task_q_fun = self.reward.get_task_q_fun_gen()
        self.learned_q_fun = self.task_q_fun

        # self.algorithm = BoostedPolicyLearning(self.domain, {}, "/tmp")
        # self.algorithm.load()

        self.start_stop_service_name = '/thr/learner_predictor/start_stop'
        rospy.Service(self.start_stop_service_name, StartStopEpisode, self.cb_start_stop)

    def cb_start_stop(self, request):
        if request.command == StartStopEpisodeRequest.START:
            pass

        elif request.command == StartStopEpisodeRequest.STOP:
            self.learn_preferences()

        return StartStopEpisodeResponse()

    def learn_preferences(self):
        rospy.loginfo("Start learning")
        input_list = []
        target_list = []
        for state, action_expert in self.dataset:
            for action in self.domain.get_actions(state):
                input_list.append((state, action))
                if action == action_expert:
                    target_list.append(0. + np.random.normal(0, 0.000001))
                else:
                    target_list.append(-1. + np.random.normal(0, 0.000001))

        tree_q_user = self.domain.learnRegressor(input_list, target_list, os.path.join(self.tmp_dir_name,
                                                 "tree_q{}".format(self.i_tree)), maxdepth=6)
        # shutil.rmtree(os.path.join(tmp_dir_name, "tree_q{}".format(i_tree)))
        human_q_fun_pfull = lambda s, a: tree_q_user((s, a))
        self.learned_q_fun = lambda s, a: self.task_q_fun(s, a) + 0.1 * human_q_fun_pfull(s, a)

        self.i_tree += 1
        rospy.loginfo("Learning done")

    def relational_action_to_MDPAction(self, action):
        if isinstance(action, tuple):
            return MDPAction(type=action[0].replace("activate", "start"),
                             parameters=[c.replace("toolbox_", "/toolbox/") for c in action[1:]])
        else:
            return MDPAction(type=action.replace("activate", "start"),
                             parameters=[])

    def MDPAction_to_relational_action(self, action):
        if len(action.parameters) == 0:
            return action.type.replace("start", "activate")
        else:
            return tuple([action.type.replace("start", "activate")] +
                         [c.replace("/toolbox/", "toolbox_") for c in action.parameters])

    def scene_state_to_state(self, scene_state):
        slot_avaiable = {
            "toolbox_handle": 0,
            "toolbox_side_right": 1,
            "toolbox_side_left": 1,
            "toolbox_side_front": 2,
            "toolbox_side_back": 2,
        }
        slot_cor = ["no_slot", "one_slot", "two_slot"]

        pred_list = scene_state.predicates

        pred_robot_list = [
            tuple([str(pred.type)] +
                  [str(s).replace("/toolbox/", "toolbox_") for s in pred.parameters]) for pred in pred_list]

        pred_domain_list = []
        for pred in pred_robot_list:
            pred_domain_list.append(pred)
            if pred[0] == "positioned":
                pred_domain_list.append(("occupied_slot", pred[1], pred[3]))
                slot_avaiable[pred[2]] -= 1

            if pred[0] == "attached":
                pred_domain_list.append(("attached_slot", pred[1], pred[3]))
            for obj in ['toolbox_handle', 'toolbox_side_right', 'toolbox_side_left',
                        'toolbox_side_front', 'toolbox_side_back']:
                pred_domain_list.append(("object", obj))

            pred_domain_list.append(("object1", "toolbox_handle"))
            pred_domain_list.append(("object2", "toolbox_side_right"))
            pred_domain_list.append(("object2", "toolbox_side_left"))
            pred_domain_list.append(("object3", "toolbox_side_front"))
            pred_domain_list.append(("object3", "toolbox_side_back"))

            for obj in slot_avaiable:
                pred_domain_list.append((slot_cor[slot_avaiable[obj]], obj))

            pred_domain_list.append(("no_slot", "toolbox_handle"))

            for pose in ["0", "1"]:
                pred_domain_list.append(("holding_position", pose))
            if len([p for p in pred_robot_list if p[0] == "picked"]) == 0:
                pred_domain_list.append(("free", "left"))

        state = ("state", frozenset(pred_domain_list))
        return state

    def predictor_handler(self, get_next_action_req):
        """
        This handler is called when a request of prediction is received. It is based on a hardcoded policy
        :param get_next_action_req: an object of type GetNextActionRequest (scene state)
        :return: an object of type GetNextActionResponse
        """

        state = self.domain.state_to_int(self.scene_state_to_state(get_next_action_req.scene_state))
        action_list = self.domain.get_actions(state)
        quality_list = [self.learned_q_fun(state, a) for a in action_list]

        print self.domain.int_to_state(state)
        print [self.domain.int_to_action(a) for a in action_list]
        print quality_list

        max_quality = max(quality_list)

        best_action_list = [a for a, q in zip(action_list, quality_list) if q == max_quality]
        best_robot_action_list = self.domain.filter_robot_actions(best_action_list)
        if len(best_robot_action_list) == 0:
            best_robot_action_list.append(self.domain.action_to_int("wait"))
        action = self.relational_action_to_MDPAction(
            self.domain.int_to_action(random.choice(best_robot_action_list)))

        resp = GetNextActionResponse()
        resp.confidence = resp.CONFIRM
        resp.probas = []

        for candidate_action in self.domain.filter_robot_actions(action_list):
            resp.actions.append(self.relational_action_to_MDPAction(
                self.domain.int_to_action(candidate_action)))

            if action.type == resp.actions[-1].type and action.parameters == resp.actions[-1].parameters:
                resp.probas.append(1.)
            else:
                resp.probas.append(0.)

        if sum(resp.probas) != 1:
            print action
            print resp.actions
        return resp

    def learner_handler(self, new_training_ex):
        """
        This handler is called when a request of learning is received, it must return an empty message
        :param snter: an object of type SetNewTrainingExampleRequest
        :return: an object of type SetNewTrainingExampleResponse (not to be filled, this message is empty)
        """
        rospy.loginfo("I'm learning that action {}{} was {}".format(new_training_ex.action.type,
                                                                    str(new_training_ex.action.parameters),
                                                                    "good" if new_training_ex.good else "bad"))

        state = self.domain.state_to_int(self.scene_state_to_state(new_training_ex.scene_state))
        action = self.domain.action_to_int(self.MDPAction_to_relational_action(new_training_ex.action))
        self.dataset.append((state, action))

        return SetNewTrainingExampleResponse()

    def run(self):
        rospy.Service(self.predictor_name, GetNextAction, self.predictor_handler)
        rospy.Service(self.learner_name, SetNewTrainingExample, self.learner_handler)
        rospy.loginfo('[LearnerPredictor] server ready...')
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('learner_and_predictor')
    Server().run()