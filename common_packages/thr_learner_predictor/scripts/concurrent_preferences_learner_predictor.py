#!/usr/bin/env python

import rospy
import rospkg

from RBLT.domains import domain_dict
from RBLT.learning.boosted_policy_learning import BoostedPolicyLearning

from thr_coop_assembly.srv import GetNextAction, GetNextActionResponse
from thr_coop_assembly.srv import SetNewTrainingExample, SetNewTrainingExampleResponse
from thr_coop_assembly.msg import MDPAction


class Server(object):
    def __init__(self):
        self.learner_name = '/thr/learner'
        self.predictor_name = '/thr/predictor'
        self.rospack = rospkg.RosPack()

        domain = domain_dict["multi_agent_box"].Domain({"decay_factor": 0.9, "random_start": False}, "/tmp")
        self.algorithm = BoostedPolicyLearning(domain, {}, "/tmp")
        self.algorithm.load()

    def check_attached_pred(self, predictate_list, obj1, obj2, id_c=None):
        return len([p for p in predictate_list if
                    p.type == 'attached' and obj1 in p.parameters and obj2 in p.parameters and
                    (id_c is None or str(id_c) in p.parameters)]) == 1

    def check_positioned_pred(self, predictate_list, obj1, obj2, id_c=None):
        return len([p for p in predictate_list if
                    p.type == 'positioned' and obj1 in p.parameters and obj2 in p.parameters and
                    (id_c is None or str(id_c) in p.parameters)]) == 1

    def check_in_hws_pred(self, predictate_list, obj):
        return len([p for p in predictate_list if
                    p.type == 'in_human_ws' and obj in p.parameters]) == 1

    def check_picked_pred(self, predictate_list, obj=None):
        return len([p for p in predictate_list if
                    p.type == 'picked' and (obj is None or obj in p.parameters)]) == 1

    def check_holded_pred(self, predictate_list, obj):
        return len([p for p in predictate_list if
                    p.type == 'holded' and obj in p.parameters]) == 1

    def check_at_home_pred(self, predictate_list, arm):
        return len([p for p in predictate_list if
                    p.type == 'at_home' and arm in p.parameters]) == 1

    def check_busy_pred(self, predictate_list, arm):
        return len([p for p in predictate_list if
                    p.type == 'busy' and arm in p.parameters]) == 1

    def string_to_action(self, string):
        string = string.replace("(", "+").replace(",", "+").replace(")", "")
        if string[-1] == "+":
            string = string[:-1]

        if "+" in string:
            return tuple(string.split("+"))
        else:
            return string

    def predictor_handler(self, get_next_action_req):
        """
        This handler is called when a request of prediction is received. It is based on a hardcoded policy
        :param get_next_action_req: an object of type GetNextActionRequest (scene state)
        :return: an object of type GetNextActionResponse
        """
        resp = GetNextActionResponse()
        obj_list = ['/toolbox/handle', '/toolbox/side_right', '/toolbox/side_left',
                    '/toolbox/side_front', '/toolbox/side_back']
        pred_list = get_next_action_req.scene_state.predicates

        pred_robot_list = [
            tuple([str(pred.type)] +
                  [str(s).replace("/toolbox/", "toolbox_") for s in pred.parameters]) for pred in pred_list]

        pred_domain_list = []
        for pred in pred_robot_list:
            pred_domain_list.append(pred)
            if pred[0] == "positioned":
                pred_domain_list.append(("occupied_slot", pred[1], pred[3]))
            for obj in ['toolbox_handle', 'toolbox_side_right', 'toolbox_side_left',
                        'toolbox_side_front', 'toolbox_side_back']:
                pred_domain_list.append(("object", obj))
            for pose in ["0", "1"]:
                pred_domain_list.append(("holding_position", pose))
            if len([p for p in pred_robot_list if p[0] == "picked"]) == 0:
                pred_domain_list.append(("free", "left"))

        state = ("state", frozenset(pred_domain_list))

        print state
        policy_action = self.algorithm.policy(state)
        print policy_action

        if isinstance(policy_action, tuple):
            policy_action = tuple([a.replace("toolbox_", "/toolbox/") for a in policy_action])

        action = MDPAction()

        if policy_action == "activate_wait_for_human" or policy_action == "WAIT":
            action.type = 'wait'
        elif type(policy_action) == str:
            action.type = policy_action.replace("activate", "start")
            action.parameters = []
        else:
            action.type = policy_action[0].replace("activate", "start")
            action.parameters = list(policy_action)[1:]

        print action, policy_action

        resp = GetNextActionResponse()
        resp.confidence = resp.SURE

        resp.probas = []

        resp.actions.append(MDPAction(type="wait", parameters=[]))
        if action.type == resp.actions[-1].type and action.parameters == resp.actions[-1].parameters:
            resp.probas.append(1.)
        else:
            resp.probas.append(0.)

        resp.actions.append(MDPAction(type="start_go_home_left", parameters=[]))
        if action.type == resp.actions[-1].type and action.parameters == resp.actions[-1].parameters:
            resp.probas.append(1.)
        else:
            resp.probas.append(0.)

        resp.actions.append(MDPAction(type="start_go_home_right", parameters=[]))
        if action.type == resp.actions[-1].type and action.parameters == resp.actions[-1].parameters:
            resp.probas.append(1.)
        else:
            resp.probas.append(0.)

        for obj in obj_list:
            resp.actions.append(MDPAction(type="start_give", parameters=[obj]))
            if action.type == resp.actions[-1].type and action.parameters == resp.actions[-1].parameters:
                resp.probas.append(1.)
            else:
                resp.probas.append(0.)

        for obj in obj_list:
            for pose in ["0", "1"]:
                resp.actions.append(MDPAction(type="start_hold", parameters=[obj, pose]))
                if action.type == resp.actions[-1].type and action.parameters == resp.actions[-1].parameters:
                    resp.probas.append(1.)
                else:
                    resp.probas.append(0.)

        for obj in obj_list:
            resp.actions.append(MDPAction(type="start_pick", parameters=[obj]))
            if action.type == resp.actions[-1].type and action.parameters == resp.actions[-1].parameters:
                resp.probas.append(1.)
            else:
                resp.probas.append(0.)

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
        return SetNewTrainingExampleResponse()

    def run(self):
        rospy.Service(self.predictor_name, GetNextAction, self.predictor_handler)
        rospy.Service(self.learner_name, SetNewTrainingExample, self.learner_handler)
        rospy.loginfo('[LearnerPredictor] server ready...')
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('learner_and_predictor')
    Server().run()
