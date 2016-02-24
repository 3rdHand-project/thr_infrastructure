#!/usr/bin/env python

from pyFolWorld import FolWorld
import rospy, rospkg
import tempfile
import pickle

from RBLT.domains import domain_dict
from RBLT.learning.boosted_policy_learning import BoostedPolicyLearning

from thr_infrastructure_msgs.srv import GetNextDecision, GetNextDecisionRequest, GetNextDecisionResponse
from thr_infrastructure_msgs.srv import SetNewTrainingExample, SetNewTrainingExampleRequest, SetNewTrainingExampleResponse
from thr_infrastructure_msgs.msg import Decision, Predicate

# To test this server, try: "rosservice call [/thr/learner or /thr/predictor] <TAB>" and complete the pre-filled request message before <ENTER>

class Server(object):
    def __init__(self):
        self.learner_name = '/thr/learner'
        self.predictor_name = '/thr/predictor'
        self.rospack = rospkg.RosPack()

        domain = domain_dict["multi_agent_box"].Domain({"decay_factor" : 0.9, "random_start" : False}, "/tmp")
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
        :param get_next_action_req: an object of type GetNextDecisionRequest (scene state)
        :return: an object of type GetNextDecisionResponse
        """
        resp = GetNextDecisionResponse()
        obj_list = ['/toolbox/handle', '/toolbox/side_right', '/toolbox/side_left', '/toolbox/side_front', '/toolbox/side_back']
        pred_list = get_next_action_req.scene_state.predicates

        pred_robot_list = [tuple([str(pred.type)] + [str(s).replace("/toolbox/", "toolbox_") for s in pred.parameters]) for pred in pred_list]


        pred_domain_list = []
        for pred in pred_robot_list:
            pred_domain_list.append(pred)
            if pred[0] == "positioned":
                pred_domain_list.append(("occupied_slot", pred[1], pred[3]))
            for obj in ['toolbox_handle', 'toolbox_side_right', 'toolbox_side_left', 'toolbox_side_front', 'toolbox_side_back']:
                pred_domain_list.append(("object", obj))
            for pose in ["0", "1"]:
                pred_domain_list.append(("holding_position", pose))
            if len([p for p in pred_robot_list if p[0] == "picked"]) == 0:
                pred_domain_list.append(("free", "left"))

        state = ("state", frozenset(pred_domain_list))


        module_path = self.rospack.get_path("thr_learner_predictor")


        # with open(module_path + "/config/learned_policy.pckl") as policy_file:
        #     policy = pickle.load(policy_file)
        
        # print state
        # if state in policy:
        #     policy_decision = policy[state]
        # else:
        #     print "state not found"
        #     policy_decision = "WAIT"

        print state
        policy_decision = self.algorithm.policy(state)
        print policy_decision

        if isinstance(policy_decision, tuple):
            policy_decision = tuple([a.replace("toolbox_", "/toolbox/") for a in policy_decision])

        in_hws_list = [o for o in obj_list if self.check_in_hws_pred(pred_list, o)]
        decision = Decision()

        if policy_decision == "activate_wait_for_human" or policy_decision == "WAIT":
            decision.type = 'wait'
        elif type(policy_decision) == str:
            decision.type = policy_decision.replace("activate", "start")
            decision.parameters = []
        else:
            decision.type = policy_decision[0].replace("activate", "start")
            decision.parameters = list(policy_decision)[1:]
            
        print decision, policy_decision

        resp = GetNextDecisionResponse()
        resp.confidence = resp.SURE

        obj_list = ['/toolbox/handle', '/toolbox/side_right', '/toolbox/side_left', '/toolbox/side_front', '/toolbox/side_back']

        actions_index = {}

        resp.probas = []

        resp.actions.append(Decision(type="wait", parameters=[]))
        if decision.type == resp.actions[-1].type and decision.parameters == resp.actions[-1].parameters:
            resp.probas.append(1.)
        else:
            resp.probas.append(0.)

        resp.actions.append(Decision(type="start_go_home_left", parameters=[]))
        if decision.type == resp.actions[-1].type and decision.parameters == resp.actions[-1].parameters:
            resp.probas.append(1.)
        else:
            resp.probas.append(0.)

        resp.actions.append(Decision(type="start_go_home_right", parameters=[]))
        if decision.type == resp.actions[-1].type and decision.parameters == resp.actions[-1].parameters:
            resp.probas.append(1.)
        else:
            resp.probas.append(0.)

        for obj in obj_list:
            resp.actions.append(Decision(type="start_give", parameters=[obj]))
            if decision.type == resp.actions[-1].type and decision.parameters == resp.actions[-1].parameters:
                resp.probas.append(1.)
            else:
                resp.probas.append(0.)

        for obj in obj_list:
            for pose in ["0", "1"]:
                resp.actions.append(Decision(type="start_hold", parameters=[obj, pose]))
                if decision.type == resp.actions[-1].type and decision.parameters == resp.actions[-1].parameters:
                    resp.probas.append(1.)
                else:
                    resp.probas.append(0.)

        for obj in obj_list:
            resp.actions.append(Decision(type="start_pick", parameters=[obj]))
            if decision.type == resp.actions[-1].type and decision.parameters == resp.actions[-1].parameters:
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
        rospy.Service(self.predictor_name, GetNextDecision, self.predictor_handler)
        rospy.Service(self.learner_name, SetNewTrainingExample, self.learner_handler)
        rospy.loginfo('[LearnerPredictor] server ready...')
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('learner_and_predictor')
    Server().run() # Blocking spinning call until shutdown!