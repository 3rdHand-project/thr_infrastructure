#!/usr/bin/env python

from pyFolWorld import FolWorld
import rospy, rospkg
import tempfile
from thr_infrastructure_msgs.srv import GetNextDecision, GetNextDecisionRequest, GetNextDecisionResponse
from thr_infrastructure_msgs.srv import SetNewTrainingExample, SetNewTrainingExampleRequest, SetNewTrainingExampleResponse
from thr_infrastructure_msgs.msg import Decision, Predicate

# To test this server, try: "rosservice call [/thr/learner or /thr/predictor] <TAB>" and complete the pre-filled request message before <ENTER>

class Server(object):
    def __init__(self):
        self.learner_name = '/thr/learner'
        self.predictor_name = '/thr/predictor'
        self.rospack = rospkg.RosPack()

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

    def string_to_decision(self, string):
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

        module_path = self.rospack.get_path("thr_learner_predictor")
        # with tempfile.TemporaryFile() as temp_file:
        with open("tmp_file_planner", "w") as temp_file:

            temp_file.write("START_STATE {\n")
            for pred in pred_list:
                if pred.parameters[-1][:2] == "eq":
                    temp_file.write(" (" + pred.type + " " + " ".join(pred.parameters[:-1]) + ")=" + str(pred.parameters[-1][2:]) + "\n")
                else:
                    temp_file.write(" (" + pred.type + " " + " ".join(pred.parameters) + ")\n")
            for pred in ["(occupied_slot " + p.parameters[0] + " " + p.parameters[2] + ")\n" for p in pred_list if p.type == "positioned"]:
                temp_file.write(pred)
            if not self.check_picked_pred(pred_list):
                temp_file.write(" (free left)\n")

            temp_file.write(" (object /toolbox/handle)\n (object /toolbox/side_right)\n (object /toolbox/side_left)\n (object /toolbox/side_front)\n (object /toolbox/side_back)\n (holding_position 0)\n (holding_position 1) \n}\n")

            with open(module_path + "/config/reward.g") as reward_file:
                for line in reward_file:
                    temp_file.write(line)


        w = FolWorld(module_path + "/config/toolbox.g", "tmp_file_planner")
        best_decision_string = w.get_best_action(200, 100)
        planner_decision = self.string_to_decision(best_decision_string)

        in_hws_list = [o for o in obj_list if self.check_in_hws_pred(pred_list, o)]
        decision = Decision()

        if planner_decision == "activate_wait_for_human" or planner_decision == "WAIT":
            decision.type = 'wait'
        elif type(planner_decision) == str:
            decision.type = planner_decision.replace("activate", "start")
            decision.parameters = []
        else:
            decision.type = planner_decision[0].replace("activate", "start")
            decision.parameters = list(planner_decision)[1:]
            
        print decision, planner_decision

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