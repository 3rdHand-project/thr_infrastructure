#!/usr/bin/env python

import random

import rospy
from thr_coop_assembly.srv import GetNextAction, GetNextActionRequest, GetNextActionResponse
from thr_coop_assembly.srv import SetNewTrainingExample, SetNewTrainingExampleRequest, SetNewTrainingExampleResponse
from thr_coop_assembly.msg import MDPAction, Predicate

# To test this server, try: "rosservice call [/thr/learner or /thr/predictor] <TAB>" and complete the pre-filled request message before <ENTER>

class Server(object):
    def __init__(self):
        self.learner_name = '/thr/learner'
        self.predictor_name = '/thr/predictor'

    def check_in_hws_pred(self, predictate_list, obj):
        return len([p for p in predictate_list if
            p.type == 'in_human_ws' and obj in p.parameters]) == 1

    def predictor_handler(self, get_next_action_req):
        obj_list = ['/toolbox/handle', '/toolbox/side_right', '/toolbox/side_left', '/toolbox/side_front', '/toolbox/side_back']
        pred_list = get_next_action_req.scene_state.predicates
        in_hws_list = [o for o in obj_list if self.check_in_hws_pred(pred_list, o)]

        resp = GetNextActionResponse()
        resp.confidence = random.choice([resp.CONFIRM, resp.NO_IDEA])

        obj_list = ['/toolbox/handle', '/toolbox/side_right', '/toolbox/side_left', '/toolbox/side_front', '/toolbox/side_back']

        actions_index = {}

        resp.probas = []

        resp.actions.append(MDPAction(type="wait", parameters=[]))
        resp.probas.append(0.)

        for obj in obj_list:
            resp.actions.append(MDPAction(type="give", parameters=[obj]))
            resp.probas.append(0.)

        for obj in obj_list:
            for pose in ["0", "1"]:
                resp.actions.append(MDPAction(type="hold", parameters=[obj, pose]))
                resp.probas.append(0.)

        resp.probas[random.choice(range(len(resp.probas)))] = 1.

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
    rospy.init_node('sequential_learner_and_predictor')
    Server().run() # Blocking spinning call until shutdown!