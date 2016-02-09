#!/usr/bin/env python

import rospy
import rospkg

from RBLT.domains import domain_dict
from RBLT.learning.boosted_policy_learning import BoostedPolicyLearning

from thr_coop_assembly.srv import GetNextAction, GetNextActionResponse,\
    StartStopEpisode, StartStopEpisodeRequest, StartStopEpisodeResponse
from thr_coop_assembly.srv import SetNewTrainingExample, SetNewTrainingExampleResponse
from thr_coop_assembly.msg import MDPAction


class Server(object):
    def __init__(self):
        self.sequence = 1  # ID of output actions
        self.learner_name = '/thr/learner'
        self.predictor_name = '/thr/predictor'
        self.rospack = rospkg.RosPack()

        self.dataset = []

        self.domain = domain_dict["multi_agent_box_coop"].Domain({"random_start": False}, "/tmp")
        self.algorithm = BoostedPolicyLearning(self.domain, {}, "/tmp")
        self.algorithm.load()

        self.start_stop_service_name = '/thr/learrner_predictor/start_stop'
        rospy.Service(self.start_stop_service_name, StartStopEpisode, self.cb_start_stop)

    def cb_start_stop(self, request):
        if request.command == StartStopEpisodeRequest.START:
            pass

        elif request.command == StartStopEpisodeRequest.STOP:
            for state, action in self.dataset:
                print state
                print action

        return StartStopEpisodeResponse()

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

    def MDPAction_to_relational_action(self, action):
        return tuple([action.type] + action.parameters)

    def string_to_action(self, string):
        string = string.replace("(", "+").replace(",", "+").replace(")", "")
        if string[-1] == "+":
            string = string[:-1]

        if "+" in string:
            return tuple(string.split("+"))
        else:
            return string

    def scene_state_to_state(self, scene_state):
        pred_list = scene_state.predicates

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
        return state

    def predictor_handler(self, get_next_action_req):
        """
        This handler is called when a request of prediction is received. It is based on a hardcoded policy
        :param get_next_action_req: an object of type GetNextActionRequest (scene state)
        :return: an object of type GetNextActionResponse
        """

        resp = GetNextActionResponse()
        pred_list = get_next_action_req.scene_state.predicates

        pred_robot_list = [tuple([str(pred.type)] + [str(s).replace("/toolbox/", "toolbox_") for s in pred.parameters])
                           for pred in pred_list]

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

        state = self.domain.state_to_int(("state", frozenset(pred_domain_list)))
        action_list = [self.domain.int_to_action(a) for a in self.domain.get_actions(state)]

        resp = GetNextActionResponse()
        obj_list = ['/toolbox/handle', '/toolbox/side_right', '/toolbox/side_left',
                    '/toolbox/side_front', '/toolbox/side_back']
        pred_list = get_next_action_req.scene_state.predicates
        in_hws_list = [o for o in obj_list if self.check_in_hws_pred(pred_list, o)]

        action = MDPAction()
        action.id = self.sequence

        if len(in_hws_list) == 0:
            if not self.check_busy_pred(pred_list, "left"):
                if not self.check_picked_pred(pred_list, '/toolbox/handle'):
                    action.parameters = ['/toolbox/handle']
                    action.type = 'start_pick'
                elif not self.check_is_holding(pred_list):
                    action.parameters = ['/toolbox/handle']
                    action.type = 'start_give'
                else:
                    action.type = 'wait'

            elif not self.check_busy_pred(pred_list, "right") and not self.check_at_home_pred(pred_list, "right"):
                action.type = 'start_go_home_right'

            else:
                action.type = 'wait'

        elif len(in_hws_list) == 1:
            if not self.check_busy_pred(pred_list, "left"):
                if not self.check_picked_pred(pred_list, '/toolbox/side_right'):
                    action.type = 'start_pick'
                    action.parameters = ['/toolbox/side_right']
                elif not self.check_is_holding(pred_list):
                    action.type = 'start_give'
                    action.parameters = ['/toolbox/side_right']
                else:
                    action.type = 'wait'

            elif not self.check_busy_pred(pred_list, "right") and not self.check_at_home_pred(pred_list, "right"):
                action.type = 'start_go_home_right'

            else:
                action.type = 'wait'

        elif len(in_hws_list) == 2:
            if self.check_attached_pred(pred_list, '/toolbox/handle', '/toolbox/side_right'):
                if not self.check_busy_pred(pred_list, "right") and not self.check_at_home_pred(pred_list, "right"):
                    action.type = 'start_go_home_right'

                elif not self.check_busy_pred(pred_list, "left"):
                    if not self.check_picked_pred(pred_list, '/toolbox/side_left'):
                        action.type = 'start_pick'
                        action.parameters = ['/toolbox/side_left']
                    elif not self.check_is_holding(pred_list):
                        action.type = 'start_give'
                        action.parameters = ['/toolbox/side_left']
                    else:
                        action.type = 'wait'
                else:
                    action.type = 'wait'

            elif self.check_positioned_pred(pred_list, '/toolbox/handle', '/toolbox/side_right', 0):
                if not self.check_busy_pred(pred_list, "right"):
                    action.type = 'start_hold'
                    action.parameters = ['/toolbox/handle', '0']

                elif not self.check_picked_pred(pred_list, '/toolbox/side_left') and not self.check_busy_pred(pred_list, "left"):
                    action.type = 'start_pick'
                    action.parameters = ['/toolbox/side_left']

                else:
                    action.type = 'wait'

            elif self.check_positioned_pred(pred_list, '/toolbox/handle', '/toolbox/side_right', 1):
                if not self.check_busy_pred(pred_list, "right"):
                    action.type = 'start_hold'
                    action.parameters = ['/toolbox/handle', '1']

                elif not self.check_picked_pred(pred_list, '/toolbox/side_left') and not self.check_busy_pred(pred_list, "left"):
                    action.type = 'start_pick'
                    action.parameters = ['/toolbox/side_left']

                else:
                    action.type = 'wait'
            else:
                if not self.check_busy_pred(pred_list, "left") and not self.check_at_home_pred(pred_list, "left"):
                    action.type = 'start_go_home_left'
                elif not self.check_busy_pred(pred_list, "right") and not self.check_at_home_pred(pred_list, "right"):
                    action.type = 'start_go_home_right'
                else:
                    action.type = 'wait'
        elif len(in_hws_list) == 3:
            if self.check_attached_pred(pred_list, '/toolbox/handle', '/toolbox/side_left'):
                if not self.check_busy_pred(pred_list, "right") and not self.check_at_home_pred(pred_list, "right"):
                    action.type = 'start_go_home_right'
                elif not self.check_busy_pred(pred_list, "left"):
                    if not self.check_picked_pred(pred_list, '/toolbox/side_front'):
                        action.type = 'start_pick'
                        action.parameters = ['/toolbox/side_front']
                    elif not self.check_is_holding(pred_list):
                        action.type = 'start_give'
                        action.parameters = ['/toolbox/side_front']
                    else:
                        action.type = 'wait'
                else:
                    action.type = 'wait'

            elif self.check_positioned_pred(pred_list, '/toolbox/handle', '/toolbox/side_left', 0):
                if not self.check_busy_pred(pred_list, "left") and not self.check_picked_pred(pred_list, '/toolbox/side_front'):
                    action.type = 'start_pick'
                    action.parameters = ['/toolbox/side_front']

                elif not self.check_busy_pred(pred_list, "right"):
                    action.type = 'start_hold'
                    action.parameters = ['/toolbox/handle', '0']
                else:
                    action.type = 'wait'

            elif self.check_positioned_pred(pred_list, '/toolbox/handle', '/toolbox/side_left', 1):
                if not self.check_busy_pred(pred_list, "left") and not self.check_picked_pred(pred_list, '/toolbox/side_front'):
                    action.type = 'start_pick'
                    action.parameters = ['/toolbox/side_front']

                elif not self.check_busy_pred(pred_list, "right"):
                    action.type = 'start_hold'
                    action.parameters = ['/toolbox/handle', '1']
                else:
                    action.type = 'wait'

            else:
                if not self.check_busy_pred(pred_list, "left") and not self.check_at_home_pred(pred_list, "left"):
                    action.type = 'start_go_home_left'
                elif not self.check_busy_pred(pred_list, "right") and not self.check_at_home_pred(pred_list, "right"):
                    action.type = 'start_go_home_right'
                else:
                    action.type = 'wait'

        elif len(in_hws_list) == 4:
            if (self.check_attached_pred(pred_list, '/toolbox/side_left', '/toolbox/side_front') and
                self.check_attached_pred(pred_list, '/toolbox/side_right', '/toolbox/side_front')):

                if not self.check_busy_pred(pred_list, "right") and not self.check_at_home_pred(pred_list, "right"):
                    action.type = 'start_go_home_right'
                    

                elif not self.check_busy_pred(pred_list, "left"):
                    if not self.check_picked_pred(pred_list, '/toolbox/side_back'):
                        action.type = 'start_pick'
                        action.parameters = ['/toolbox/side_back']
                    elif not self.check_is_holding(pred_list):
                        action.type = 'start_give'
                        action.parameters = ['/toolbox/side_back']
                    else:
                        action.type = 'wait'
                else:
                    action.type = 'wait'
                    

   
            elif (self.check_positioned_pred(pred_list, '/toolbox/side_left', '/toolbox/side_front', 0) and
                self.check_positioned_pred(pred_list, '/toolbox/side_right', '/toolbox/side_front', 1)):


                if not self.check_attached_pred(pred_list, '/toolbox/side_left', '/toolbox/side_front', 0):
                    if not self.check_busy_pred(pred_list, "left") and not self.check_at_home_pred(pred_list, "left"):
                        action.type = 'start_go_home_left'
                        

                    elif not self.check_busy_pred(pred_list, "right"):
                        action.type = 'start_hold'
                        action.parameters = ['/toolbox/side_left', '0']
                        

                    else:
                        action.type = 'wait'
                        
                else:
                    if not self.check_busy_pred(pred_list, "left") and not self.check_picked_pred(pred_list, '/toolbox/side_back'):
                        rospy.logerr("1")
                        action.type = 'start_pick'
                        action.parameters = ['/toolbox/side_back']
                        

                    elif not self.check_busy_pred(pred_list, "right"):
                        action.type = 'start_hold'
                        action.parameters = ['/toolbox/side_right', '1']
                        

                    else:
                        action.type = 'wait'
                        

            elif (self.check_positioned_pred(pred_list, '/toolbox/side_left', '/toolbox/side_front', 1) and
                self.check_positioned_pred(pred_list, '/toolbox/side_right', '/toolbox/side_front', 0)):

                if not self.check_attached_pred(pred_list, '/toolbox/side_left', '/toolbox/side_front', 1):
                    if not self.check_busy_pred(pred_list, "left") and not self.check_at_home_pred(pred_list, "left"):
                        action.type = 'start_go_home_left'
                        

                    elif not self.check_busy_pred(pred_list, "right"):
                        action.type = 'start_hold'
                        action.parameters = ['/toolbox/side_left', '1']
                        
                    else:
                        action.type = 'wait'
                        
                else:
                    if not self.check_busy_pred(pred_list, "left") and not self.check_picked_pred(pred_list, '/toolbox/side_back'):
                        action.type = 'start_pick'
                        rospy.logerr("2")
                        action.parameters = ['/toolbox/side_back']
                        

                    elif not self.check_busy_pred(pred_list, "right"):
                        action.type = 'start_hold'
                        action.parameters = ['/toolbox/side_right', '0']
                        
                    else:
                        action.type = 'wait'
                        

            else:
                if not self.check_busy_pred(pred_list, "left") and not self.check_at_home_pred(pred_list, "left"):
                    action.type = 'start_go_home_left'
                    
                elif not self.check_busy_pred(pred_list, "right") and not self.check_at_home_pred(pred_list, "right"):
                    action.type = 'start_go_home_right'
                    
                else:
                    action.type = 'wait'
                    

        elif len(in_hws_list) == 5:
            if (self.check_attached_pred(pred_list, '/toolbox/side_left', '/toolbox/side_back') and
                self.check_attached_pred(pred_list, '/toolbox/side_right', '/toolbox/side_back')):

                if not self.check_busy_pred(pred_list, "right") and not self.check_at_home_pred(pred_list, "right"):
                    action.type = 'start_go_home_right'
                    
                elif not self.check_busy_pred(pred_list, "left") and not self.check_at_home_pred(pred_list, "left"):
                    action.type = 'start_go_home_left'
                    
                else:
                    action.type = 'wait'
                    
   
            elif (self.check_positioned_pred(pred_list, '/toolbox/side_left', '/toolbox/side_back', 0) and
                self.check_positioned_pred(pred_list, '/toolbox/side_right', '/toolbox/side_back', 1)):


                if not self.check_busy_pred(pred_list, "left") and not self.check_at_home_pred(pred_list, "left"):
                    action.type = 'start_go_home_left'
                    

                elif not self.check_attached_pred(pred_list, '/toolbox/side_left', '/toolbox/side_back', 0) and not self.check_busy_pred(pred_list, "right"):
                    action.type = 'start_hold'
                    action.parameters = ['/toolbox/side_left', '0']
                    
                elif not self.check_busy_pred(pred_list, "right"):
                    action.type = 'start_hold'
                    action.parameters = ['/toolbox/side_right', '1']
                    
                else:
                    action.type = 'wait'
                    

            elif (self.check_positioned_pred(pred_list, '/toolbox/side_left', '/toolbox/side_back', 1) and
                self.check_positioned_pred(pred_list, '/toolbox/side_right', '/toolbox/side_back', 0)):

                if not self.check_busy_pred(pred_list, "left") and not self.check_at_home_pred(pred_list, "left"):
                    action.type = 'start_go_home_left'
                    

                elif not self.check_attached_pred(pred_list, '/toolbox/side_left', '/toolbox/side_back', 1) and not self.check_busy_pred(pred_list, "right"):
                    action.type = 'start_hold'
                    action.parameters = ['/toolbox/side_left', '1']
                    
                elif not self.check_busy_pred(pred_list, "right"):
                    action.type = 'start_hold'
                    action.parameters = ['/toolbox/side_right', '0']
                    
                else:
                    action.type = 'wait'
                    

            else:
                if not self.check_busy_pred(pred_list, "left") and not self.check_at_home_pred(pred_list, "left"):
                    action = MDPAction()
                    action.type = 'start_go_home_left'
                    
                elif not self.check_busy_pred(pred_list, "right") and not self.check_at_home_pred(pred_list, "right"):
                    action.type = 'start_go_home_right'
                    
                else:
                    action.type = 'wait'

        resp = GetNextActionResponse()
        resp.confidence = resp.SURE
        resp.probas = []

        for candidate_action in action_list:
            if isinstance(candidate_action, tuple):
                resp.actions.append(MDPAction(
                    type=candidate_action[0].replace("activate", "start"),
                    parameters=[c.replace("toolbox_", "/toolbox/") for c in candidate_action[1:]]))
            else:
                resp.actions.append(MDPAction(type=candidate_action.replace("activate", "start"),
                                              parameters=[]))

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

        state = self.scene_state_to_state(new_training_ex.scene_state)
        action = self.MDPAction_to_relational_action(new_training_ex.action)
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
