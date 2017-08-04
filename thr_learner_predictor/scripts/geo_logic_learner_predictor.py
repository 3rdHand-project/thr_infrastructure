#!/usr/bin/env python
import rospy
from thr_infrastructure_msgs.srv import StartStopEpisode, StartStopEpisodeRequest, StartStopEpisodeResponse
from thr_infrastructure_msgs.srv import GetNextDecision, GetNextDecisionRequest, GetNextDecisionResponse
from thr_infrastructure_msgs.srv import SetNewTrainingExample, SetNewTrainingExampleRequest, SetNewTrainingExampleResponse
from thr_infrastructure_msgs.msg import Decision, Predicate

from geo_logic_planner.srv import Plan
from geo_logic_planner.msg import ShowSlide

# To test this server, try: "rosservice call [/thr/learner or /thr/predictor] <TAB>" and complete the pre-filled request message before <ENTER>

class Server(object):
    def __init__(self):
        self.learner_name = '/thr/learner'
        self.predictor_name = '/thr/predictor'
        self.start_stop_service_name = '/thr/learner_predictor/start_stop'
        self.current_action_idx = -1
        rospy.Service(self.start_stop_service_name, StartStopEpisode, self.cb_start_stop)
        rospy.wait_for_service('/geo_logic_planner/proxy_plan')
        # get plan from planner
        try:
            proxy_plan = rospy.ServiceProxy('/geo_logic_planner/proxy_plan', Plan)
            res = proxy_plan('no_reba', 'exp_baxter', True, 4, False)
            self.planned_actions = res.logic_sequence
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        self.slide_pub = rospy.Publisher('/geo_logic_planner/show_slide', ShowSlide, queue_size=10)
        # initialize with the first action 
        self.get_next_action()


    def cb_start_stop(self, request):
        if request.command == StartStopEpisodeRequest.START:
            pass
        elif request.command == StartStopEpisodeRequest.STOP:
            pass
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

    def check_picked_pred(self, predictate_list, obj):
        return len([p for p in predictate_list if
            p.type == 'picked' and obj in p.parameters]) == 1

    def check_at_home_pred(self, predictate_list, arm):
        return len([p for p in predictate_list if
            p.type == 'at_home' and arm in p.parameters]) == 1

    def check_busy_pred(self, predictate_list, arm):
        return len([p for p in predictate_list if
            p.type == 'busy' and arm in p.parameters]) == 1

    def check_is_holding(self, predicate_list, obj):
        return len([p for p in predicate_list if 
            p.type == 'held' and obj in p.parameters]) == 1

    def get_current_action(self):
        if self.current_action_idx < len(self.planned_actions):
            self.slide_pub.publish(ShowSlide(self.current_action_idx))
            literals = self.planned_actions[self.current_action_idx].literals
            self.current_action = literals[0][9:]
            self.current_action_param = literals[2:]
        else:
            self.current_action = "finished"
            self.current_action_param = []

    def get_next_action(self):
        self.current_action_idx += 1
        self.get_current_action()

    def predictor_handler(self, get_next_decision_req):
        """
        This handler is called when a request of prediction is received. It is based on a hardcoded policy
        :param get_next_decision_req: an object of type GetNextDecisionRequest (scene state)
        :return: an object of type GetNextDecisionResponse
        """
        obj_list = ['/toolbox/handle', '/toolbox/side_right', '/toolbox/side_left', '/toolbox/side_front', '/toolbox/side_back']
        pred_list = get_next_decision_req.scene_state.predicates
        in_hws_list = [o for o in obj_list if self.check_in_hws_pred(pred_list, o)]

        decision = Decision()

        if self.current_action == "grasping":
            if not self.check_busy_pred(pred_list, "left"):
                if not self.check_picked_pred(pred_list, self.current_action_param[0]):
                    if not self.check_at_home_pred(pred_list, "left"):
                        decision.type = 'start_go_home_left'
                    else:
                        decision.parameters = self.current_action_param
                        decision.type = 'start_pick' 
                else:
                    self.get_next_action()
                    decision.type = 'wait'
            else:
                decision.type = 'wait'

        elif self.current_action == "placing":
            if not self.check_busy_pred(pred_list, "left"):
                if self.check_picked_pred(pred_list, self.current_action_param[0]):
                    decision.parameters = self.current_action_param
                    decision.type = 'start_place_left'
                elif not self.check_in_hws_pred(pred_list, self.current_action_param[0]):
                    if not self.check_at_home_pred(pred_list, "left"):
                        decision.type = 'start_go_home_left'
                    else:
                        decision.parameters = [self.current_action_param[0]]
                        decision.type = 'start_pick'
                else:
                    self.get_next_action()
                    decision.type = 'wait' 
            else:
                decision.type = 'wait'

        elif self.current_action == "handing":
            if not self.check_busy_pred(pred_list, "left"):
                if not self.check_picked_pred(pred_list, self.current_action_param[0]):
                    if not self.check_at_home_pred(pred_list, "left"):
                        decision.type = 'start_go_home_left'
                    else:
                        decision.parameters = self.current_action_param
                        decision.type = 'start_pick' 
                elif not self.check_in_hws_pred(pred_list, self.current_action_param[0]):
                    decision.parameters = self.current_action_param
                    decision.type = 'start_give'
                else:
                    self.get_next_action()
                    decision.type = 'wait'
            else:
                decision.type = 'wait'

        elif self.current_action == "holding":
            if not self.check_busy_pred(pred_list, "right"):
                if not self.check_is_holding(pred_list, self.current_action_param[0]):
                    decision.parameters = [self.current_action_param[0], '0']
                    decision.type = 'start_hold'
                else:
                    self.get_next_action()
                    descision.type = 'wait'
            else:
                decision.type = 'wait'

        elif self.current_action == "screwing":
            if not self.check_is_holding(pred_list, self.current_action_param[1]):
                decision.parameters = [self.current_action_param[1], '0']
                decision.type = 'start_hold'
            elif not self.check_attached_pred(pred_list, self.current_action_param[1],
                                              self.current_action_param[0]):
                decision.type = 'wait'
            else:
                self.get_next_action()
                decision.type = 'wait'

        elif self.current_action == "finished":
            if not self.check_at_home_pred(pred_list, "right"):
                decision.type = 'start_go_home_right'
            elif not self.check_at_home_pred(pred_list, "left"):
                decision.type = 'start_go_home_left'
            else:
                decision.type = 'wait'

        else:
            rospy.logerr("Action not defined")

        decision_response = GetNextDecisionResponse()
        decision_response.mode = decision_response.SURE
        decision_response.decisions.append(decision)
        decision_response.probas.append(1)

        return decision_response

    def learner_handler(self, new_training_ex):
        """
        This handler is called when a request of learning is received, it must return an empty message
        :param snter: an object of type SetNewTrainingExampleRequest
        :return: an object of type SetNewTrainingExampleResponse (not to be filled, this message is empty)
        """
        return SetNewTrainingExampleResponse()

    def run(self):
        rospy.Service(self.predictor_name, GetNextDecision, self.predictor_handler)
        rospy.Service(self.learner_name, SetNewTrainingExample, self.learner_handler)
        rospy.loginfo('[LearnerPredictor] server ready...')
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node('learner_and_predictor')
    Server().run() # Blocking spinning call until shutdown!
