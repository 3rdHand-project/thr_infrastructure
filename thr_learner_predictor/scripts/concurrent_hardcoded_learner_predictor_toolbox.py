#!/usr/bin/env python

import rospy
from thr_infrastructure_msgs.srv import StartStopEpisode, StartStopEpisodeRequest, StartStopEpisodeResponse
from thr_infrastructure_msgs.srv import GetNextDecision, GetNextDecisionRequest, GetNextDecisionResponse
from thr_infrastructure_msgs.srv import SetNewTrainingExample, SetNewTrainingExampleRequest, SetNewTrainingExampleResponse
from thr_infrastructure_msgs.msg import Decision, Predicate

# To test this server, try: "rosservice call [/thr/learner or /thr/predictor] <TAB>" and complete the pre-filled request message before <ENTER>

class Server(object):
    def __init__(self):
        self.sequence = 1  # ID of output actions
        self.learner_name = '/thr/learner'
        self.predictor_name = '/thr/predictor'
        self.start_stop_service_name = '/thr/learner_predictor/start_stop'
        rospy.Service(self.start_stop_service_name, StartStopEpisode, self.cb_start_stop)

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

    def check_is_holding(self, predicate_list):
        return len([p for p in predicate_list if p.type=="hold"])==1

    def predictor_handler(self, get_next_decision_req):
        """
        This handler is called when a request of prediction is received. It is based on a hardcoded policy
        :param get_next_decision_req: an object of type GetNextDecisionRequest (scene state)
        :return: an object of type GetNextDecisionResponse
        """
        decision_response = GetNextDecisionResponse()
        obj_list = ['/toolbox/handle', '/toolbox/side_right', '/toolbox/side_left', '/toolbox/side_front', '/toolbox/side_back']
        pred_list = get_next_decision_req.scene_state.predicates
        in_hws_list = [o for o in obj_list if self.check_in_hws_pred(pred_list, o)]

        decision = Decision()

        if len(in_hws_list) == 0:
            if not self.check_busy_pred(pred_list, "left"):
                if not self.check_picked_pred(pred_list, '/toolbox/handle'):
                    decision.parameters = ['/toolbox/handle']
                    decision.type = 'start_pick'
                elif not self.check_is_holding(pred_list):
                    decision.parameters = ['/toolbox/handle']
                    decision.type = 'start_give'
                else:
                    decision.type = 'wait'
                
            elif not self.check_busy_pred(pred_list, "right") and not self.check_at_home_pred(pred_list, "right"):
                decision.type = 'start_go_home_right'

            else:
                decision.type = 'wait'

        elif len(in_hws_list) == 1:
            if not self.check_busy_pred(pred_list, "left"):
                if not self.check_picked_pred(pred_list, '/toolbox/side_right'):
                    decision.type = 'start_pick'
                    decision.parameters = ['/toolbox/side_right']
                elif not self.check_is_holding(pred_list):
                    decision.type = 'start_give'
                    decision.parameters = ['/toolbox/side_right']
                else:
                    decision.type = 'wait'

            elif not self.check_busy_pred(pred_list, "right") and not self.check_at_home_pred(pred_list, "right"):
                decision.type = 'start_go_home_right'

            else:
                decision.type = 'wait'

        elif len(in_hws_list) == 2:
            if self.check_attached_pred(pred_list, '/toolbox/handle', '/toolbox/side_right'):
                if not self.check_busy_pred(pred_list, "right") and not self.check_at_home_pred(pred_list, "right"):
                    decision.type = 'start_go_home_right'

                elif not self.check_busy_pred(pred_list, "left"):
                    if not self.check_picked_pred(pred_list, '/toolbox/side_left'):
                        decision.type = 'start_pick'
                        decision.parameters = ['/toolbox/side_left']
                    elif not self.check_is_holding(pred_list):
                        decision.type = 'start_give'
                        decision.parameters = ['/toolbox/side_left']
                    else:
                        decision.type = 'wait'
                else:
                    decision.type = 'wait'

            elif self.check_positioned_pred(pred_list, '/toolbox/handle', '/toolbox/side_right', 0):
                if not self.check_busy_pred(pred_list, "right"):
                    decision.type = 'start_hold'
                    decision.parameters = ['/toolbox/handle', '0']

                elif not self.check_picked_pred(pred_list, '/toolbox/side_left') and not self.check_busy_pred(pred_list, "left"):
                    decision.type = 'start_pick'
                    decision.parameters = ['/toolbox/side_left']

                else:
                    decision.type = 'wait'
                    

            elif self.check_positioned_pred(pred_list, '/toolbox/handle', '/toolbox/side_right', 1):
                if not self.check_busy_pred(pred_list, "right"):
                    decision.type = 'start_hold'
                    decision.parameters = ['/toolbox/handle', '1']
                    

                elif not self.check_picked_pred(pred_list, '/toolbox/side_left') and not self.check_busy_pred(pred_list, "left"):
                    decision.type = 'start_pick'
                    decision.parameters = ['/toolbox/side_left']
                    
                else:
                    decision.type = 'wait'
                    
            else:
                if not self.check_busy_pred(pred_list, "left") and not self.check_at_home_pred(pred_list, "left"):
                    decision.type = 'start_go_home_left'
                    
                elif not self.check_busy_pred(pred_list, "right") and not self.check_at_home_pred(pred_list, "right"):
                    decision.type = 'start_go_home_right'
                    
                else:
                    decision.type = 'wait'
                    

        elif len(in_hws_list) == 3:
            if self.check_attached_pred(pred_list, '/toolbox/handle', '/toolbox/side_left'):
                if not self.check_busy_pred(pred_list, "right") and not self.check_at_home_pred(pred_list, "right"):
                    decision.type = 'start_go_home_right'
                    

                elif not self.check_busy_pred(pred_list, "left"):
                    if not self.check_picked_pred(pred_list, '/toolbox/side_front'):
                        decision.type = 'start_pick'
                        decision.parameters = ['/toolbox/side_front']
                    elif not self.check_is_holding(pred_list):
                        decision.type = 'start_give'
                        decision.parameters = ['/toolbox/side_front']
                    else:
                        decision.type = 'wait'
                else:
                    decision.type = 'wait'
                    

            elif self.check_positioned_pred(pred_list, '/toolbox/handle', '/toolbox/side_left', 0):
                if not self.check_busy_pred(pred_list, "left") and not self.check_picked_pred(pred_list, '/toolbox/side_front'):
                    decision.type = 'start_pick'
                    decision.parameters = ['/toolbox/side_front']
                    

                elif not self.check_busy_pred(pred_list, "right"):
                    decision.type = 'start_hold'
                    decision.parameters = ['/toolbox/handle', '0']
                    
                else:
                    decision.type = 'wait'
                    


            elif self.check_positioned_pred(pred_list, '/toolbox/handle', '/toolbox/side_left', 1):
                if not self.check_busy_pred(pred_list, "left") and not self.check_picked_pred(pred_list, '/toolbox/side_front'):
                    decision.type = 'start_pick'
                    decision.parameters = ['/toolbox/side_front']
                    

                elif not self.check_busy_pred(pred_list, "right"):
                    decision.type = 'start_hold'
                    decision.parameters = ['/toolbox/handle', '1']
                    
                else:
                    decision.type = 'wait'
                    

            else:
                if not self.check_busy_pred(pred_list, "left") and not self.check_at_home_pred(pred_list, "left"):
                    decision.type = 'start_go_home_left'
                    
                elif not self.check_busy_pred(pred_list, "right") and not self.check_at_home_pred(pred_list, "right"):
                    decision.type = 'start_go_home_right'
                    
                else:
                    decision.type = 'wait'
                    

        elif len(in_hws_list) == 4:
            if (self.check_attached_pred(pred_list, '/toolbox/side_left', '/toolbox/side_front') and
                self.check_attached_pred(pred_list, '/toolbox/side_right', '/toolbox/side_front')):

                if not self.check_busy_pred(pred_list, "right") and not self.check_at_home_pred(pred_list, "right"):
                    decision.type = 'start_go_home_right'
                    

                elif not self.check_busy_pred(pred_list, "left"):
                    if not self.check_picked_pred(pred_list, '/toolbox/side_back'):
                        decision.type = 'start_pick'
                        decision.parameters = ['/toolbox/side_back']
                    elif not self.check_is_holding(pred_list):
                        decision.type = 'start_give'
                        decision.parameters = ['/toolbox/side_back']
                    else:
                        decision.type = 'wait'
                else:
                    decision.type = 'wait'
                    

   
            elif (self.check_positioned_pred(pred_list, '/toolbox/side_left', '/toolbox/side_front', 0) and
                self.check_positioned_pred(pred_list, '/toolbox/side_right', '/toolbox/side_front', 1)):


                if not self.check_attached_pred(pred_list, '/toolbox/side_left', '/toolbox/side_front', 0):
                    if not self.check_busy_pred(pred_list, "left") and not self.check_at_home_pred(pred_list, "left"):
                        decision.type = 'start_go_home_left'
                        

                    elif not self.check_busy_pred(pred_list, "right"):
                        decision.type = 'start_hold'
                        decision.parameters = ['/toolbox/side_left', '0']
                        

                    else:
                        decision.type = 'wait'
                        
                else:
                    if not self.check_busy_pred(pred_list, "left") and not self.check_picked_pred(pred_list, '/toolbox/side_back'):
                        rospy.logerr("1")
                        decision.type = 'start_pick'
                        decision.parameters = ['/toolbox/side_back']
                        

                    elif not self.check_busy_pred(pred_list, "right"):
                        decision.type = 'start_hold'
                        decision.parameters = ['/toolbox/side_right', '1']
                        

                    else:
                        decision.type = 'wait'
                        

            elif (self.check_positioned_pred(pred_list, '/toolbox/side_left', '/toolbox/side_front', 1) and
                self.check_positioned_pred(pred_list, '/toolbox/side_right', '/toolbox/side_front', 0)):

                if not self.check_attached_pred(pred_list, '/toolbox/side_left', '/toolbox/side_front', 1):
                    if not self.check_busy_pred(pred_list, "left") and not self.check_at_home_pred(pred_list, "left"):
                        decision.type = 'start_go_home_left'
                        

                    elif not self.check_busy_pred(pred_list, "right"):
                        decision.type = 'start_hold'
                        decision.parameters = ['/toolbox/side_left', '1']
                        
                    else:
                        decision.type = 'wait'
                        
                else:
                    if not self.check_busy_pred(pred_list, "left") and not self.check_picked_pred(pred_list, '/toolbox/side_back'):
                        decision.type = 'start_pick'
                        rospy.logerr("2")
                        decision.parameters = ['/toolbox/side_back']
                        

                    elif not self.check_busy_pred(pred_list, "right"):
                        decision.type = 'start_hold'
                        decision.parameters = ['/toolbox/side_right', '0']
                        
                    else:
                        decision.type = 'wait'
                        

            else:
                if not self.check_busy_pred(pred_list, "left") and not self.check_at_home_pred(pred_list, "left"):
                    decision.type = 'start_go_home_left'
                    
                elif not self.check_busy_pred(pred_list, "right") and not self.check_at_home_pred(pred_list, "right"):
                    decision.type = 'start_go_home_right'
                    
                else:
                    decision.type = 'wait'
                    

        elif len(in_hws_list) == 5:
            if (self.check_attached_pred(pred_list, '/toolbox/side_left', '/toolbox/side_back') and
                self.check_attached_pred(pred_list, '/toolbox/side_right', '/toolbox/side_back')):

                if not self.check_busy_pred(pred_list, "right") and not self.check_at_home_pred(pred_list, "right"):
                    decision.type = 'start_go_home_right'
                    
                elif not self.check_busy_pred(pred_list, "left") and not self.check_at_home_pred(pred_list, "left"):
                    decision.type = 'start_go_home_left'
                    
                else:
                    decision.type = 'wait'
                    
   
            elif (self.check_positioned_pred(pred_list, '/toolbox/side_left', '/toolbox/side_back', 0) and
                self.check_positioned_pred(pred_list, '/toolbox/side_right', '/toolbox/side_back', 1)):


                if not self.check_busy_pred(pred_list, "left") and not self.check_at_home_pred(pred_list, "left"):
                    decision.type = 'start_go_home_left'
                    

                elif not self.check_attached_pred(pred_list, '/toolbox/side_left', '/toolbox/side_back', 0) and not self.check_busy_pred(pred_list, "right"):
                    decision.type = 'start_hold'
                    decision.parameters = ['/toolbox/side_left', '0']
                    
                elif not self.check_busy_pred(pred_list, "right"):
                    decision.type = 'start_hold'
                    decision.parameters = ['/toolbox/side_right', '1']
                    
                else:
                    decision.type = 'wait'
                    

            elif (self.check_positioned_pred(pred_list, '/toolbox/side_left', '/toolbox/side_back', 1) and
                self.check_positioned_pred(pred_list, '/toolbox/side_right', '/toolbox/side_back', 0)):

                if not self.check_busy_pred(pred_list, "left") and not self.check_at_home_pred(pred_list, "left"):
                    decision.type = 'start_go_home_left'
                    

                elif not self.check_attached_pred(pred_list, '/toolbox/side_left', '/toolbox/side_back', 1) and not self.check_busy_pred(pred_list, "right"):
                    decision.type = 'start_hold'
                    decision.parameters = ['/toolbox/side_left', '1']
                    
                elif not self.check_busy_pred(pred_list, "right"):
                    decision.type = 'start_hold'
                    decision.parameters = ['/toolbox/side_right', '0']
                    
                else:
                    decision.type = 'wait'
                    

            else:
                if not self.check_busy_pred(pred_list, "left") and not self.check_at_home_pred(pred_list, "left"):
                    decision = Decision()
                    decision.type = 'start_go_home_left'
                    
                elif not self.check_busy_pred(pred_list, "right") and not self.check_at_home_pred(pred_list, "right"):
                    decision.type = 'start_go_home_right'
                    
                else:
                    decision.type = 'wait'
                    
        decision_response = GetNextDecisionResponse()
        decision_response.mode = decision_response.SURE

        obj_list = ['/toolbox/handle', '/toolbox/side_right', '/toolbox/side_left', '/toolbox/side_front', '/toolbox/side_back']

        decision_response.probas = []

        decision_response.decisions.append(Decision(type="wait", parameters=[]))
        if decision.type == decision_response.decisions[-1].type and decision.parameters == decision_response.decisions[-1].parameters:
            decision_response.probas.append(1.)
        else:
            decision_response.probas.append(0.)

        decision_response.decisions.append(Decision(type="start_go_home_left", parameters=[]))
        if decision.type == decision_response.decisions[-1].type and decision.parameters == decision_response.decisions[-1].parameters:
            decision_response.probas.append(1.)
        else:
            decision_response.probas.append(0.)

        decision_response.decisions.append(Decision(type="start_go_home_right", parameters=[]))
        if decision.type == decision_response.decisions[-1].type and decision.parameters == decision_response.decisions[-1].parameters:
            decision_response.probas.append(1.)
        else:
            decision_response.probas.append(0.)

        for obj in obj_list:
            decision_response.decisions.append(Decision(type="start_give", parameters=[obj]))
            if decision.type == decision_response.decisions[-1].type and decision.parameters == decision_response.decisions[-1].parameters:
                decision_response.probas.append(1.)
            else:
                decision_response.probas.append(0.)

        for obj in obj_list:
            for pose in ["0", "1"]:
                decision_response.decisions.append(Decision(type="start_hold", parameters=[obj, pose]))
                if decision.type == decision_response.decisions[-1].type and decision.parameters == decision_response.decisions[-1].parameters:
                    decision_response.probas.append(1.)
                else:
                    decision_response.probas.append(0.)

        for obj in obj_list:
            decision_response.decisions.append(Decision(type="start_pick", parameters=[obj]))
            if decision.type == decision_response.decisions[-1].type and decision.parameters == decision_response.decisions[-1].parameters:
                decision_response.probas.append(1.)
            else:
                decision_response.probas.append(0.)

        self.sequence += 1
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
