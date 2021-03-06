#! /usr/bin/env python

import rospy, json
import actionlib

from thr_infrastructure_msgs.msg import *
from thr_infrastructure_msgs.srv import *
from actionlib_msgs.msg import GoalStatus

class InteractionController(object):

    def __init__(self):
        self.running = True
        self.current_scene = None
        self.previous_decision = Decision(type='wait')
        self.scene_before_decision = None
        self.run_decision_name = '/thr/run_decision'
        self.scene_state_service = '/thr/scene_state'

        self.logs = []

        # Initiating topics ands links to services/actions
        self.run_decision_client = actionlib.SimpleActionClient(self.run_decision_name, RunDecisionAction)
        rospy.loginfo("Waiting action client {}...".format(self.run_decision_name))
        self.run_decision_client.wait_for_server()
        self.services = [self.scene_state_service]
        for service in self.services:
            rospy.loginfo("Waiting service {}...".format(service))
            rospy.wait_for_service(service)

        self.start_or_stop_episode(True)  # Start a new (and unique) episode

    def start_or_stop_episode(self, start=True):
        for node in ['scene_state_manager', 'action_server']:
            url = '/thr/{}/start_stop'.format(node)
            rospy.wait_for_service(url)
            rospy.ServiceProxy(url, StartStopEpisode).call(StartStopEpisodeRequest(
                command=StartStopEpisodeRequest.START if start else
                StartStopEpisodeRequest.STOP))
    
    ################################################# SERVICE CALLERS #################################################
    def update_scene(self):
        request = GetSceneStateRequest()
        try:
            getscene = rospy.ServiceProxy(self.scene_state_service, GetSceneState)
            self.current_scene = getscene(request).state
        except rospy.ServiceException, e:
            rospy.logerr("Cannot update scene {}:".format(e.message))

    def wizard_entry(self):
        while not rospy.is_shutdown():
            command = raw_input("> ").strip('\r\n').lower()
            parameters = []
            type = "wait"

            if len(command)<1 or len(command)>4:
                rospy.logerr("Invalid command {} (1a)".format(command))
                continue
            elif command[0] == 'w':
                type = 'wait'
            elif command[0] == 'a':
                type = 'start_grasp'
            elif command[0] == 'g':
                type = 'start_give'
            elif command[0] == 'b':
                type = 'start_bring'
            elif command[0] == 'c':
                type = 'start_place'
            elif command[0] == 'l':
                type = 'start_go_home_left'
            elif command[0] == 'r':
                type = 'start_go_home_right'
            elif command[0] == 'p':
                type = 'start_pick'
            else:
                rospy.logerr("Invalid command {} (1b)".format(command))
                continue

            # Place and Bring also require Left or Right
            if type in ['start_place', 'start_bring']:
                if len(command)<2:
                    rospy.logerr("Invalid command {} (2a)".format(command))
                    continue
                elif command[1] == 'l':
                    type += '_left'
                elif command[1] == 'r':
                    type += '_right'
                else:
                    rospy.logerr("Invalid command {} (2b: missing l or r)".format(command))
                    continue

            if type in ['start_grasp', 'start_pick', 'start_give']:
                if len(command)<2:
                    rospy.logerr("Invalid command {} (2c)".format(command))
                    continue
                elif command[1] == 'p':
                    parameters.append('/romeo/pan')
                elif command[1] == 'b':
                    parameters.append('/romeo/bowl')
                elif command[1] == 'c':
                    parameters.append('/romeo/cover')
                elif command[1] == 'd':
                    parameters.append('/romeo/drugs')
                elif command[1] == 'j':
                    parameters.append('/romeo/juice')
                elif command[1] == 'g':
                    parameters.append('/romeo/glass')
                else:
                    rospy.logerr("Invalid command {} (2d)".format(command))
                    continue

            # Case where two objects are involved: SLAVE object
            if type in ['start_place_left', 'start_place_right', 'start_bring_left', 'start_bring_right']:
                if len(command)<3:
                    rospy.logerr("Invalid command {} (3a)".format(command))
                    continue
                elif command[2] == 'b':
                    parameters.append('/romeo/bowl')
                elif command[2] == 'd':
                    parameters.append('/romeo/drugs')
                elif command[2] == 'j':
                    parameters.append('/romeo/juice')
                elif command[2] == 'g':
                    parameters.append('/romeo/glass')
                elif command[2] == 'c':
                    parameters.append('/romeo/cover')
                else:
                    rospy.logerr("Invalid command {} (3b)".format(command))
                    continue

            # Case where two objects are involved: MASTER object i.e. an object that receives the SLAVE object
            if type in ['start_place_left', 'start_place_right']:
                if len(command)<4:
                    rospy.logerr("Invalid command {} (4a)".format(command))
                    continue
                elif command[3] == 'h':
                    parameters.append('/romeo/hot_plate')
                elif command[3] == 'p':
                    parameters.append('/romeo/pan')
                elif command[3] == 'b':
                    parameters.append('/romeo/bowl')
                elif command[3] == 't':
                    parameters.append('/romeo/tray')
                else:
                    rospy.logerr("Invalid command {} (4b)".format(command))
                    continue
            return type, parameters

    ###################################################################################################################

    def run(self):
        rospy.loginfo('Manual interaction starting from keyboard!')
        try:
            while self.running and not rospy.is_shutdown():
                self.update_scene()
                ret = self.wizard_entry()
                if ret is not None:
                    type, params = ret
                    self.check_for_previous_decisions()  # user inputs are blocking for this setup so update action state at the last time

                    self.logs.append({'timestamp': rospy.get_time(),
                                      'type': type,
                                      'parameters': params})
                    decision = Decision(type=type, parameters=params)
                    self.run_decision(decision)
        finally:
            logs_name = rospy.get_param('/thr/logs_name')
            if logs_name != "none":
                with open('decisions_'+logs_name+'.json', 'w') as f:
                    json.dump(self.logs, f)


    def run_decision(self, decision):
        if self.previous_decision.type== 'wait':
            self.run_decision_client.cancel_all_goals()
            self.scene_before_decision = self.current_scene
            goal = RunDecisionGoal()
            goal.decision = decision
            self.run_decision_client.send_goal(goal)
            self.previous_decision = decision
            rospy.loginfo("You're asking to run decision {}({})".format(decision.type, ', '.join(decision.parameters)))

    def check_for_previous_decisions(self):
        if self.previous_decision.type != 'wait' and self.run_decision_client.get_state() not in [GoalStatus.PENDING, GoalStatus.ACTIVE]:  # ... and the action server reports it's ended...
            state = self.run_decision_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Decision {}({}) succeeded!".format(self.previous_decision.type, ', '.join(self.previous_decision.parameters)))
            else:
                rospy.logwarn("Decision {}({}) failed ;-(".format(self.previous_decision.type, ', '.join(self.previous_decision.parameters)))
            self.previous_decision = Decision(type='wait')

if __name__=='__main__':
    rospy.init_node("interaction_controller")
    InteractionController().run()
