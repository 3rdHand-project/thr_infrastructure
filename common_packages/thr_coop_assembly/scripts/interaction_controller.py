#!/usr/bin/env python
import os, rospy
import actionlib

from thr_coop_assembly.msg import *
from thr_coop_assembly.srv import *
from actionlib_msgs.msg import *
from copy import deepcopy

class InteractionController(object):
    def __init__(self):
        self.running = True
        self.current_scene = None
        self.user_commands = []
        self.current_action = None
        self.scene_before_action = None

        # Parameters to be tweaked
        self.interaction_loop_rate = rospy.Rate(2)  # Rate of the interaction loop in Hertz
        self.user_cmd_service = '/thr/usercommands'
        self.reward_service = '/thr/learner'
        self.predictor_service = 'thr/predictor'
        self.scene_state_service = '/thr/scene_state'
        self.run_action_name = '/thr/run_action'
        self.action_history_name = '/thr/action_history'

        # Initiating topics ands links to services/actions
        self.run_action_client = actionlib.SimpleActionClient(self.run_action_name, RunActionAction)
        rospy.loginfo("Waiting action client {}...".format(self.run_action_name))
        self.run_action_client.wait_for_server()
        for service in [self.reward_service, self.predictor_service, self.scene_state_service]:#, self.user_cmd_service]:
            rospy.loginfo("Waiting service {}...".format(service))
            rospy.wait_for_service(service)
        self.action_history = rospy.Publisher(self.action_history_name, ActionHistoryEvent, queue_size=10)

    ################################################# SERVICE CALLERS #################################################
    def send_reward(self, good, scene_before_action):
        request = SetNewTrainingExampleRequest()
        request.action = self.current_action
        request.scene_state = scene_before_action
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
            predicted_cmd = predict(request)
        except rospy.ServiceException, e:
            rospy.logerr("Cannot call predictor:".format(e.message))
            return MDPAction(type='wait')
        return predicted_cmd.action
    ###################################################################################################################

    def run(self):
        print 'Interaction starting!'
        while self.running and not rospy.is_shutdown():
            self.update_scene()
            action = self.predict()
            self.run_action(action)
            self.action_postprocessing()
            self.user_commands = []
            self.interaction_loop_rate.sleep()

    def run_action(self, action):
        if not self.current_action:
            if action.type!='wait':
                os.system('beep')
            self.scene_before_action = deepcopy(self.current_scene)
            goal = RunActionGoal()
            goal.action = action
            self.run_action_client.send_goal(goal)

            self.current_action = action

            # Publish the event to the action history topic
            event = ActionHistoryEvent()
            event.header.stamp = rospy.Time.now()
            event.type = ActionHistoryEvent.STARTING
            event.action = action
            self.action_history.publish(event)

    def action_postprocessing(self):
            if self.current_action: # If an action is running for this arm...
                if self.run_action_client.get_state() not in [GoalStatus.PENDING, GoalStatus.ACTIVE]: # ... and the action server reports it's ended...
                    # ... then we have a good or bad reward to send
                    state = self.run_action_client.get_state()
                    if state == GoalStatus.SUCCEEDED:
                        rospy.loginfo("Action {} succeeded!".format(self.current_action.type))
                        self.send_reward(True, self.scene_before_action)
                    #elif set(self.get_user_commands_types()).intersection(set(self.bad_user_commands)):
                    #    rospy.logwarn("Cancelling action {}".format(self.current_action.type))
                    #    self.run_action_client.cancel_goal()
                    #    self.send_reward(False, self.scene_before_action)
                    #    self.run_action(self.command_mapper(self.user_commands[0]))

                    # Publish the event to the action history topic
                    event = ActionHistoryEvent()
                    event.header.stamp = rospy.Time.now()
                    event.type = ActionHistoryEvent.FINISHED_SUCCESS if state == GoalStatus.SUCCEEDED else ActionHistoryEvent.FINISHED_FAILURE
                    event.action = self.current_action
                    self.action_history.publish(event)

                    self.current_action = None


if __name__=='__main__':
    rospy.init_node("interaction_controller")
    InteractionController().run()