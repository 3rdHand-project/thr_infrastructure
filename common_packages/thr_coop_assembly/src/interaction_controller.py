import os, rospy
import actionlib

from thr_coop_assembly.msg import *
from thr_coop_assembly.srv import *
from actionlib_msgs.msg import GoalStatus
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
        self.user_cmd_service = '/thr/user_mdp_actions'
        self.reward_service = '/thr/learner'
        self.predictor_service = 'thr/predictor'
        self.scene_state_service = '/thr/scene_state'
        self.run_action_name = '/thr/run_mdp_action'

        # Initiating topics ands links to services/actions
        self.run_action_client = actionlib.SimpleActionClient(self.run_action_name, RunMDPActionAction)
        rospy.loginfo("Waiting action client {}...".format(self.run_action_name))
        self.run_action_client.wait_for_server()
        for service in [self.reward_service, self.predictor_service, self.scene_state_service, self.user_cmd_service]:
            rospy.loginfo("Waiting service {}...".format(service))
            rospy.wait_for_service(service)

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

    def update_user_inputs(self):
        request = GetUserMDPActionRequest()
        try:
            get_ui = rospy.ServiceProxy(self.user_cmd_service, GetUserMDPAction)
        except rospy.ServiceException, e:
            rospy.logerr("Cannot update user inputs {}:".format(e.message))
            return MDPAction(type='wait')
        else:
            return get_ui(request).action

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
        raise Exception("This is the mother class of Interaction controller, you must inherit from it")

    def run_action(self, action):
        if not self.current_action:
            if action.type!='wait':
                os.system('beep')
            self.scene_before_action = deepcopy(self.current_scene)
            goal = RunMDPActionGoal()
            goal.action = action
            self.run_action_client.send_goal(goal)
            self.current_action = action

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
                    self.current_action = None
