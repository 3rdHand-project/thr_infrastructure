#!/usr/bin/env python
import rospy
import actionlib

from thr_coop_assembly.msg import *
from thr_coop_assembly.srv import *
from actionlib_msgs.msg import *
from copy import deepcopy

class InteractionController(object):
    def __init__(self):
        self.running = True
        self.scene_before_action = None
        self.current_scene = None
        self.current_action = None
        self.user_commands = []

        # Parameters to be tweaked
        self.interaction_loop_rate = rospy.Rate(2)  # Rate of the interaction loop in Hertz
        self.bad_user_commands = [UserCommand.GIVE, UserCommand.HOLD] # User commands implying a BAD reward during execution of an action
        self.user_cmd_service = '/thr/usercommands'
        self.reward_service = '/thr/learner'
        self.predictor_service = 'thr/predictor'
        self.scene_state_service = '/thr/scene_state'
        self.run_action_name = '/thr/run_action'

        # Initiating links to services and actions
        self.run_action_client = actionlib.SimpleActionClient(self.run_action_name, RunActionAction)
        rospy.loginfo("Waiting action client RunAction...")
        self.run_action_client.wait_for_server()
        for service in [self.reward_service, self.predictor_service, self.scene_state_service]:#, self.user_cmd_service]:
            rospy.loginfo("Waiting service {}...".format(service))
            rospy.wait_for_service(service)

    ################################################# SERVICE CALLERS #################################################
    def get_user_commands(self):
        return 0 # TODO TODO TODO
        request = GetUserCommandsRequest()
        self.user_commands = []
        try:
            getusercmd = rospy.ServiceProxy(self.user_cmd_service, GetUserCommands)
            self.current_scene = getusercmd(request)
        except rospy.ServiceException, e:
            rospy.logerr("Cannot fetch user commands {}:".format(e.message))
            return 0
        return len(self.user_commands)

    def send_reward(self, good):
        request = SetNewTrainingExampleRequest()
        request.action = self.current_action
        request.scene_state = self.scene_before_action
        request.good = good
        try:
            reward = rospy.ServiceProxy(self.reward_service, SetNewTrainingExample)
            reward(request)
        except rospy.ServiceException, e:
            rospy.logerr("Cannot send reward {}:".format('good' if good else 'bad', e.message))

    def horrible_print_preds(self):
        attached = []
        inhws = []
        positioned=[]
        for p in self.current_scene.predicates:
            if p.type==Predicate.ATTACHED:
                attached.append(p)
            elif p.type==Predicate.IN_HUMAN_WS:
                inhws.append(p)
            elif p.type==Predicate.POSITIONED:
                positioned.append(p)
        rospy.logwarn("ATTACHED")
        for a in attached:
            rospy.logwarn(' '.join(a.objects))
        rospy.logwarn("IN_HUMAN_WS")
        for a in inhws:
            rospy.logwarn(' '.join(a.objects))
        rospy.logwarn("POSITIONNED")
        for a in positioned:
            rospy.logwarn(' '.join(a.objects))

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
        predicted_cmd = Action(type=Action.WAIT)
        try:
            predict = rospy.ServiceProxy(self.predictor_service, GetNextAction)
            predicted_cmd = predict(request)
        except rospy.ServiceException, e:
            rospy.logerr("Cannot call predictor:".format(e.message))
        return predicted_cmd.action
    ###################################################################################################################

    def command_mapper(self, usercommand):
        """
        Map symbolic user commands in system actions
        :param usercommand:
        :return: The system action passed in input, or wait if mapping unknown
        """
        c = Action()
        c.type = Action.WAIT
        if usercommand.type==UserCommand.GIVE:
            c.type = Action.GIVE
            c.give = Give()
            raise Exception("Must tf.lookupTransform() and fill the parameters of Action")
        elif usercommand.type==UserCommand.HOLD:
            c.type = Action.HOLD
            c.hold = Hold()
        return c

    def pause_interaction(self):
        rate_pause = rospy.Rate(10)
        while UserCommand.RESTART not in self.get_user_commands_types() and not rospy.is_shutdown():
            self.get_user_commands()
            rate_pause.sleep()

    def get_user_commands_types(self):
        """
        :return: the numeric types of received user commands
        """
        return [u.type for u in self.user_commands]

    def run(self):
        print 'Interaction starting!'
        while self.running and not rospy.is_shutdown():
            self.update_scene()
            #self.horrible_print_preds()
            cmd = self.predict()
            self.run_action(cmd)
            self.user_commands = []
            self.current_action = None
            self.interaction_loop_rate.sleep()

    def run_action(self, action):
        self.current_action = action
        self.scene_before_action = deepcopy(self.current_scene)
        goal = RunActionGoal()
        goal.action = action

        # Action is started
        rospy.loginfo("Starting action {}".format(action.type))
        self.run_action_client.send_goal(goal)   # feedback and transition callbacks, it's here
        while self.run_action_client.get_state() in [GoalStatus.PENDING, GoalStatus.ACTIVE] and not rospy.is_shutdown():
            if self.get_user_commands():
                if UserCommand.PAUSE in self.get_user_commands_types():
                    rospy.loginfo("Interaction paused")
                    self.pause_interaction()
                    rospy.loginfo("Interaction restarted")
                    self.user_commands = []
            else:
                self.interaction_loop_rate.sleep()

        # Action has now finished, been cancelled, or failed
        if not rospy.is_shutdown():
            # Check for success or failure
            state = self.run_action_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Action {} succeeded!".format(self.current_action.type))
                self.send_reward(True)
            elif set(self.get_user_commands_types()).intersection(set(self.bad_user_commands)):
                rospy.logwarn("Cancelling action {}".format(self.current_action.type))
                self.run_action_client.cancel_goal()
                self.send_reward(False)
                self.run_action(self.command_mapper(self.user_commands[0]))

if __name__=='__main__':
    rospy.init_node("interaction_controller")
    InteractionController().run()