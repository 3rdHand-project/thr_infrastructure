#! /usr/bin/env python
import rospy
import json
import rospkg
import actionlib

from actionlib_msgs.msg import GoalStatus
from thr_coop_assembly.srv import StartStopEpisode, StartStopEpisodeRequest, StartStopEpisodeResponse
from thr_coop_assembly.msg import RunRobotActionAction, RunRobotActionGoal, RunMDPActionGoal, RunMDPActionAction, ActionHistoryEvent, MDPAction

class MDPActionServer:
    """
    This is the action server that transform an MDP action in Robot action for the concurrent system.
    It requires a mapping file mdp_robot_mapping.json
    """
    def __init__(self):
        # Action server attributes
        self.sequence = 1
        self.running = False
        self.server = actionlib.SimpleActionServer('/thr/run_mdp_action', RunMDPActionAction, self.execute, False)
        self.rospack = rospkg.RosPack()
        self.current_actions = {'right': None, 'left': None}
        self.action_history_name = '/thr/action_history'
        self.action_history = rospy.Publisher(self.action_history_name, ActionHistoryEvent, queue_size=10)

        with open(self.rospack.get_path("thr_coop_assembly")+"/config/mdp_robot_mapping.json") as f:
            self.mapping = json.load(f)
        with open(self.rospack.get_path("thr_coop_assembly")+"/config/action_params.json") as f:
            self.action_params = json.load(f)

        # Connect to inner action servers L/R
        self.clients = {'left': actionlib.SimpleActionClient('/thr/robot_run_action/left', RunRobotActionAction),
                        'right': actionlib.SimpleActionClient('/thr/robot_run_action/right', RunRobotActionAction)}
        for name, client in self.clients.iteritems():
            rospy.loginfo('MDP action server for concurrent mode is waiting for action server '+name)
            client.wait_for_server()

        self.server.start()
        self.start_stop_service_name = '/thr/action_server/start_stop'
        rospy.Service(self.start_stop_service_name, StartStopEpisode, self.cb_start_stop)

    def cb_start_stop(self, request):
        if request.command == StartStopEpisodeRequest.START:
            rospy.set_param('/thr/action_server/stopped', False)
        elif request.command == StartStopEpisodeRequest.STOP:
            if self.clients['left'].get_state() in [GoalStatus.ACTIVE, GoalStatus.PREEMPTING]:
                self.clients['left'].cancel_all_goals()
            if self.clients['right'].get_state() in [GoalStatus.ACTIVE, GoalStatus.PREEMPTING]:
                self.clients['right'].cancel_all_goals()
            # Execute the go_homes and wait for them before stopping
            self.execute(mdp_goal=RunMDPActionGoal(action=MDPAction(type='start_go_home_left')))
            self.execute(mdp_goal=RunMDPActionGoal(action=MDPAction(type='start_go_home_right')))
            while not self.clients['right'].get_state() == GoalStatus.ACTIVE and not self.clients['left'].get_state() == GoalStatus.ACTIVE:
                rospy.sleep(0.1)
            rospy.set_param('/thr/action_server/stopped', True)
        return StartStopEpisodeResponse()

    def execute(self, mdp_goal):
        """
        Execute a goal if the server is started or if force mode is enabled
        :param mdp_goal:
        """
        if not rospy.get_param('/thr/action_server/stopped'):
            if mdp_goal.action.type == 'wait':
                self.execute_wait()
            else:
                robot_goal = RunRobotActionGoal()
                try:
                    robot_goal.action.type = self.mapping[mdp_goal.action.type]['type']
                    client = self.mapping[mdp_goal.action.type]['client']
                except KeyError, k:
                    rospy.logerr("No client is capable of action {}{}: KeyError={}".format(mdp_goal.action.type, str(mdp_goal.action.parameters), k.message))
                    self.server.set_aborted()
                else:
                    robot_goal.action.id = self.sequence
                    self.sequence += 1
                    robot_goal.action.parameters = mdp_goal.action.parameters
                    self.clients[client].send_goal(robot_goal)
                    self.current_actions[client] = robot_goal.action

                    # Publish the event to the action history topic
                    event = ActionHistoryEvent()
                    event.header.stamp = rospy.Time.now()
                    event.type = ActionHistoryEvent.STARTING
                    event.action = robot_goal.action
                    event.side = client
                    self.action_history.publish(event)
                    self.server.set_succeeded()

    def should_interrupt(self):
        """
        :return: True if motion should interrupts at that time for whatever reason
        """
        return rospy.is_shutdown() or self.server.is_preempt_requested()

    def update_status(self):
        """
        This method gets the status of the children action clients and update the MDP action server according to them.
        :return:
        """
        for side, action in self.current_actions.iteritems():
            if action: # If an action is running for this arm...
                robot_state = self.clients[side].get_state()
                if robot_state not in [GoalStatus.PENDING, GoalStatus.ACTIVE]: # ... and the action server reports it's ended...
                    state = self.clients[side].get_state()
                    # Publish the event to the action history topic
                    event = ActionHistoryEvent()
                    event.header.stamp = rospy.Time.now()
                    event.type = ActionHistoryEvent.FINISHED_SUCCESS if state == GoalStatus.SUCCEEDED else ActionHistoryEvent.FINISHED_FAILURE
                    event.action = action
                    event.side = side
                    self.action_history.publish(event)
                    self.current_actions[side] = None

            # Update the MDP action state of the corresponding goal here
            #                     if robot_state == GoalStatus.ABORTED:
            #            self.server.set_aborted(None, self.clients.get_goal_status_text())
            #        elif robot_state == GoalStatus.PREEMPTED:
            #            self.server.set_preempted(None, self.clients.get_goal_status_text())
            #    else:
            #        self.server.set_succeeded(None, self.clients.get_goal_status_text())


    def start(self):
        while not rospy.is_shutdown():
            self.update_status()
            rospy.sleep(self.action_params['sleep_step'])

if __name__ == '__main__':
    rospy.init_node('robot_action_server')
    server = MDPActionServer()
    server.start()