from . action import Action
import rospy
from baxter_commander.persistence import dicttostate
import json

class GoToJS(Action):
    def __init__(self, commander, tf_listener, action_params, poses, seeds, should_interrupt=None):
        super(GoToJS, self).__init__(commander, tf_listener, action_params, poses, seeds, should_interrupt)
        self.starting_state = self.commander.get_current_state()

    def run(self, parameters=None):
        rospy.loginfo("Returning in idle mode")
        if parameters is None or parameters[0] == "default":
            return self.commander.move_to_controlled(self.starting_state, pause_test=self.pause_test)
        else:
            # read pose from file
            with open(parameters[0]) as f:
                data = json.load(f)
            goal = dicttostate(data['state'])
            return self.commander.move_to_controlled(goal, pause_test=self.pause_test)
        # No stop_test=, Go Home can't be interrupted
