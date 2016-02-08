from . action import Action
import rospy

class GoHome(Action):
    def __init__(self, commander, tf_listener, action_params, poses, seeds, should_interrupt=None):
        super(GoHome, self).__init__(commander, tf_listener, action_params, poses, seeds, should_interrupt)
        self.starting_state = self.commander.get_current_state()

    def run(self, parameters=None):
        rospy.loginfo("Returning in idle mode")
        return self.commander.move_to_controlled(self.starting_state, pause_test=self.pause_test, stop_test=self.stop_test)
