#! /usr/bin/env python
import rospy
import rospkg
import json
import tf
import sys
import actionlib
import transformations
from actions import Give, GoHome, Hold, Pick
from baxter_commander import ArmCommander
from thr_coop_assembly.msg import RunRobotActionAction, RunRobotActionActionResult

class RobotActionServer:
    """
    This is the action server that executes actions the robot is capable of.
    It requires two config files:
    * poses.json: poses relative to actions and objects
    * action_params.json: generic parameters for action execution and scenario
    """
    def __init__(self, side):
        # General attributes
        self.rospack = rospkg.RosPack()
        self.side = side
        self.arm_name = side+'_arm'
        self.gripper_name = side+'_gripper'

        # Transform/Geometric attributes
        self.tfl = tf.TransformListener(True, rospy.Duration(5*60)) # TF Interpolation ON and duration of its cache = 5 minutes
        self.world = "base"
        rospy.loginfo("Loading config...")
        with open(self.rospack.get_path("thr_coop_assembly")+"/config/poses.json") as f:
            self.poses = json.load(f)
        with open(self.rospack.get_path("thr_coop_assembly")+"/config/action_params.json") as f:
            self.action_params = json.load(f)
        with open(self.rospack.get_path("thr_coop_assembly")+"/config/seeds.json") as f:
            self.seeds = json.load(f)
        with open(self.rospack.get_path("thr_coop_assembly")+"/config/abilities.json") as f:
            self.abilities = json.load(f)

        # Motion/Grasping attributes
        self.commander = ArmCommander(side, default_kv_max=self.action_params['limits']['kv'], default_ka_max=self.action_params['limits']['ka'], kinematics='robot')

        # Home poses are taken when the server starts:
        self.starting_state = self.commander.get_current_state()
        self.tfl.waitForTransform(self.world, self.gripper_name, rospy.Time(0), rospy.Duration(10))
        self.starting_pose = transformations.list_to_pose(self.tfl.lookupTransform(self.world, self.gripper_name, rospy.Time(0)))

        # Action server attributes
        rospy.loginfo("Starting server "+side)
        self.server = actionlib.SimpleActionServer('/thr/robot_run_action/'+side, RunRobotActionAction, self.execute, False)
        self.result = RunRobotActionActionResult()

        # Actual actions
        self.actions = {
            'give': Give(self.commander, self.tfl, self.action_params, self.poses, self.seeds, self.server.is_preempt_requested),
            'go_home_'+self.side: GoHome(self.commander, self.tfl, self.action_params, self.poses, self.seeds, self.server.is_preempt_requested),
            'hold': Hold(self.commander, self.tfl, self.action_params, self.poses, self.seeds, self.server.is_preempt_requested),
            'pick': Pick(self.commander, self.tfl, self.action_params, self.poses, self.seeds, self.server.is_preempt_requested),
            }

        # On starting blocks...
        self.server.start()
        rospy.loginfo(side+' server ready')

    def execute(self, goal):
        """
        Dispatches a new goal on the method executing each type of action
        :param goal:
        """
        try:
            assert self.abilities[goal.action.type] == self.side
            action = self.actions[goal.action.type]
        except KeyError, AssertionError:
            rospy.logwarn('{} arm is not capable of action {}'.format(self.side, goal.action.type))
            self.server.set_aborted()
        else:
            # Note: do not try/catch: an Exception ends up in an aborted state, ... perfect!
            if action.run(goal.action.parameters):
                self.server.set_succeeded()
            else:
                self.server.set_aborted()


if __name__ == '__main__':
    rospy.init_node('robot_action_server')
    server = RobotActionServer(sys.argv[1])
    rospy.spin()