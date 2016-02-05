import transformations
from rospy import is_shutdown, get_param

class Action(object):
    """
    Abstract class representing an executable action
    """
    def __init__(self, commander, tf_listener, action_params, poses, seeds, should_interrupt=None):
        self.__should_interrupt = should_interrupt
        self.world = "base"
        self.commander = commander
        self.tfl = tf_listener
        self.action_params = action_params
        self.poses = poses
        self.seeds = seeds

    def pause_test(self):
        return get_param('/thr/paused')

    def _object_grasp_pose_to_world(self, poselist, object):
        """
        Returns the transformation from world to the point defined by "poselist attached to object frame" aka
        a constraint or a grasp pose.
        :param poselist: A poselist [[x, y, z], [x, y, z, w]]
        :param object: The frame where the pose should be attached
        :return: the transformation from world to the point defined by "poselist attached to object frame" (PoseStamped)
        """
        obj_pose = transformations.list_to_pose(poselist, object)
        world_pose = self.tfl.transformPose(self.world, obj_pose)
        return world_pose

    def _should_interrupt(self):
        """
        :return: True if motion should interrupts at that time for whatever reason
        """
        return is_shutdown() or callable(self.__should_interrupt) and self.__should_interrupt()

    def run(self, parameters=None):
        raise NotImplementedError("Action is an abstract class for actions and can't be instanciated")