from thr_infrastructure_msgs.msg import RobotAction
from thr_infrastructure_msgs.srv import GetCarryPose, GetCarryPoseRequest
from transformations import pose_to_list, list_to_raw_list
import rospy


class CarryDecisionMapper(object):
    # This is a singleton, never instanciated if action is never executed
    _instance = None

    def init(self):
        rospy.loginfo('CarryDecisionMapper is being instanciated for this run...')
        rospy.wait_for_service('/thr/get_carry_pose')
        self.get_carry_pose = rospy.ServiceProxy('/thr/get_carry_pose', GetCarryPose)

    def map(self, decision):
        object = decision.parameters[0]
        shape = decision.parameters[2]
        type = decision.parameters[1]
        laterality = decision.parameters[3]

        pose = self.get_carry_pose(GetCarryPoseRequest(object=object, method=type, location=shape))
        frame_id = pose.object_pose.header.frame_id
        success = pose.success  # A Decision always success so possible failure is transmitted to the RobotAction
        pose = list_to_raw_list(pose_to_list(pose.object_pose))
        parameters = [object, shape] + map(str, pose) + [frame_id, str(success)]
        return RobotAction(type=decision.type.replace('start_', ''), parameters=parameters)

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(CarryDecisionMapper, cls).__new__(cls, *args, **kwargs)
            cls._instance.init()
        return cls._instance
