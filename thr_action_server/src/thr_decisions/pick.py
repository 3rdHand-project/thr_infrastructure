from thr_infrastructure_msgs.msg import RobotAction

def map(decision):
    return RobotAction(type=decision.type.replace('start_', ''), parameters=decision.parameters)
