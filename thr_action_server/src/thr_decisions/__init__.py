from . import pick, give, hold, grasp, go_home, place, bring, turn
from thr_infrastructure_msgs.msg import Decision, RobotAction

def decision_action_mapping(decision):
    """
    Map an high-level logic decision to a low-level robot action
    :param decision: The Decision() to map
    :return The couple (robot_action, client) corresponding to RobotAction() 
    """
    if decision.type=='start_pick':
        return pick.map(decision), 'left'
    elif decision.type=='start_give':
        return give.map(decision), 'left'
    elif decision.type=='start_hold':
        return hold.map(decision), 'right'
    elif decision.type=='start_grasp':
        return grasp.map(decision), 'right'
    elif decision.type=='start_turn':
        return turn.map(decision), 'right'
    elif decision.type=='start_go_home_left':
        return go_home.map(decision), 'left'
    elif decision.type=='start_go_home_right':
        return go_home.map(decision), 'right'
    elif decision.type=='start_place_left':
        return place.map(decision), 'left'
    elif decision.type=='start_place_right':
        return place.map(decision), 'right'
    elif decision.type=='start_bring_left':
        return bring.map(decision), 'left'
    elif decision.type=='start_bring_right':
        return bring.map(decision), 'right'
    else:
        raise NotImplementedError("Unknown mapping of decision {} to RobotAction, make sure a mapping exists in {}".format(decision.type, __file__))

