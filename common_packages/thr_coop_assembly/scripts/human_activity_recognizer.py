#!/usr/bin/env python

import rospy, rospkg, tf, transformations, json
from thr_coop_assembly.msg import Predicate, ActionHistoryEvent, MDPAction
from thr_coop_assembly.srv import GetSceneState, GetSceneStateRequest, UpdateRelationalState, UpdateRelationalStateRequest, StartStopEpisode, StartStopEpisodeRequest, StartStopEpisodeResponse
from itertools import product
from threading import RLock

class HumanActivityRecognizer(object):
    def __init__(self, rate):
        self.rate = rate
        self.world = 'base'
        self.service_update = '/thr/update_relational_state'
        self.scene_state_service = '/thr/scene_state'
        self.tfl = tf.TransformListener()
        rospy.wait_for_service(self.service_update)
        self.update_relational_state = rospy.ServiceProxy(self.service_update, UpdateRelationalState)
        self.getscene = rospy.ServiceProxy(self.scene_state_service, GetSceneState)
        self.running_human_activity = None
        self.lock = RLock()
        self.attaching_stamps = {}
        self.scene = rospy.get_param('/thr/scene')
        self.screwdriver = '/tools/screwdriver'
        self.rospack = rospkg.RosPack()
        self.objects = rospy.get_param('/thr/objects')[rospy.get_param('/thr/scene')]
        self.running = False

        self.start_stop_service_name = '/thr/human_activity_recognizer/start_stop'
        rospy.Service(self.start_stop_service_name, StartStopEpisode, self.cb_start_stop)

        self.action_history_name = '/thr/action_history'
        self.action_history = rospy.Publisher(self.action_history_name, ActionHistoryEvent, queue_size=10)

        with open(self.rospack.get_path("thr_coop_assembly")+"/config/scenes/"+self.scene+"/poses.json") as f:
            self.poses = json.load(f)
        with open(self.rospack.get_path("thr_coop_assembly")+"/config/perception.json") as f:
            self.config = json.load(f)

    def cb_start_stop(self, request):
        if request.command == StartStopEpisodeRequest.START:
            with self.lock:
                self.attaching_stamps = {}
            self.running = True

        elif request.command == StartStopEpisodeRequest.STOP:
            self.running = False
        return StartStopEpisodeResponse()

    def pred_position(self, master, slave, atp):
        try:
            # WARNING: Do not ask the relative tf directly, it is outdated!
            tf_slave = self.tfl.lookupTransform(self.world, slave, rospy.Time(0))
            tf_master = self.tfl.lookupTransform(self.world, master, rospy.Time(0))
        except Exception:
            pass
        else:
            relative = transformations.multiply_transform(transformations.inverse_transform(tf_master), tf_slave)
            constraint = self.poses[master]['constraints'][atp][slave]
            cart_dist = transformations.distance(constraint, relative)
            quat_dist = transformations.distance_quat(constraint, relative)
            return (cart_dist < self.config['start_position']['position_tolerance'] and
                    quat_dist < self.config['start_position']['orientation_tolerance']) and\
                not (cart_dist < self.config['positioned']['position_tolerance'] and
                     quat_dist < self.config['positioned']['orientation_tolerance'])

    def pred_screw(self, master, slave, atp, state):
        if Predicate(type='positioned', parameters=[master, slave, str(atp)]) in state.predicates:
            try:
                # WARNING: Do not ask the relative tf directly, it is outdated!
                screwdriver = self.tfl.lookupTransform(self.world, self.screwdriver, rospy.Time(0))
                tf_master = self.tfl.lookupTransform(self.world, master, rospy.Time(0))
            except:
                pass
            else:
                relative = transformations.multiply_transform(transformations.inverse_transform(tf_master), screwdriver)
                cart_dist = transformations.distance(relative, self.poses[master]['constraints'][atp][self.screwdriver])
                if cart_dist < self.config['attached']['tool_position_tolerance']:
                    with self.lock:
                        try:
                            return rospy.Time.now() - self.attaching_stamps[master][slave] < rospy.Duration(
                                self.config['attached']['screwdriver_attaching_time'])
                        except KeyError:
                            if master not in self.attaching_stamps:
                                self.attaching_stamps[master] = {}
                            self.attaching_stamps[master][slave] = rospy.Time.now()
        return False

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.running:
                predicate = Predicate()
                state = self.getscene(GetSceneStateRequest()).state
                if self.running_human_activity is None:
                    for master, slave, atp in product(self.objects, self.objects, [0, 1]):
                        if not ('constraints' in self.poses[master] and len(
                                [c for c in self.poses[master]['constraints'] if slave in c]) > 0):
                            continue
                        if self.pred_position(master, slave, atp):
                            predicate.type = 'position'
                            predicate.parameters = [master, slave, str(atp), "eq1"]
                            break
                        if self.pred_screw(master, slave, atp, state):
                            predicate.type = 'screw'
                            predicate.parameters = [master, slave, str(atp), "eq1"]
                            break
                    if predicate.type != '':
                        request = UpdateRelationalStateRequest(
                            command=UpdateRelationalStateRequest.ADD, predicate=predicate)
                        reply = self.update_relational_state(request)
                        self.running_human_activity = predicate
                        if not reply.success:
                            rospy.logerr('SSM failed to add {}{}'.format(
                                self.running_human_activity.type, str(self.running_human_activity.parameters)))

                        event = ActionHistoryEvent()
                        event.header.stamp = rospy.Time.now()
                        event.type = ActionHistoryEvent.STARTING
                        event.action = MDPAction(type="start_" + self.running_human_activity.type,
                                                 parameters=self.running_human_activity.parameters[:-1])
                        event.side = 'human'
                        self.action_history.publish(event)
                else:
                    still_running = True
                    if self.running_human_activity.type == 'position' and not self.pred_position(
                            self.running_human_activity.parameters[0],
                            self.running_human_activity.parameters[1],
                            int(self.running_human_activity.parameters[2])):
                        still_running = False
                    elif self.running_human_activity.type == 'screw' and not self.pred_screw(
                            self.running_human_activity.parameters[0],
                            self.running_human_activity.parameters[1],
                            int(self.running_human_activity.parameters[2]),
                            state):
                        still_running = False

                    if not still_running:
                        request = UpdateRelationalStateRequest(
                            command=UpdateRelationalStateRequest.REMOVE, predicate=self.running_human_activity)
                        reply = self.update_relational_state(request)
                        if not reply.success:
                            rospy.logerr('SSM failed to remove {}{}'.format(
                                self.running_human_activity.type, str(self.running_human_activity.parameters)))

                        event = ActionHistoryEvent()
                        event.header.stamp = rospy.Time.now()
                        event.type = ActionHistoryEvent.FINISHED_SUCCESS
                        event.action = MDPAction(type="start_" + self.running_human_activity.type,
                                                 parameters=self.running_human_activity.parameters[:-1])
                        event.side = 'human'
                        self.action_history.publish(event)
                        self.running_human_activity = None
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('human_activity_recognizer')
    HumanActivityRecognizer(20).run()
