#!/usr/bin/env python

import rospy, rospkg, tf, transformations, json
from thr_infrastructure_msgs.msg import Predicate, ActionHistoryEvent, Decision
from thr_infrastructure_msgs.srv import GetSceneState, GetSceneStateRequest, UpdateRelationalState, UpdateRelationalStateRequest, StartStopEpisode, StartStopEpisodeRequest, StartStopEpisodeResponse
from itertools import product


class ToolBoxSceneStateUpdater(object):
    def __init__(self, rate):
        self.rate = rate
        self.world = 'base'
        self.service_update = '/thr/update_relational_state'
        self.scene_state_service = '/thr/scene_state'
        self.action_history_name = '/thr/action_history'

        self.tfl = tf.TransformListener()
        rospy.wait_for_service(self.service_update)
        self.update_relational_state = rospy.ServiceProxy(self.service_update, UpdateRelationalState)
        self.getscene = rospy.ServiceProxy(self.scene_state_service, GetSceneState)
        self.running_human_activity = None

        # Predicate holders
        self.old_predicates = []
        self.attaching_stamps = {}
        self.attached = [] # Pairs of attached objects on the form o1_o2 with o1<o2
        self.screwed = [] # Pairs of screwed objects (screwdriver 7 seconds => screwed ; screw + wrist > 0.6 m => attached)

        self.scene = rospy.get_param('/thr/scene')
        self.screwdriver = '/tools/screwdriver'
        self.rospack = rospkg.RosPack()
        self.objects = rospy.get_param('/thr/objects')[rospy.get_param('/thr/scene')]
        self.running = False

        self.start_stop_service_name = '/thr/human_activity_recognizer/start_stop'
        rospy.Service(self.start_stop_service_name, StartStopEpisode, self.cb_start_stop)
        self.action_history = rospy.Publisher(self.action_history_name, ActionHistoryEvent, queue_size=10)

        with open(self.rospack.get_path("thr_scenes")+"/config/"+self.scene+"/poses.json") as f:
            self.poses = json.load(f)
        with open(self.rospack.get_path("thr_scene_state_manager")+"/config/perception.json") as f:
            self.config = json.load(f)

    def cb_start_stop(self, request):
        if request.command == StartStopEpisodeRequest.START:
            self.running_human_activity = None
            self.old_predicates = []
            self.attaching_stamps = {}
            self.attached = []
            self.screwed = []
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
                    try:
                        return rospy.Time.now() - self.attaching_stamps[master][slave] < rospy.Duration(
                            self.config['attached']['screwdriver_attaching_time'])
                    except KeyError:
                        if master not in self.attaching_stamps:
                            self.attaching_stamps[master] = {}
                        self.attaching_stamps[master][slave] = rospy.Time.now()
        return False

    def pred_positioned(self, master, slave, atp):
        """
        Checks if any constraint between master and slave exists at attach point atp and returns True if the constraint is within the tolerance
        :param master:
        :param slave:
        :param atp: (int)
        :return: True if predicate POSITIONED(master, slave, atp) is True
        """
        if master+slave+str(atp) in self.attached:
            return True
        try:
            # WARNING: Do not ask the relative tf directly, it is outdated!
            tf_slave = self.tfl.lookupTransform(self.world, slave, rospy.Time(0))
            tf_master = self.tfl.lookupTransform(self.world, master, rospy.Time(0))
        except Exception, e:
            pass
        else:
            relative = transformations.multiply_transform(transformations.inverse_transform(tf_master), tf_slave)
            constraint = self.poses[master]['constraints'][atp][slave]
            cart_dist = transformations.distance(constraint, relative)
            quat_dist = transformations.distance_quat(constraint, relative)
            return cart_dist<self.config['positioned']['position_tolerance'] and quat_dist<self.config['positioned']['orientation_tolerance']
        return False

    def pred_attached(self, master, slave, atp):
        if master+slave+str(atp) in self.attached:
            return True
        elif self.pred_positioned(master, slave, atp):
            try:
                # WARNING: Do not ask the relative tf directly, it is outdated!
                screwdriver = self.tfl.lookupTransform(self.world, self.screwdriver, rospy.Time(0))
                tf_master = self.tfl.lookupTransform(self.world, master, rospy.Time(0))
            except:
                pass
            else:
                if self.screwdriver in self.poses[master]['constraints'][atp]:  # For objects that need to be screwed
                    relative = transformations.multiply_transform(transformations.inverse_transform(tf_master), screwdriver)
                    cart_dist = transformations.distance(relative, self.poses[master]['constraints'][atp][self.screwdriver])
                    if cart_dist < self.config['attached']['tool_position_tolerance']:
                        try:
                            if rospy.Time.now() - self.attaching_stamps[master][slave] > rospy.Duration(self.config['attached']['screwdriver_attaching_time']):
                                rospy.logwarn("[Scene state manager] User has attached {} and {}".format(master, slave))
                                # self.screwed.append(master+slave+str(atp))
                                self.attached.append(master+slave+str(atp))
                        except KeyError:
                            if not self.attaching_stamps.has_key(master):
                                self.attaching_stamps[master] = {}
                            self.attaching_stamps[master][slave] = rospy.Time.now()
                else: # For objects that only need to be inserted
                    # self.screwed.append(master+slave+str(atp))
                    self.attached.append(master+slave+str(atp))
        return False

    def check_new_activity_predicate(self, master, slave, atp, state):
        """
        Generate the predicate related to human activities, if no one is already known to the SSU
        We consider that human is not threaded so only 1 predicate can be generated here
        """
        if self.running_human_activity is None:
            predicate = Predicate()
            if self.pred_position(master, slave, atp):
                predicate.type = 'position'
                predicate.parameters = [master, slave, str(atp), "eq1"]
            elif self.pred_screw(master, slave, atp, state):
                predicate.type = 'screw'
                predicate.parameters = [master, slave, str(atp), "eq1"]

            if predicate.type != '':
                self.running_human_activity = predicate
                self.add_predicate(predicate)

                # Human has no action server so he can't publish its action history, we do this now
                event = ActionHistoryEvent()
                event.header.stamp = rospy.Time.now()
                event.type = ActionHistoryEvent.FINISHED_SUCCESS
                event.action = Decision(type="start_" + self.running_human_activity.type,
                                        parameters=self.running_human_activity.parameters[:-1])
                event.side = 'human'
                self.action_history.publish(event)

    def check_ended_human_activity(self, state):
        """
        If the SSU knows a running human activity, check that it's still active and disable it if not
        """
        if self.running_human_activity is not None:
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
                self.remove_predicate(self.running_human_activity)
                self.running_human_activity = None

    def add_predicate(self, predicate):
        request = UpdateRelationalStateRequest(command=UpdateRelationalStateRequest.ADD, predicate=predicate)
        reply = self.update_relational_state(request)
        if not reply.success:
            rospy.logerr('SSU failed to add {}{}'.format(predicate.type, str(predicate.parameters)))

    def remove_predicate(self, predicate):
        request = UpdateRelationalStateRequest(command=UpdateRelationalStateRequest.REMOVE, predicate=predicate)
        reply = self.update_relational_state(request)
        if not reply.success:
            rospy.logerr('SSU failed to remove {}{}'.format(predicate.type, str(predicate.parameters)))

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.running:
                current_predicates = []
                # Update the scene state predicates
                state = self.getscene(GetSceneStateRequest()).state
                for master, slave, atp in product(self.objects, self.objects, [0, 1]):
                    if not ('constraints' in self.poses[master] and len(
                            [c for c in self.poses[master]['constraints'] if slave in c]) > 0):
                        continue
                    if self.pred_positioned(master, slave, atp):
                        current_predicates.append(Predicate(type='positioned', parameters=[master, slave, str(atp)]))
                    if self.pred_attached(master, slave, atp):
                        current_predicates.append(Predicate(type='attached', parameters=[master, slave, str(atp)]))

                    # Update the Human Activities that could be performed on these objects
                    self.check_new_activity_predicate(master, slave, atp, state)
                self.check_ended_human_activity(state)

                union = self.old_predicates + current_predicates
                to_add = [p for p in union if p not in self.old_predicates]
                to_rm = [p for p in union if p not in current_predicates]
                for predicate in to_add:
                    self.add_predicate(predicate)
                for predicate in to_rm:
                    self.remove_predicate(predicate)
                self.old_predicates = current_predicates

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('scene_state_updater')
    ToolBoxSceneStateUpdater(20).run()
