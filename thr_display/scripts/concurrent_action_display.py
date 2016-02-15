#!/usr/bin/env python

import rospy, rospkg
from thr_infrastructure_msgs.msg import ActionHistoryEvent
from tf import LookupException
import json, cv2, cv_bridge
from numpy import zeros, uint8
from sensor_msgs.msg import Image
from collections import deque
from baxter_commander import FaceCommander

class ConcurrentActionDisplay(object):
    def __init__(self, width, height, font=cv2.FONT_HERSHEY_SIMPLEX, scale=1, thickness=1, color=[255]*3, interline=1.1):
        self.face = FaceCommander()
        self.rospack = rospkg.RosPack()
        self.action_history_name = '/thr/action_history'
        self.font = font
        self.scale = scale
        self.thickness = thickness
        self.color = color
        self.interline = interline
        self.queue = deque()
        self.width, self.height = width, height
        self.events = []  # Current events are stored in this stack
        self.scene = rospy.get_param('/thr/scene')
        self.image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)

        with open(self.rospack.get_path("thr_scenes")+"/config/"+self.scene+"/display.json") as f:
            self.text = json.load(f)

        self.display_text(self.text['start'], [], self.font, self.scale, self.thickness, self.color, self.interline)
        rospy.Subscriber(self.action_history_name, ActionHistoryEvent, self.cb_action_event_received)

    def react_to_event(self, event):
        """
        React to the specified event
        :param event:
        :return: False in case no reaction is needed, True otherwise
        """
        def map_const(index):
            #STARTING = 0
            #FINISHED_SUCCESS = 1
            #FINISHED_FAILURE = 2
            return ['starting', 'finished_success', 'finished_failure'][index]

        try:
            action = self.text[event.action.type][map_const(event.type)]
        except KeyError:
            return False
        else:
            self.display_text(action['text'], event.action.parameters,
                              self.font, self.scale, self.thickness, self.color, self.interline)
            if isinstance(action['look_at'], int):
                try:
                    if action['look_at'] >= 0:
                        self.face.look_at(event.action.parameters[action['look_at']])
                    else:
                        self.face.look_at(None)  # Reset gaze at neutral position
                except LookupException:
                    # Given object not found, ignore
                    pass
            return True

    def cb_action_event_received(self, msg):
        def del_event():
            # Deletes the last event and its associated STARTING event, if any
            if self.events[-1].type != ActionHistoryEvent.STARTING:
                # Remove the STARTING action corresponding to the last event
                for event in range(len(self.events)-1, -1, -1):
                    if self.events[event].action.id == self.events[-1].action.id and self.events[event].type == ActionHistoryEvent.STARTING:
                        del self.events[event]
                        break
                # Remove the FINISHED action
                self.events.pop()

        self.events.append(msg)
        need_reaction = self.react_to_event(msg)
        if msg.type != ActionHistoryEvent.STARTING:
            del_event()
        if not need_reaction and len(self.events)>0:
            self.react_to_event(self.events[-1])


    def display_text(self, lines, parameters=[], font=cv2.FONT_HERSHEY_SIMPLEX, scale=1, thickness=1, color=[255]*3, interline=1.1):
        def center(sentence):
            (width, height), _ = cv2.getTextSize(sentence, font, scale, thickness)
            return (self.width-width)/2, height

        y0 = (self.height - len(lines)*cv2.getTextSize('_', font, scale, thickness)[0][1]*interline)/2
        img = zeros((self.height, self.width, 3), uint8)
        for line_i, line in enumerate(lines):
            if isinstance(line, int):  # ints are expanded with their associated parameter
                line = parameters[line].split('/')[-1].replace('_', ' ').upper()
            x = center(line)[0]
            y = y0 + int(center(line)[1]*(line_i+1)*interline)
            cv2.putText(img, line, (x, y), font, scale, color, thickness=thickness)

        #cv2.imshow("Screen", img)
        #cv2.waitKey(1)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        self.image_pub.publish(msg)



if __name__ == "__main__":
    rospy.init_node('concurrent_action_display')
    ConcurrentActionDisplay(1024, 600, font=cv2.FONT_HERSHEY_SIMPLEX, scale=3, thickness=2, interline=2)
    rospy.spin()