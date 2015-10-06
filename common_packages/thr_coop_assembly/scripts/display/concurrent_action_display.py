#!/usr/bin/env python

import rospy, rospkg
from thr_coop_assembly.msg import ActionHistoryEvent
from threading import Lock
import json, cv2, cv_bridge
from numpy import zeros, uint8
from sensor_msgs.msg import Image
from collections import deque
from baxter_commander import FaceCommander

class ConcurrentActionDisplay(object):
    def __init__(self, width, height, font=cv2.FONT_HERSHEY_SIMPLEX, scale=1, thickness=1, color=[255]*3, interline=1.1):
        self.face = FaceCommander()
        self.rospack = rospkg.RosPack()
        self.history_lock = Lock()
        self.action_history_name = '/thr/action_history'
        self.font = font
        self.scale = scale
        self.thickness = thickness
        self.color = color
        self.interline = interline
        self.queue = deque()
        self.width, self.height = width, height

        self.image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)

        with open(self.rospack.get_path("thr_coop_assembly")+"/config/display.json") as f:
            self.text = json.load(f)
        rospy.Subscriber(self.action_history_name, ActionHistoryEvent, self.cb_action_event_received)

    def cb_action_event_received(self, msg):
        def map_const(index):
            #STARTING = 0
            #FINISHED_SUCCESS = 1
            #FINISHED_FAILURE = 2
            return ['starting', 'finished_success', 'finished_failure'][index]

        with self.history_lock:
            try:
                action = self.text[msg.action.type][map_const(msg.type)]
            except KeyError:
                pass
            else:
                self.display_text(action['text'], msg.action.parameters,
                                  self.font, self.scale, self.thickness, self.color, self.interline)
                if isinstance(action['look_at'], int):
                    if action['look_at'] >= 0:
                        self.face.look_at(msg.action.parameters[action['look_at']])
                    else:
                        self.face.look_at(None)  # Reset gaze at neutral position


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