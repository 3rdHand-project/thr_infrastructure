#!/usr/bin/env python

import rospy, rospkg
from thr_coop_assembly.srv import GetSceneStateRequest, GetSceneState
import cv2, cv_bridge
from numpy import zeros, uint8
from sensor_msgs.msg import Image
import os
class ConcurrentDebugDisplay(object):
    def __init__(self, width, height, rate, face=cv2.FONT_HERSHEY_SIMPLEX):
        self.rospack = rospkg.RosPack()
        self.face = face
        self.rate = rospy.Rate(rate)
        self.width, self.height = width, height

        self.state = None
        self.old_state = None

        self.image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)

        rospy.loginfo("[concurrent_debug_display] Waiting service /thr/scene_state...")
        rospy.wait_for_service('/thr/scene_state')
        self.getscene = rospy.ServiceProxy('/thr/scene_state', GetSceneState)

    def update_scene(self):
        request = GetSceneStateRequest()
        try:
            scene = self.getscene(request).state
        except rospy.ServiceException, e:
            rospy.logerr("[concurrent_debug_display] Cannot update scene {}:".format(e.message))
        else:
            self.old_state = self.state
            self.state = scene

    def display_image(self):
        img = zeros((self.height, self.width, 3), uint8)
        preds = {"attached": [], "in_hws": [], "positioned": [], "busy": [], "picked": [], "held": [], "at_home": [], "activity": []}
        for p in self.state.predicates:
            if p.type=='in_human_ws':
                preds["in_hws"].append(p)
            elif p.type=='positioned':
                preds["positioned"].append(p)
            elif p.type=='attached':
                preds["attached"].append(p)
            elif p.type=='picked':
                preds["picked"].append(p)
            elif p.type=='holded':
                preds["held"].append(p)
            elif p.type=='busy':
                preds["busy"].append(p)
            elif p.type=='at_home':
                preds["at_home"].append(p)
            else:
                preds["activity"].append(p)

        # Now draw the image with opencv
        line = 1
        for i_pred, pred in preds.iteritems():
            if i_pred=='activity':
                continue
            cv2.putText(img, '#'+i_pred.upper()+' ['+str(len(pred))+']', (10, 20*line), self.face, 0.55, [255]*3)
            line+=1
            for i, p in enumerate(pred):
                cv2.putText(img, str(p.parameters), (50, 20*line), self.face, 0.5, [180]*3)
                line += 1

        # Column 2: activities
        cv2.putText(img, '# ACTIVITIES ['+str(len(preds['activity']))+']', (self.width/2, 20), self.face, 0.55, [255]*3)
        line = 2
        for i, p in enumerate(preds['activity']):
            cv2.putText(img, p.type+str(p.parameters), (self.width/2, 20*line), self.face, 0.5, [180]*3)
            line += 1

        #cv2.imshow("Predicates", img)
        #cv2.waitKey(1)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        self.image_pub.publish(msg)

    def start(self):
        while not rospy.is_shutdown():
            self.update_scene()
            if not self.old_state or self.state.predicates != self.old_state.predicates:
                self.display_image()

if __name__ == "__main__":
    rospy.init_node('concurrent_debug_display')
    ConcurrentDebugDisplay(1024, 600, 2).start()