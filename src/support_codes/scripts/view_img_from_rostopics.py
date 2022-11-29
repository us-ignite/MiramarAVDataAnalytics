#!/usr/bin/env python3

# This code helps display camera feed from a particular rostopic in OpenCV directly from a topic that publishes Images in Compressed format
# Please provide the topic name in the self.cam_topic to view feed from that particular topic.

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


class display_img:
    def __init__(self) -> None:
        self.bridge = CvBridge()
        self.cam__topic = '/cam_front_top_image/compressed'
        self.cam__sub = rospy.Subscriber(self.cam__topic, CompressedImage, self.cam_display_callback)

    def cam_display_callback(self,msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        img = cv2.resize(img, (0,0),fx=0.5, fy=0.5)
        # print(img.shape)
        cv2.imshow("Cam Frame", img)
        # cv2.waitKey()

        if cv2.waitKey(1) == ord('q'):
            cv2.waitey(0)
            cv2.destroyAllWindows()
        

        


def main():
    rospy.init_node('Display_Image',anonymous=True)
    images = display_img()
    rospy.spin()

if __name__ == '__main__':
    main()
