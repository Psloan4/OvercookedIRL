#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys

class ImageViewerNode:
    def __init__(self):
        self.bridge = CvBridge()

        self.topic = rospy.get_param("~image_topic", "/head_camera/rgb/image_raw")

        rospy.loginfo(f"[image_viewer] Subscribing to image topic: {self.topic}")

        self.sub = rospy.Subscriber(self.topic, Image, self.image_callback, queue_size=1)

    def image_callback(self, msg):
        try:
            # Most RGB camera topics are 'bgr8' or 'rgb8'; Fetch head RGB is bgr8
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"cv_bridge error: {e}")
            return

        cv2.imshow("Fetch Camera", cv_img)
        # WaitKey needed to keep window responsive; 1ms is enough
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rospy.signal_shutdown("User pressed 'q'")

def main():
    rospy.init_node("image_viewer", anonymous=True)
    viewer = ImageViewerNode()
    rospy.loginfo("Image viewer node started. Close the window or press 'q' to exit.")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image viewer...")
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
