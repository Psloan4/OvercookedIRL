#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class ImageViewerNode:
    def __init__(self):
        # Subscribe to the compressed topic
        self.topic = rospy.get_param(
            "~image_topic",
            "/head_camera/rgb/image_raw/compressed",
        )

        rospy.loginfo(f"[image_viewer] Subscribing to image topic: {self.topic}")

        self.sub = rospy.Subscriber(
            self.topic,
            CompressedImage,
            self.image_callback,
            queue_size=1,
        )

        rospy.loginfo("[image_viewer] Subscriber created")

    def image_callback(self, msg: CompressedImage):
        # Decode JPEG/PNG bytes into an OpenCV image
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_img is None:
            rospy.logwarn("Failed to decode compressed image")
            return

        cv2.imshow("Fetch Camera", cv_img)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rospy.signal_shutdown("User pressed 'q'")

def main():
    rospy.init_node("image_viewer", anonymous=False)
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
