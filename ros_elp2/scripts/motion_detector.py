#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class MotionDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.prev_gray = None
        self.motion_pub = rospy.Publisher("/usb_cam/motion_detection", Bool, queue_size=1)
        self.sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=1)
        rospy.loginfo("MotionDetector: подписан на /usb_cam/image_raw")

        self.motion_pixel_threshold = 500
        self.per_pixel_delta = 25

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            motion_detected = False
            if self.prev_gray is not None:
                frame_diff = cv2.absdiff(self.prev_gray, gray)

                _, thresh = cv2.threshold(frame_diff, self.per_pixel_delta, 255, cv2.THRESH_BINARY)

                changed_pixels = np.sum(thresh > 0)

                motion_detected = changed_pixels > self.motion_pixel_threshold

                rospy.loginfo(f"Движение: {motion_detected} | Изменившихся пикселей: {changed_pixels}")

            self.prev_gray = gray.copy()

	    # Использую подходящий тип сообщения для бинарного флага движения
            self.motion_pub.publish(Bool(data=motion_detected))

        except Exception as e:
            rospy.logerr(f"Ошибка в image_callback: {e}")

if __name__ == "__main__":
    rospy.init_node("motion_detector_node", anonymous=False)
    MotionDetector()
    rospy.spin()
    cv2.destroyAllWindows()

