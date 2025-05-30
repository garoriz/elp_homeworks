#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2

class ImageSubscriber:
    def __init__(self):
        rospy.init_node('image_subscriber_node', anonymous=False)
        self.sub = rospy.Subscriber("/usb_cam/image_raw", Image, queue_size=1)
        rospy.loginfo("Подписка на /usb_cam/image_raw запущена.")

if __name__ == '__main__':
    try:
        ImageSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

