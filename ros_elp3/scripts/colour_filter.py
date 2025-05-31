#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from functools import reduce
import sys

class ColorFilterNode:
    def __init__(self):
        rospy.init_node('color_filter_node', anonymous=False)
        self.bridge = CvBridge()
        self.color_name = self.get_color_from_args().lower()

        self.sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback, queue_size=1)
        rospy.loginfo(f"Фильтрация по цвету: {self.color_name}")
        
    def get_color_from_args(self):
        for arg in sys.argv:
            if arg.startswith("color:="):
                return arg.split(":=")[1]
        rospy.logwarn("Параметр 'color' не задан, используется 'красный'")
        return "красный"

    def get_color_bounds(self, color_name):
        if color_name == "красный":
            lower1 = np.array([0, 100, 100])
            upper1 = np.array([10, 255, 255])
            lower2 = np.array([160, 100, 100])
            upper2 = np.array([179, 255, 255])
            return [(lower1, upper1), (lower2, upper2)]
        elif color_name == "зеленый":
            lower = np.array([40, 100, 100])
            upper = np.array([80, 255, 255])
            return [(lower, upper)]
        elif color_name == "синий":
            lower = np.array([100, 100, 100])
            upper = np.array([130, 255, 255])
            return [(lower, upper)]
        else:
            rospy.logwarn("Неизвестный цвет. Использую фильтр по умолчанию: красный.")
            return self.get_color_bounds("красный")

    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            masks = []
            for lower, upper in self.get_color_bounds(self.color_name):
                masks.append(cv2.inRange(hsv, lower, upper))
            mask = reduce(cv2.bitwise_or, masks)

            result = cv2.bitwise_and(cv_image, cv_image, mask=mask)

            cv2.imshow("Original", cv_image)
            cv2.imshow("Filtered", result)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Ошибка обработки изображения: {e}")

if __name__ == '__main__':
    try:
        ColorFilterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()

