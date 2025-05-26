#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class YUYVtoRGBConverter:
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=1)
        self.pub = rospy.Publisher("/usb_cam/image_color", Image, queue_size=1)
        rospy.loginfo("Запущен узел преобразования YUYV → RGB")

    def image_callback(self, msg):
        try:
            height = msg.height
            width = msg.width
            # Преобразуем данные в numpy-массив
            yuyv = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 2))
            # OpenCV требует (height, width*2) для YUYV
            yuyv_bgr = cv2.cvtColor(yuyv, cv2.COLOR_YUV2RGB_YUY2)
            # Обратно в ROS-сообщение
            rgb_msg = self.bridge.cv2_to_imgmsg(yuyv_bgr, encoding="rgb8")
            rgb_msg.header = msg.header
            self.pub.publish(rgb_msg)
        except Exception as e:
            rospy.logerr(f"Ошибка при обработке изображения: {e}")

if __name__ == "__main__":
    rospy.init_node("yuyv_to_rgb_converter", anonymous=False)
    YUYVtoRGBConverter()
    rospy.spin()

