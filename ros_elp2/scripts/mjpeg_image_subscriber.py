#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class MJPEGImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=1)
        rospy.loginfo("Подписка на /usb_cam/image_raw запущена")

    def image_callback(self, msg):
        try:
            # Преобразуем ROS-сообщение в OpenCV-изображение
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Показываем изображение
            cv2.imshow("MJPEG Image", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Ошибка при обработке изображения: {e}")

if __name__ == "__main__":
    rospy.init_node("mjpeg_image_subscriber", anonymous=False)
    MJPEGImageSubscriber()
    rospy.spin()
    cv2.destroyAllWindows()

