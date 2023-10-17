import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge


class CamPublisher(Node):

    def __init__(self):
        super().__init__('CamPublisher')
        self.pub_video = self.create_publisher(Image, '/mobi/webcam', 10)

        self.get_logger().info("Started cam Publisher")

        self.cam = cv2.VideoCapture(0)
        # 1280x720
        self.cam.set(3,1280) # width
        self.cam.set(4,720) # hight
        # 1920x1080
        # self.cam.set(3,1920) # width
        # self.cam.set(4,1080) # hight
        
        timer_period = 0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        meta, frame = self.cam.read()
        msg_frame = CvBridge().cv2_to_imgmsg(frame, "bgr8")
        self.pub_video.publish(msg_frame)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CamPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()