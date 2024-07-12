#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageRotatorNode:
    def __init__(self, input_image_topic, output_image_topic, rot90_n):
        """
        Args:
            input_image_topic (str) : Image topic to be rotated
            output_image_topic (str) : Output rotate Image topic to be published
            rot90_n (int) : Number of 90 deg rotations on the input image
            
        """
        rospy.init_node('image_rotator_node')
        self.bridge = CvBridge()
        self.rot90_n = rot90_n

        # Create subscribers and publishers
        self.image_sub = rospy.Subscriber(input_image_topic, Image, self.image_callback)
        self.image_pub = rospy.Publisher(output_image_topic, Image, queue_size=10)

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Rotate the image
        rotated_image = np.rot90(cv_image, self.rot90_n)

        # Convert OpenCV image back to ROS Image message
        rotated_msg = self.bridge.cv2_to_imgmsg(rotated_image, encoding='bgr8')

        # Publish the rotated image
        self.image_pub.publish(rotated_msg)

if __name__ == '__main__':
    node = ImageRotatorNode(input_image_topic='/camera/color/image_raw',
                            output_image_topic='/camera/color/image_raw_rotated',
                            rot90_n=3)
    rospy.spin()
