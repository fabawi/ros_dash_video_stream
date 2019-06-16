#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# instantiate CvBridge
bridge = CvBridge()


def image_callback(msg):
    try:
        # convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Stream", cv2_img)
        key = cv2.waitKey(1)
        # print "key pressed: " + str(key)
        if key == 27 or key == 1048603:
            exit(0)
    except CvBridgeError, e:
        print(e)
    # else:
    #     # save your OpenCV2 image as a jpeg
    #     cv2.imwrite('camera_image.jpeg', cv2_img)


def main():
    rospy.init_node('image_listener')
    # define your image topic
    image_topic = "/prototype_image"
    # set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # spin until ctrl + c
    rospy.spin()


if __name__ == '__main__':
    main()