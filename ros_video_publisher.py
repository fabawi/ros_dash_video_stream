#!/usr/bin/env python

import rospy
import cv2
import sys
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import video_generators

MODE = 'CV2_TIMESTAMP_SYNTHETIC' # 'CV2_CAM', 'CV2_WEB_VIDEO', 'CV2_LOCAL_VIDEO', 'CV2_NOISE_SYNTHETIC', 'CV2_TIMESTAMP_SYNTHETIC'


class ROSVideoPublisher(object):
    def __init__(self, resource=None, topic='prototype_image', visualize=False):
        self.visualize = visualize

        self.pub = rospy.Publisher(topic, Image, queue_size=100)
        # if pub != None:
        # print "pub created"
        rospy.init_node('image_publisher', anonymous=True)
        # rate = rospy.Rate(10) # not sure if needed
        self.bridge = CvBridge()

        if resource is None:
            print "You must give an argument to open a video stream."
            print "  It can be a number as video device,  e.g.: 0 would be /dev/video0"
            print "  It can be a url of a stream,         e.g.: rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mov"
            print "  It can be a video file,              e.g.: robotvideo.mkv"
            print "  It can be a class generating images, e.g.: TimeStampVideo"
            exit(0)

        # given a number interpret it as a video device
        if isinstance(resource, int) or len(resource) < 3:
            self.resource_name = "/dev/video" + str(resource)
            resource = int(resource)
            self.vidfile = False
        else:
            self.resource_name = str(resource)
            self.vidfile = True
        print "Trying to open resource: " + self.resource_name

        if isinstance(resource, VideoSource):
            self.cap = resource
        else:
            self.cap = cv2.VideoCapture(resource)

        if not self.cap.isOpened():
            print "Error opening resource: " + str(resource)
            exit(0)

    def publish(self):
        print "Correctly opened resource, starting to show feed."
        rval, frame = self.cap.read()
        while rval:
            if self.visualize:
                cv2.imshow("Stream: " + self.resource_name, frame)
            rval, frame = self.cap.read()

            # ROS image conversion
            if self.vidfile and frame is not None:
                frame = np.uint8(frame)
                image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.pub.publish(image_message)

            if self.visualize:
                key = cv2.waitKey(1)
                # print "key pressed: " + str(key)
                if key == 27 or key == 1048603:
                    break
        cv2.destroyWindow("preview")


if __name__ == '__main__':

    try:
        if MODE == 'CV2_CAM':
            source = 0
        elif MODE == 'CV2_WEB_VIDEO':
            source = 'rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mov'
        elif MODE == 'CV2_LOCAL_VIDEO':
            source = 'samples/moon.avi'
        elif MODE == 'CV2_NOISE_SYNTHETIC':
            source = video_generators.VideoSource(image_size=(1080, 1920))
        elif MODE == 'CV2_TIMESTAMP_SYNTHETIC':
            source = video_generators.TimeStampVideo(image_size=(1080, 1920), time_fmt ='%H:%M:%S.%f')
        else:
            source =  video_generators.TimeStampVideo(image_size=(1080, 1920), time_fmt ='%H:%M:%S.%f')

        rp = ROSVideoPublisher(source, visualize=True)
        rp.publish()
    except rospy.ROSInterruptException:
        pass