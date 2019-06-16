#! /usr/bin/python

from threading import Thread
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

from video_streamers import ROSVideoStreamer


class ROSVideoListener(object):
    def __init__(self, topic='/prototype_image', visualize=False):
        self.topic = topic
        self.visualize = visualize
        self.bridge = CvBridge()

    def _streamer_hook(self, streamer):
        def image_callback(msg):
            try:
                streamer.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                if self.visualize:
                    cv2.imshow("Stream", streamer.image)
                    key = cv2.waitKey(1)
                    # print "key pressed: " + str(key)
                    if key == 27 or key == 1048603:
                        exit(0)
            except CvBridgeError, e:
                print(e)
            # else:
            #     # save your OpenCV2 image as a jpeg
            #     cv2.imwrite('camera_image.jpeg', cv2_img)

        return image_callback

    def subscribe(self, streamer):
        rospy.init_node('image_listener')
        image_topic = self.topic
        rospy.Subscriber(image_topic, Image, self._streamer_hook(streamer=streamer))
        rospy.spin()


def streamer_thread(streamer):
    while True:
        streamer.get_frame()


if __name__ == '__main__':
    streamer = ROSVideoStreamer()
    t1 = Thread(target=streamer_thread, args={streamer})
    t1.start()

    rl = ROSVideoListener(visualize=False)
    rl.subscribe(streamer)