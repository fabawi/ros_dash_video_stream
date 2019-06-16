#!/usr/bin/env python

from fps import FPS
from video_generators import VideoSource
import cv2


class VideoStreamer(object):
    def __init__(self):
        self.fps = FPS().start()

    def get_fps(self):
        self.fps.stop()
        fps = self.fps.fps()
        self.fps.start()
        return fps

    def get_frame(self):
        raise NotImplementedError("Choose a video streamer from the available ones "
                                  "e.g., CV2VideoStreamer or ROSVideoStreamer")


class ROSVideoStreamer(VideoStreamer):
    def __init__(self):
        super(ROSVideoStreamer, self).__init__()
        self.image = None

    def get_frame(self):
        if self.image is not None:
            ret, jpeg = cv2.imencode('.jpg', self.image)
            jpeg_bytes = jpeg.tobytes()
            self.fps.update()
            return jpeg_bytes


class CV2VideoStreamer(VideoStreamer):
    def __init__(self, resource=None):
        super(CV2VideoStreamer, self).__init__()
        if resource is None:
            print "You must give an argument to open a video stream."
            print "  It can be a number as video device,  e.g.: 0 would be /dev/video0"
            print "  It can be a url of a stream,         e.g.: rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mov"
            print "  It can be a video file,              e.g.: samples/moon.avi"
            print "  It can be a class generating images, e.g.: TimeStampVideo"
            exit(0)

        # If given a number interpret it as a video device
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
        self.fps = FPS().start()

    def __del__(self):
        self.cap.release()

    def get_frame(self):
        success, image = self.cap.read()
        # TODO (fabawi): resizing the image takes some time. make it multi-threaded
        # image = imutils.resize(image, width=VID_WIDTH)

        ret, jpeg = cv2.imencode('.jpg', image)
        jpeg_bytes = jpeg.tobytes()
        self.fps.update()
        return jpeg_bytes
