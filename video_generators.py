#!/usr/bin/env python

import numpy as np
import cv2
from datetime import datetime


class VideoSource(object):
    """ The super class which video resources must inherit"""
    def __init__(self, image_size=(1080, 1920)):
        self.image_size = image_size
        self.__len = image_size[0] * image_size[1] * 3

    def isOpened(self):
        return True

    def read(self):
        rval = True
        frame = np.random.randint(0, 255,  (self.image_size[0],self.image_size[1], 3), np.uint8)
        return rval, frame

    def __len__(self):
        return self.__len

    def __str__(self):
        return 'Video Source'


class TimeStampVideo(VideoSource):
    """ This class generates an image with the current time displayed within a box"""
    def __init__(self, image_size=(1080, 1920), time_fmt ='%H:%M:%S.%f'):
        super(TimeStampVideo, self).__init__(image_size)
        self.time_fmt = time_fmt

        # the background image
        self.image =np.ones((image_size[0],image_size[1], 3), np.uint8)*np.array([0, 0, 255])

        # get the text once to estimate its size
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        text = datetime.now().strftime(time_fmt)
        textsize = cv2.getTextSize(text, self.font, 1, 2)[0]
        self.text_x_y =((self.image.shape[1] - textsize[0]) // 2, (self.image.shape[0] - textsize[1]) // 2, )

    def read(self):
        rval = True
        # self.image = np.random.randint(0, 255,  (self.image_size[0],self.image_size[1], 3), np.uint8)
        frame = np.ascontiguousarray(self.image, dtype=np.uint8)

        cv2.putText(frame, datetime.now().strftime(self.time_fmt), self.text_x_y,
                    self.font, 1, (255, 255, 255), 3, cv2.LINE_AA)
        return rval, frame

    def __str__(self):
        return 'Timestamp Video Generator'

