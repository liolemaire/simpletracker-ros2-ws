# Original work Copyright (c) 2022 Sky360
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

import cv2
from simple_tracker_shared.utils import kp_to_bbox
import pysky360 as sky360

class BlobDetector():

    # Static factory select method to determine what masking implementation to use
    @staticmethod
    def Select(settings):

        detector_type = settings['blob_detector_type']

        if detector_type == 'simple':
            return BlobDetector.Simple(settings)

        if detector_type == 'sky360':
            return BlobDetector.Sky360(settings)

    @staticmethod
    def Simple(settings):
        return SimpleBlobDetector(settings)

    @staticmethod
    def Sky360(settings):
        return Sky360BlobDetector(settings)


    def __init__(self, settings):
        self.settings = settings
        self.sensitivity = settings['tracker_detection_sensitivity']

    def detect(self, frame):
        pass

class SimpleBlobDetector(BlobDetector):

    def __init__(self, settings):
        super().__init__(settings)

        params = cv2.SimpleBlobDetector_Params()

        params.minRepeatability = 2
        params.minDistBetweenBlobs = 32
        params.minThreshold = 3
        params.filterByArea = 1
        params.filterByColor = 0
        # params.blobColor=255

        if self.sensitivity == 1:  # Detects small, medium and large objects
            params.minArea = 3
        elif self.sensitivity == 2:  # Detects medium and large objects
            params.minArea = 5
        elif self.sensitivity == 3:  # Detects large objects
            params.minArea = 25
        else:
            raise Exception(
                f"Unknown sensitivity option ({self.sensitivity}). 1, 2 and 3 is supported not {self.sensitivity}.")

        self.blob_detector = cv2.SimpleBlobDetector_create(params)

    def detect(self, frame):
        super().detect(frame)

        keypoints = self.blob_detector.detect(frame)

        bboxes = [kp_to_bbox(x) for x in keypoints]
        return bboxes

class Sky360BlobDetector(BlobDetector):

    def __init__(self, settings):
        super().__init__(settings)

        self.blob_detector = sky360.ConnectedBlobDetection()

        self.blob_detector.setMinDistance(32)

        self.blob_detector.setSizeThreshold(2)

        if self.sensitivity == 1:  # Detects small, medium and large objects
            self.blob_detector.setAreaThreshold(2)
        elif self.sensitivity == 2:  # Detects medium and large objects
            self.blob_detector.setAreaThreshold(10)
        elif self.sensitivity == 3:  # Detects large objects
            self.blob_detector.setAreaThreshold(25)
        else:
            raise Exception(
                f"Unknown sensitivity option ({self.sensitivity}). 1, 2 and 3 is supported not {self.sensitivity}.")

    def detect(self, frame):
        super().detect(frame)

        bboxes = self.blob_detector.detectBB(frame)
        return bboxes