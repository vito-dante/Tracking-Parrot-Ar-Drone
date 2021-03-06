from __future__ import print_function
from .figure import FigureStatus
from imutils.object_detection import non_max_suppression
import numpy as np
import cv2

hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

class Body(FigureStatus):
    '''
    this class detect Body --> method haarcascade + descriptor HOG
    input --> image BGR
    output --> image BGR + draw detection + position object
            || only image in case no detection
    '''

    def __init__(self, sizeA=30,sizeB=40):
        super(Body, self).__init__(sizeA, sizeB)

    def find_object(self, image):
        (rects, weights) = hog.detectMultiScale(image.copy(), winStride=(4, 4),
                                                padding=(8, 8), scale=1.05)
        rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
        pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
        if len(pick) == 1:
            (xA, yA, xB, yB) = pick[0]
            # for (xA, yA, xB, yB) in pick:
                # draw the final bounding boxes
            cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)
            x = ((xB - xA) /2.0) + xA
            y = ((yB - yA) / 2.0) + yA
            size = xB - xA
            self.update_position_object(x, y, size)
        else:
            self.update_position_object(-1, -1, -1)
        return image
