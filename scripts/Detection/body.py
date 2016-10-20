from __future__ import print_function
from .figure import FigureStatus
from imutils.object_detection import non_max_suppression
import numpy as np
import imutils
import cv2

hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

cap = cv2.VideoCapture(0)


class Body(FigureStatus):
    # detect people in the image
    def __init__(self):
        super(Body, self).__init__()

    def findObject(self, image):
        image = imutils.resize(image, width=min(360, image.shape[1]))
        (rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),
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
            self.actualizarSituacion(x,y,size)
        else:
            self.actualizarSituacion(-1, -1, -1)
        return image
