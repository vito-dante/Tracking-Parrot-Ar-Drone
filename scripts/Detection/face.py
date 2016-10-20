
import cv2
from .figure import FigureStatus

import os
cascPath = os.path.dirname(os.path.realpath(__file__))+'/haarcascade_frontalface_alt_tree.xml'
faceCascade = cv2.CascadeClassifier(cascPath)

class Face(FigureStatus):
    def __init__(self):
        super(Face, self).__init__()

    def findObject(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        faces = faceCascade.detectMultiScale(
            hsv,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.cv.CV_HAAR_SCALE_IMAGE
        )
        if len(faces) == 1:
            (x, y, w, h) = faces[0]
            # for (x, y, w, h) in faces:
            cv2.rectangle(self.cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            self.actualizarSituacion(x, y, w)
            cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
            return self.cv_image
        else:
            self.actualizarSituacion(-1, -1, -1)
            cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
            return self.cv_image
