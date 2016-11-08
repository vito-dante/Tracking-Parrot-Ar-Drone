
import cv2
import rospy
from .figure import FigureStatus

import os
cascPath = os.path.dirname(os.path.realpath(__file__))+'/haarcascade_frontalface_alt_tree.xml'
faceCascade = cv2.CascadeClassifier(cascPath)

class Face(FigureStatus):
    '''
        this class detect Face --> method haarcascade
        input --> image BGR
        output --> image BGR + draw detection + position object
                || only image in case no detection
    '''

    def __init__(self):
        super(Face, self).__init__()

    def find_object(self, image):
        hsv = cv2.cvtColor(image.copy(), cv2.COLOR_BGR2GRAY)
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
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            self.update_position_object(x, y, w)
            rospy.loginfo("x:{} y:{}".format(x,y))
        else:
            self.update_position_object(-1, -1, -1)
        return image
