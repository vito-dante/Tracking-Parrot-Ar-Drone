import cv2
from .figure import FigureStatus
from collections import deque
import numpy as np

# Color space
# The values below were obtained using ImageJ (image-> adjust-> threshold)
MIN_H = 30
MAX_H = 100
MIN_S = 15
MAX_S = 176
MIN_V = 47
MAX_V = 175
# size draw = 64
queque = 20
pts = deque(maxlen=queque)

class Ball(FigureStatus):

    def __init__(self):
        super(Ball, self).__init__()
        #TODO pick up color from window
        self.greenLower = (MIN_H, MIN_S, MIN_V)
        self.greenUpper = (MAX_H, MAX_S, MAX_V)
        self.mask = None
        self.frame = None

    def segmenta_objetos_color_roi(self):
        copy_image = self.frame.copy()
        cv2.GaussianBlur(copy_image, (11, 11), 0)
        hsv = cv2.cvtColor(copy_image, cv2.COLOR_BGR2HSV)

        self.mask = cv2.inRange(hsv, self.greenLower, self.greenUpper)
        self.mask = cv2.erode(self.mask, None, iterations=2)
        self.mask = cv2.dilate(self.mask, None, iterations=2)

    # analyzes the image related components
    def detecta_objeto_mas_redondo(self):

        cnts = cv2.findContours(self.mask,
                                cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            (_, radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            m00 = M["m00"]
            x = int(M['m10'] / m00)
            y = int(M['m01'] / m00)
            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(self.frame, (int(x), int(y)), int(radius),
                        (0, 255, 255), 2)
                cv2.circle(self.frame, center, 5, (0, 255, 0), -1)
                # update the points queue
                pts.appendleft(center)
                # loop over the set of tracked points
                for i in xrange(1, len(pts)):
                    # otherwise, compute the thickness of the line and
                    # draw the connecting lines
                    thickness = int(np.sqrt(queque / float(i + 1)) * 2.5)
                    cv2.line(self.frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
                    #TODO add size to actualizar_situacion
                self.actualizar_situacion(x, y, 5)
        else:
            self.actualizar_situacion(-1, -1, -1)

    def set_color_hsv(self,min,max):
        self.greenLower = min
        self.greenUpper = max

    def find_object(self, image):
        self.frame = image
        self.segmenta_objetos_color_roi()
        self.detecta_objeto_mas_redondo()
        return self.frame
