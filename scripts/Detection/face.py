
import cv2
from .figure import FigureStatus

# importa la ruta del haarcascade_frontalface_alt_tree
import os
cascPath = os.path.dirname(os.path.realpath(__file__))+'/haarcascade_frontalface_alt_tree.xml'
faceCascade = cv2.CascadeClassifier(cascPath)

class Face(FigureStatus):
    def __init__(self):
        super(Face, self).__init__()

    def findObject(self, image):
        self.ToOpenCV(image)  # Covierte de ROS para OpenCV
        self.hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        self.faces = faceCascade.detectMultiScale(
            self.hsv,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.cv.CV_HAAR_SCALE_IMAGE
        )
        if len(self.faces):
            for (x, y, w, h) in self.faces:
                cv2.rectangle(self.cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            self.actualizarSituacion(x, y, 5)
            faceDraw = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
            return faceDraw
        else:
            # usa -1 para indicar que no encontro ninguna esfera en este fotograma
            self.actualizarSituacion(-1, -1, -1)
            myimage =cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
            return myimage
        # cv2.imshow('Deteccion del objeto', self.cv_image)
