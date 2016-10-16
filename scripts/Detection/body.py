import cv2

# importa la ruta del haarcascade_frontalface_alt_tree
import os
#TODO reemplace face haarcascad by body haarcascad
cascPath = os.path.dirname(os.path.realpath(__file__))+'/haarcascade_frontalface_alt_tree.xml'
faceCascade = cv2.CascadeClassifier(cascPath)