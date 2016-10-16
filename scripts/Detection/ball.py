
import numpy as np
import cv2
from .figure import FigureStatus

# ===================== SETTINGS BALL ==============================
# Define la funcion que convierte una imagen para el formato de Opencv2,
# trabaja usando la clase CvBridge del paquete cv_bridge
# ==================== ESPACIOS DE COLORES =================
# Los valores de abajo fueron obtenidos utilizando imagenJ (image->adjust->threshold)
# en una imagen capturada por el drone, guarda un archivo despues
# "binarizarla" en imageJ
MIN_H = 22  # Valor minimo de H al ser usado la limitarizacion en HSV
MAX_H = 71  # maximo de H
MIN_S = 43  # minimo de S
MAX_S = 204  # maximo de S
MIN_V = 97  # minimo de V
MAX_V = 203  # maximo de V
# ==================== END ESPACIOS DE COLORES =================

# ==================== OBJETO =================
MIN_AREA = 100   # area minima para considerar la esfera (en pixeles)
MAX_AREA = 13000  # are maxima en pixeles
MIN_CIRCULARIDAD = 0.4  # variable que define que tan circular es el objeto
# que se va a detectar si pongo un valor muy alto sera precisamente un circulo
# y no detectara otra cosa que no sea parecida a ella, estoy colocando un
# valor bajo para que detecte objetos que no son precisamentes circulares o esferico
# va ser usado para identificar movimientos para el frente y para atras
# con base en el cambio en porcentaje del tamano de la esfera (en la
# perspectiva de la camara del drone)
PORCENTAJE_CAMBIO_AREA = 0.5
# ==================== END OBJETO =================
# usado para limpiar blobs/contornos pequenos (nucleo para aplicaciones de
# convulsion, de dilatacion en este caso )
kernelSize = 3
kernel = np.ones((kernelSize, kernelSize), np.uint8)
# ===================== END SETTINGS BALL ==============================


class Ball(FigureStatus):

    def __init__(self):
        super(Ball, self).__init__()
        self.greenLower = (29, 86, 6)
        self.greenUpper = (64, 255, 255)
        self.mask = None
        self.frame = None
        self.copyFrame = None
    # utilizando la segmentacion basada en binarizacion en el espacio
    #  de color HSV, segmenta las imagenes con pixeles segun al color
    # Los umbrales son fijos y definidos por la variable MIN H, MIN S ... en
    # el principio del codigo
    def segmentaObjetosColorRoi(self):
        self.copyFrame = self.frame
        blurred = cv2.GaussianBlur(self.copyFrame, (11, 11), 0)
        hsv = cv2.cvtColor(self.copyFrame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.greenLower, self.greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        self.mask = cv2.dilate(mask, None, iterations=2)

    # analiza los componentes conexos de la imagen binarizada y filtra solo
    # los que tengan area mayor o menor que los umbrales declarados a
    # principios de este codigo y que tambien tenga una circularidad mayor que
    # un valor predeterminado (tambien definido a principios del codigo)
    def detectaObjetoMasRedondo(self):

        cnts = cv2.findContours(self.mask.copy(),
                                cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
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
                self.actualizarSituacion(x, y, 5)
            cv2.circle(self.frame, center, 5, (0, 0, 255), -1)
            return self.frame
        else:
            self.actualizarSituacion(-1, -1, -1)

    # Va a detectar la esfera dentro de la imagen (o su ausencia) y
    # actualizara la situacion no hace mucho, ademas de llamar a las funciones
    # que realmente lo hacen
    def findObject(self, image):
        # self.ToOpenCV(image)
        # image type RGB
        self.frame = image
        self.segmentaObjetosColorRoi()
        self.detectaObjetoMasRedondo()
        return self.frame
