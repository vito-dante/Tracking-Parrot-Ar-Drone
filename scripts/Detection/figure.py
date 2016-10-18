from .objectStatus import ObjectStatus

# Para convertir el tipo de imgaen de ROS a OpenCV
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
bridge = CvBridge()

from settings_objects import MAX_X
from settings_objects import MAX_Y

# Porcentaje en relacion al tamano de imagen que debe ser considerado como margen
PORC_MARGEN = 0.15
# 15% considerado como margen
MARGEN_IZQ = MAX_X * PORC_MARGEN
MARGEN_DER = MAX_X * (1 - PORC_MARGEN)
MARGEN_ABAJO = MAX_Y * PORC_MARGEN
MARGEN_ARRIBA = MAX_Y * (1 - PORC_MARGEN)

ObjectStatus = ObjectStatus()

class FigureStatus(object):
    """docstring for ClassName"""

    def __init__(self):
        super(FigureStatus, self).__init__()
        self.tamano_antes = -1  # tamano de la esfera en el fotograma anterior
        self.estado_antes = -1  # Guarda el estado anterior para mejorar la deteccion al desaparecer
        self.xc = -1  # X el centro del fotograma actual
        self.yc = -1  # Y el centro del fotograma actual
        self.tamano = -1  # tamano de la esfera en el fotograma actual
        self.estado = ObjectStatus.disapared  # comienza el estado Desaparecio (o sea la esfera no aparece en la imagen )
        self.contadorDesaparecio = 0  # a veces la esfera desaparece por un problema de deteccion, vamos a guardar varias imagenes de desaparecido en sequencia para confiramar que en verdad ha desaparecido
        self.cv_image = None  # va guardar una imagen en formato compatible con OpenCV
        self.contaImagens = 0  # Contador con el fin de nombrar a los archivos que contienen la secuencia de fotogramas capturados

    def findObject(self,image):
        pass

    def ToOpenCV(self,ros_image):
        try:
            self.cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print (e)
            raise Exception("Falla en conversion de imagen para OpenCV")

    def actualizarSituacion(self, pxc, pyc, ptamano):

        # the size should be 20 porcent from before size
        # for to know if the object is in the same place
        porc = self.tamano_antes*0.05
        ptamano_inf = self.tamano_antes - porc
        ptamano_sup = self.tamano_antes + porc

        self.estado_antes = self.estado
        #TODO samePlace almost imporsible set up
        if  self.estado_antes == self.estado and \
                self.estado != ObjectStatus.disapared and \
                ptamano > ptamano_inf and ptamano < ptamano_sup:
            self.estado = ObjectStatus.samePlace

        # si el tamano de antes era -1 es porque la esfera no aparecia en la imagen
        # si ahora es diferente de -1 es poque la esfera aparecio
        if self.tamano_antes == -1 and ptamano != -1:
            self.estado = ObjectStatus.appeared

        # tamano -1 indica que no tiene la esfera en la imagen
        elif ptamano == -1:
            self.estado = ObjectStatus.disapared

        # si esta por lo menos dos fotogramas con la esfera en la imagen comienza a identificar, si estan en los margens o no
        elif self.tamano_antes != -1 and ptamano != -1:

            # voy a identificar aqui si el centro de la esfera golpeo los margenes de la imagen
            # los margenes de la izquierda, derecha, arriba y abajo seran utilizado para mover el drone en direccion que corresponda hasta que la esfera vuelva a quedar en el centro de la imagen

            # una imagen es invertida por eso la derecha gira a la izquierda
            front = self.tamano_antes + (self.tamano_antes*0.1)
            behind = self.tamano_antes - (self.tamano_antes*0.1)

            if pxc > MARGEN_DER:
                self.estado = ObjectStatus.movedLeft
            elif pxc < MARGEN_IZQ:
                self.estado = ObjectStatus.movedRight
            elif pyc > MARGEN_ARRIBA:
                self.estado = ObjectStatus.movedDown
            elif pyc < MARGEN_ABAJO:
                self.estado = ObjectStatus.movedUp
            elif ptamano > front:
                self.estado =  ObjectStatus.movedFront
            elif ptamano < behind:
                self.estado =  ObjectStatus.movedBack

        # actualiza las variables del objeto con los valores de dos parametros
        self.tamano_antes = self.tamano
        self.xc = pxc
        self.yc = pyc
        self.tamano = ptamano