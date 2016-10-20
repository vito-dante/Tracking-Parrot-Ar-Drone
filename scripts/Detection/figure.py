from .objectStatus import ObjectStatus
from settings_objects import MAX_X
from settings_objects import MAX_Y

# Percentage considered for margin, in relation to the image
PORC_MARGEN = 0.15
MARGEN_IZQ = MAX_X * PORC_MARGEN
MARGEN_DER = MAX_X * (1 - PORC_MARGEN)
MARGEN_ABAJO = MAX_Y * PORC_MARGEN
MARGEN_ARRIBA = MAX_Y * (1 - PORC_MARGEN)

ObjectStatus = ObjectStatus()

class FigureStatus(object):
    """docstring for ClassName"""
    def __init__(self):
        super(FigureStatus, self).__init__()
        # size of the object in the previous frame
        self.tamano_antes = -1
        # Saves the previous state
        self.estado_antes = -1
        #the center of the object at the location x
        self.xc = -1
        # the center of the object at the location x
        self.yc = -1
        #current size of the object
        self.tamano = -1
        #initial state of the object
        self.estado = ObjectStatus.disapared

    def findObject(self,image):
        pass

    def actualizarSituacion(self, pxc, pyc, ptamano):

        #it takes 5 pociento the size to see if the continuous drone
        # in the same place.
        # if the size of the object to be located
        # passes or is less than 5 percent in the next state figure
        # or object it has moved
        porc = self.tamano_antes*0.05
        ptamano_inf = self.tamano_antes - porc
        ptamano_sup = self.tamano_antes + porc

        self.estado_antes = self.estado

        if  self.estado_antes == self.estado and \
                self.estado != ObjectStatus.disapared and \
                ptamano > ptamano_inf and ptamano < ptamano_sup:
            self.estado = ObjectStatus.samePlace

        # If the size of before was -1 it is because the object
        #  did not appear in the image
        # if it's different now -1 it is because the object appeared
        if self.tamano_antes == -1 and ptamano != -1:
            self.estado = ObjectStatus.appeared

        # ptamano -1 It indicates no object in the image
        elif ptamano == -1:
            self.estado = ObjectStatus.disapared

        elif self.tamano_antes != -1 and ptamano != -1:

            # like "Object Status.samePlace" 10 percent is used
            # to see if the object became larger or perqueno
            front = self.tamano_antes + (self.tamano_antes*0.1)
            behind = self.tamano_antes - (self.tamano_antes*0.1)

            # image is inverted
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

        # updated object variables
        self.tamano_antes = self.tamano
        self.xc = pxc
        self.yc = pyc
        self.tamano = ptamano