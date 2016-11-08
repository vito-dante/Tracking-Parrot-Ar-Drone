from .object_status import ObjectStatus
# SCREEN SETTING, this size is equal than size image asked for ros image
MAX_X = 640
MAX_Y = 360
# Percentage considered for margin, in relation to the image
PORC_MARGEN = 0.20
MARGEN_IZQ = MAX_X * PORC_MARGEN
MARGEN_DER = MAX_X * (1 - PORC_MARGEN)
MARGEN_ABAJO = MAX_Y * PORC_MARGEN
MARGEN_ARRIBA = MAX_Y * (1 - PORC_MARGEN)

ObjectStatus = ObjectStatus()

class FigureStatus(object):
    '''
    this class represent methods:
     - find_object --> try detect object
     - update_position_object --> set position object current

    '''
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

    def find_object(self, image):
        pass

    def update_position_object(self, pxc, pyc, ptamano):

        self.estado_antes = self.estado

        # ptamano -1 It indicates no object in the image
        if ptamano == -1:
            self.estado = ObjectStatus.disapared

        # If the size of before was -1 it is because the object
        #  did not appear in the image
        # if it's different now -1 it is because the object appeared
        else:
            if self.tamano_antes == -1:
                self.estado = ObjectStatus.appeared

            else:
                # like "Object Status.same_place" 10 percent is used
                # to see if the object became larger or perqueno
                front = self.tamano_antes + (self.tamano_antes*0.1)
                behind = self.tamano_antes - (self.tamano_antes*0.1)


                # image is inverted
                if ptamano > front:
                    self.estado =  ObjectStatus.moved_back
                    pass
                elif ptamano < behind:
                    self.estado =  ObjectStatus.moved_front
                    pass
                elif pxc > MARGEN_DER:
                    self.estado = ObjectStatus.moved_left
                elif pxc < MARGEN_IZQ:
                    self.estado = ObjectStatus.moved_right
                elif pyc > MARGEN_ARRIBA:
                    self.estado = ObjectStatus.moved_down
                    pass
                elif pyc < MARGEN_ABAJO:
                    self.estado = ObjectStatus.moved_up
                    pass
                else:
                    # it takes 5 percentage the size to see if the continuous drone
                    # in the same place.
                    # if the size of the object to be located
                    # passes or is less than 5 percent in the next state figure
                    # or object it has moved

                    porc = self.tamano_antes * 0.05
                    ptamano_inf = self.tamano_antes - porc
                    ptamano_sup = self.tamano_antes + porc
                    if self.estado_antes == self.estado and \
                                ptamano > ptamano_inf and \
                                ptamano < ptamano_sup:
                        self.estado = ObjectStatus.same_place

        # updated object variables
        self.tamano_antes = self.tamano
        self.xc = pxc
        self.yc = pyc
        self.tamano = ptamano
