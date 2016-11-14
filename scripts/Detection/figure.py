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
    # status of the object for the window
    MessageSituacion = {
        ObjectStatus.appeared: 'Aparecio',
        ObjectStatus.disapared: 'Desaparecio',
        ObjectStatus.moved_left: 'Izquierda',
        ObjectStatus.moved_right: 'Derecha',
        ObjectStatus.moved_up: 'Arriba',
        ObjectStatus.moved_down: 'Abajo',
        ObjectStatus.moved_front: 'Se movio hacia adelante',
        ObjectStatus.moved_back: 'Se movio hacia atras',
        ObjectStatus.same_place: 'Mismo lugar'
    }

    def __init__(self,sizeA,sizeB):
        super(FigureStatus, self).__init__()
        # size of the object in the previous frame
        self.tamano_antes = -1
        # Saves the previous state
        self.estado_antes = -1
        #current size of the object
        self.tamano = -1
        #initial state of the object
        self.estado = ObjectStatus.disapared
        # size for each figure
        self.size_wished_inf = sizeA
        self.size_wished_sup = sizeB


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
                # to see if the object became larger or small

                if pxc > MARGEN_DER:
                    self.estado = ObjectStatus.moved_right
                elif pxc < MARGEN_IZQ:
                    self.estado = ObjectStatus.moved_left
                elif pyc > MARGEN_ARRIBA:
                    self.estado = ObjectStatus.moved_down
                elif pyc < MARGEN_ABAJO:
                    self.estado = ObjectStatus.moved_up
                elif self.estado_antes == self.estado and \
                                ptamano > self.size_wished_inf and \
                                ptamano < self.size_wished_sup:
                    self.estado = ObjectStatus.same_place
                elif ptamano > self.size_wished_sup:
                    self.estado =  ObjectStatus.moved_front
                    pass
                elif ptamano < self.size_wished_inf:
                    self.estado =  ObjectStatus.moved_back

        # updated object variables
        self.tamano_antes = self.tamano
        self.tamano = ptamano

    def status_msg(self):
        return self.MessageSituacion[self.estado]