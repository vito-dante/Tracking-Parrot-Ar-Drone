#! /usr/bin/python

# Biblioteca ROS para python
from threading import Lock

# connector for ROS
import rospy

import time

import sys

# Librerary for GUI (window)
from PySide import QtCore, QtGui

# status of the object(FACE,BALL,BODY)
from Detection.objectStatus import ObjectStatus

# status of the drone
from Drone.drone_status import DroneStatus

# importa el controlador del drone
from Drone.drone_controller import DroneController

#Objects for detection
from Detection.ball import Ball
from Detection.face import Face
from Detection.qrcode import qrCode
from Detection.body import Body


# settings
from settings import CONNECTION_CHECK_PERIOD
from settings import GUI_UPDATE_PERIOD
from settings import PORCENTAJE_VELOCIDAD


# mensajes que vienen de ROS
from sensor_msgs.msg import Image  # Para recibir los fotogramas del video

from std_srvs.srv import Empty #Mensaje de parametro para el cambio de camara

# mensaje de ardrone_autonomy
from ardrone_autonomy.msg import Navdata # Para recibir informacion sobre el estado del drone

from ardrone_autonomy.srv import FlightAnim,LedAnim


class SeguirObjeto(QtGui.QMainWindow):
    # definicion de los tipos de objectos
    # ObjectTaarget: sera el objeto principal para el seguimiento
    # secondaryTarget: sera utilizado para que drone
    # reaccione con un movimiento(FLIP)
    # objectTarget = Face()
    objectTarget = Body()
    # secondaryTarget = Ball()
    secondaryTarget = qrCode()

    DroneStatus = DroneStatus()
    ObjectStatus = ObjectStatus()
    # status of the drone for the windows
    StatusMessages = {
        DroneStatus.emergency: 'Emergencia',
        DroneStatus.inited: 'Inciado',
        DroneStatus.landed: 'Aterrizado',
        DroneStatus.flying: 'Volando',
        DroneStatus.hovering: 'Parado en el aire',
        DroneStatus.test: 'Es un Test ?',
        DroneStatus.takingoff: 'Despegando',
        DroneStatus.gotohover: 'entrando en el modo parado en el aire ',
        DroneStatus.landing: 'Aterrizando',
        DroneStatus.looping: 'Realizando un Loop ?'
    }
    DisconnectedMessage = 'Desconectado'
    UnknownMessage = 'Estado Desconocido'

    # status of the object for the window
    MessageSituacion = {
        ObjectStatus.appeared: 'Aparecio',
        ObjectStatus.disapared: 'Desaparecio',
        ObjectStatus.movedLeft: 'MovioParaIzquierda',
        ObjectStatus.movedRight: 'MovioParaDerecha',
        ObjectStatus.movedUp: 'MovioParaArriba',
        ObjectStatus.movedDown: 'MovioParaAbajo',
        ObjectStatus.movedFront: 'MovioParaFrente',
        ObjectStatus.movedBack: 'MovioParaAtraz',
        ObjectStatus.samePlace: 'Parada'
    }

    def __init__(self):

        super(SeguirObjeto, self).__init__()

        # titulo de la ventana
        self.setWindowTitle('Vision del drone')
        # imageBox donde sera dibujada la ventana
        self.imageBox = QtGui.QLabel(self) #aqui entran los FPS
        self.setCentralWidget(self.imageBox)

        # se inscribe en el topico  de ardrone_autonomy que publica
        # informaciones sobre la situacion del drone
        self.subNavdata = rospy.Subscriber(
            '/ardrone/navdata', Navdata, self.ReceiveNavdata)

        # Se inscribe en el topico de ardrone_autonomy que publica imagenes
        # capturadas por la camara
        self.subVideo = rospy.Subscriber(
            '/ardrone/image_raw', Image, self.ReceiveImage)

        # variable que ira a guardar los fotogramas de la imagen caputrada por el drone
        self.image = None
        self.imageLock = Lock()   # guarda una cerradura para la parte del codigo
        #  que muestra la imagen en la ventana

        # mensaje de estado del drone para ser prensetada en la
        # interfaz grafica ( se modifica durante la ejecucion)
        self.statusMessage = ''

        # usado para saber si recibimos datos del drone desde la ultima vez que
        # el temporizador desaperecio
        self.communicationSinceTimer = False
        self.connected = False

        # Temporizador para verificar de tiempo en tiempo si
        # el drone todavia esta conectado via WI-FI
        self.connectionTimer = QtCore.QTimer(self)    # Crea un temporizador
        # Indica que la funcion que debe ser llamada cuando le de el tiempo
        self.connectionTimer.timeout.connect(self.ConnectionCallback)
        # Indica la frequencia cuando la funcion debe ser llamada
        self.connectionTimer.start(CONNECTION_CHECK_PERIOD)

        # temporizador para redisenar la ventana de interfaz en tiempo en tiempo
        self.redrawTimer = QtCore.QTimer(self)
        self.redrawTimer.timeout.connect(self.RedrawCallback)
        self.redrawTimer.start(GUI_UPDATE_PERIOD)



    # Funcion que sera llamada de tiempo en tiempo y que actualiza
    # una variable connect para indicar
    # si hubiera comunicacion con el drone, el portatil y el drone estan conectados.
    # la variable communicationSinceTimer es actualizada en otra parte del codigo
    def ConnectionCallback(self):
        self.connected = self.communicationSinceTimer
        self.communicationSinceTimer = False

    # Funcion que sera llamada de tiempo en tiempo
    # para redisenar la ventana con una imagen que ve el drone
    def RedrawCallback(self):
        if self.image is not None:
            # Por problemas de sincronizacion, el sistema solicita un bloqueo aqui
            # que la imagen sea actualizada en la ventana
            self.imageLock.acquire()
            try:
                # pide para el objeto se ha detectado, pasa el
                # fotograma recien capturado como parametro
                frame = self.objectTarget.findObject(self.image)
                #frame of type RGB
                frame = self.secondaryTarget.findObject(frame)

                image = QtGui.QImage(frame,
                                     frame.shape[1],
                                     frame.shape[0],
                                     QtGui.QImage.Format_RGB888)
                pix = QtGui.QPixmap.fromImage(image)

            finally:
                self.imageLock.release()

            # Muestra una imagen en la ventana de la interfaz grafica
            self.resize(image.width(), image.height())
            self.imageBox.setPixmap(pix)
            # llama una funcion que va mandar al drone moverse,
            # si el objeto (Ball) lo indica
            self.moverDrone()

        # actualiza un mensaje con la situacion del drone
        self.statusBar().showMessage(
            self.statusMessage if self.connected else self.DisconnectedMessage)

    # Funcion que es llamada cuando llega una nueva imagen
    def ReceiveImage(self, data):
        # Indica que hubo comunicacion (ya que el fotograma de la imagen fue recibido )
        self.communicationSinceTimer = True

        # Bloqua para evitar problemas con la
        # sincronizacion(ocurre por que la imagen es grande
        # y no da para parar y copiarla en medio del proceso)
        self.imageLock.acquire()
        try:
            self.image = data  # Guarda la imagen recibida a traves del parametro data
        finally:
            self.imageLock.release()

    def ReceiveNavdata(self, navdata):
        # Indica que hubo comunicacion (ya que llegaron datos sobre el drone)
        # por el puerto 5556
        self.communicationSinceTimer = True

        # actualiza la situacion del drone que aparece en la interfaz con el estado del drone
        msg = self.StatusMessages[
            navdata.state] if navdata.state in self.StatusMessages else self.UnknownMessage
        # Estado del objeto(Ball) a ser acrecentado en el mensaje
        # que aparece abajo en la imagen capturada
        msgTarget = self.MessageSituacion[self.objectTarget.estado]

        # Mas alla del estado del drone y el estado del objeto(Ball),
        # anade informacion sobre la carga de la bateria del drone
        self.statusMessage = '{} | Target: {} (Battery: {}%)'.format(
            msg, msgTarget, int(navdata.batteryPercent))  # mostrar la carga de la bateria

    # esta funcion es asociada siempre que un nuevo fotograma
    #  en la imagen es capturado y depende de la situacion del drone
    # y el objeto (Ball) decide (despegar, aterrizar, mover a la izquierda, ....)
    def moverDrone(self):
        # verifica si el objeto responsable por el control del drone ya fue inicializado
        if controller is not None:
            if self.secondaryTarget.estado == self.ObjectStatus.appeared:
                    self.flip_animation("/ardrone/setflightanimation", FlightAnim)
                    # self.changeCamera("/ardrone/togglecam", Empty)

            elif self.objectTarget.estado == self.ObjectStatus.disapared:
                    # controller.SendLand()
                    pass

            elif self.objectTarget.estado == self.ObjectStatus.appeared:
                    # controller.SendTakeoff()
                    pass

            elif self.objectTarget.estado == self.ObjectStatus.movedLeft:
                    controller.SetCommand(-1*PORCENTAJE_VELOCIDAD, 0,0,0)
                    # controller.SetCommand(0,0,-1*PORCENTAJE_VELOCIDAD,0)

            # similar a la funcion de arriba, con la diferencia que va para la derecha
            elif self.objectTarget.estado == self.ObjectStatus.movedRight:
                    controller.SetCommand(PORCENTAJE_VELOCIDAD, 0,0,0)
                    # controller.SetCommand(0,0,PORCENTAJE_VELOCIDAD,0)

            # Movimiento en el eje Z
            elif self.objectTarget.estado == self.ObjectStatus.movedUp:
                    # controller.SetCommand(0,0,0,PORCENTAJE_VELOCIDAD)
                    pass

            elif self.objectTarget.estado == self.ObjectStatus.movedDown:
                    # controller.SetCommand(0,0,0,-1*PORCENTAJE_VELOCIDAD)
                    pass
            # Si la Ball estuviera en el centro de la imagen pasa parametros
            # que indican al drone de debe estar parado (cero todas las velocidades)
            elif self.objectTarget.estado == ObjectStatus.samePlace:
                controller.SetCommand(0,0,0,0)

    # adiciona control por teclado en la ventana
    # principal(aquella donde aparece los mensajes de situacion del drone)
    #  para poder aterrizar usando la barra espaciadora y depegar la tecla D
    # tambien es posible usar la tecla R para reiniciar
    # el drone (cuando los LEDs del drone esten en rojo, por ejemplo
    # es preciso reinicializar senal cuando el drone cae desprevenidamente)
    def keyPressEvent(self, event):

        # Recibe una tecla que fue tecleada
        key = event.key()
        # Si el controlador del drone estuviese ok y
        # el teclado no esta en modo de repeticion automotica
        if controller is not None and not event.isAutoRepeat():
            if key == QtCore.Qt.Key.Key_Space:   # Espacio para aterrizar
                controller.SendLand()
            elif key == QtCore.Qt.Key.Key_D: # Tecla "D" para despegar
                controller.SendTakeoff()
            elif key == QtCore.Qt.Key.Key_R: # Tecla "R" para resetear el estado de emergencia
                controller.SendEmergency()
            elif key == QtCore.Qt.Key.Key_P: # Tecla "P" para cambiar la camara
                self.changeCamera("/ardrone/togglecam", Empty)
            elif key == QtCore.Qt.Key.Key_B: # Tecla "B" para cambiar la camara
                self.ledAnimation("/ardrone/setledanimation", LedAnim)
            elif key == QtCore.Qt.Key.Key_X: # Tecla "X" para cambiar la camara
                self.flip_animation("/ardrone/setflightanimation", FlightAnim)


    def changeCamera(self, serviceDrone, estructuraParam):
        rospy.wait_for_service(serviceDrone)
        try:
            proxyDrone = rospy.ServiceProxy(serviceDrone, estructuraParam)
            proxyDrone()
        except rospy.ServiceException, e:
            print("Failed services %s"%(e))

    def ledAnimation(self,serviceDrone, estructuraParam, typeofAnimation=1,
                     frequency=4, duration=5,):
        rospy.wait_for_service(serviceDrone)
        try:
            proxyDrone = rospy.ServiceProxy(serviceDrone, estructuraParam)
            proxyDrone(typeofAnimation, frequency, duration)
        except rospy.ServiceException, e:
            print("Failed services %s"%(e))

    def flip_animation(self,serviceDrone, estructuraParam, typeofAnimation=1, duration=0,):
        rospy.wait_for_service(serviceDrone)
        try:
            proxyDrone = rospy.ServiceProxy(serviceDrone, estructuraParam)
            proxyDrone(typeofAnimation, duration)
        except rospy.ServiceException, e:
            print("Failed services %s"%(e))


if __name__ == '__main__':
    # Inicia el nodo
    rospy.init_node('tracking')

    # Inicia Qt y el controlador del drone (Realiza instancias)
    app = QtGui.QApplication(sys.argv)
    controller = DroneController()
    seguidor = SeguirObjeto()

    # Indica que la ventana principal debe ser mostrada
    seguidor.show()

    # Inicia una ejecucion de una aplicacion basada en Qt
    status = app.exec_()

    # llega aqui cuando la ventana principal es cerrada
    rospy.signal_shutdown('Finish by Vito')
    sys.exit(status)
