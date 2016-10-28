#! /usr/bin/python

from threading import Lock

import rospy

import sys

#TODO QtGui to put combo box for objectTarget, secondaryTarget
# TODO ImageWithoutProcessing, ProcessingImage
# Librerary for GUI (window)
from PySide import QtCore, QtGui

# status of the object(FACE,BALL,BODY)
from Detection.objectStatus import ObjectStatus

# status of the drone
from Drone.drone_status import DroneStatus

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

# messages coming from ROS
#To receive video frames
from sensor_msgs.msg import Image
# Message parameter for changing camera
from std_srvs.srv import Empty

# message ardrone_autonomy
#To receive information on the status of the drone
from ardrone_autonomy.msg import Navdata
#animation for the drone
from ardrone_autonomy.srv import FlightAnim,LedAnim

# To convert the picture type ROS to OpenCV
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
bridge = CvBridge()


class SeguirObjeto(QtGui.QMainWindow):

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
        self.imageS = None

        #settings objects recognition
        self.settings_navigation()

        # title window
        self.setWindowTitle('Vision del drone')
        # imageBox, where the window will be drawn
        #here come the FPS
        self.imageBox = QtGui.QLabel(self)
        self.imageBox.setScaledContents(True)
        #TODO SETTING ALL CONFIG SIZE 640 360
        self.resize(640,360)
        self.setCentralWidget(self.imageBox)
        self.centerOnScreen()
        # # layout = QtGui.QHBoxLayout(self)
        # self.cb = QtGui.QComboBox(self)
        # self.cb.addItem("C")
        # self.cb.addItem("C++")
        # self.cb.addItems(["Java", "C#", "Python"])
        # self.cb.currentIndexChanged.connect(self.selectionchange)

        # layout.addWidget(self.cb)
        # self.setLayout(layout)
        # self.setWindowTitle("combo box demo")

        # to receive data from drone
        self.subNavdata = rospy.Subscriber(
            '/ardrone/navdata', Navdata, self.ReceiveNavdata)

        # to receive images from drone
        self.subVideo = rospy.Subscriber(
            '/ardrone/image_raw', Image, self.ReceiveImage)

        # self.image: save the images captured by the drone
        self.image = None
        # a lock is required so that there is a synchronization
        # between the reception of the current image and the previous
        self.imageLock = Lock()

        #image convert to opencv
        self.imageOpencv = None

        # drone status message to be shown in the GUI
        self.statusMessage = ''

        # used to know whether the drone receive data from the last time the timer disappeared
        self.communicationSinceTimer = False
        self.connected = False

        # Timer to check from time to time if the drone still is connected via WI-FI
        # Create a timer
        self.connectionTimer = QtCore.QTimer(self)
        self.connectionTimer.timeout.connect(self.ConnectionCallback)
        # It indicates the frequency
        self.connectionTimer.start(CONNECTION_CHECK_PERIOD)

        # timer to redraw the interface window in time to time
        self.redrawTimer = QtCore.QTimer(self)
        self.redrawTimer.timeout.connect(self.RedrawCallback)
        self.redrawTimer.start(GUI_UPDATE_PERIOD)

    def centerOnScreen(self):
        resolution = QtGui.QDesktopWidget().screenGeometry()
        self.move((resolution.width() / 2) - (self.frameSize().width() / 2),
                  (resolution.height() / 2) - (self.frameSize().height() / 2))
    def selectionchange(self, i):
        print ("Items in the list are :")

        for count in range(self.cb.count()):
            print (self.cb.itemText(count))
        print ("Current index", i, "selection changed ", self.cb.currentText())

    def settings_navigation(self, detectionObject=False, objectTarget=Face, secondaryTarget=qrCode):
        # ObjectTarget: It is the main object for tracking
        # secondaryTarget: It is used to give an animation to drone like flip
        self.detectionObject = detectionObject
        self.objectTarget = objectTarget()
        self.secondaryTarget = secondaryTarget()

    def buttonAction(self):
        self.settings_navigation()

    def ConnectionCallback(self):
        self.connected = self.communicationSinceTimer
        self.communicationSinceTimer = False

        # ROS IMAGE converted to OpenCV
    def ToOpenCV(self, ros_image):
        try:
            self.imageOpencv = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
        except CvBridgeError as e:
            print (e)
            raise Exception("failure in convertion to OpenCV image")

    def RedrawCallback(self):
        if self.image is not None:
            # By sync problems, the system requests a lock here
            # the image is updated in the window
            self.imageLock.acquire()
            try:
                self.ToOpenCV(self.image)#Covierte de ROS para OpenCV
                # detection object
                if self.detectionObject:
                    self.imageS = self.ProcessingImage(self.imageOpencv)
                else:
                    self.imageS = self.ImageWithoutProcessing(self.imageOpencv)

                pix = QtGui.QPixmap.fromImage(self.imageS)

            finally:
                self.imageLock.release()
            # Displays an image in the window of the GUI
            self.imageBox.setPixmap(pix)
            self.imageBox.setMinimumSize(1, 1)
            self.imageBox.installEventFilter(self)
            # Motion for the drone
            self.moverDrone()

        # updates a message with the current situation of the drone
        self.statusBar().showMessage(
            self.statusMessage if self.connected else self.DisconnectedMessage)

    def eventFilter(self, source, event):
        if (source is self.imageBox and event.type() == QtCore.QEvent.Resize):
            # re-scale the pixmap when the label resizes
            pix = QtGui.QPixmap.fromImage(self.imageS)
            self.imageBox.setPixmap(pix)
        return super(SeguirObjeto, self).eventFilter(source, event)

    def ImageWithoutProcessing(self,image):
        image = QtGui.QImage(image,
                            640,
                            360,
                            QtGui.QImage.Format_RGB888)
        return image

    def ProcessingImage(self,image):
        frame = self.objectTarget.findObject(image)
        frame = self.secondaryTarget.findObject(frame)

        image = QtGui.QImage(frame,
                             frame.shape[1],
                             frame.shape[0],
                             QtGui.QImage.Format_RGB888)
        return image

    # Function that is called when a new image arrives
    def ReceiveImage(self, data):
        # Indicates that there was communication (the picture frame was received)
        self.communicationSinceTimer = True

        # Block to avoid problems with synchronization (occurs because
        # the image is large
        # and gives to stop and copy it through the process
        self.imageLock.acquire()
        try:
            #save image
            self.image = data
        finally:
            self.imageLock.release()

    def ReceiveNavdata(self, navdata):
        # Indicates that there was communication (since data on the drone arrived)
        #  through the port 5556
        self.communicationSinceTimer = True

        # updates the status of the drone in the window
        msg = self.StatusMessages[
            navdata.state] if navdata.state in self.StatusMessages else self.UnknownMessage
        msgTarget = self.MessageSituacion[self.objectTarget.estado]

        # the object type and battery
        self.statusMessage = '{} | Target: {} (Battery: {}%)'.format(
            msg, msgTarget, int(navdata.batteryPercent))


    def moverDrone(self):
        # verifies if the driver is ready to use drone
        if controller is not None:
            if self.secondaryTarget.estado == self.ObjectStatus.appeared:
                    self.flip_animation("/ardrone/setflightanimation", FlightAnim)
                    # self.changeCamera_flatTrim("/ardrone/togglecam", Empty)

            elif self.objectTarget.estado == self.ObjectStatus.disapared:
                    # controller.SendTakeoff_SendLand()
                    pass

            elif self.objectTarget.estado == self.ObjectStatus.appeared:
                    # controller.SendTakeoff_SendLand()
                    pass

            elif self.objectTarget.estado == self.ObjectStatus.movedLeft:
                    # controller.SetCommand(-1*PORCENTAJE_VELOCIDAD, 0,0,0)
                    # controller.SetCommand(0,0,-1*PORCENTAJE_VELOCIDAD,0)
                    pass

            # similar a la funcion de arriba, con la diferencia que va para la derecha
            elif self.objectTarget.estado == self.ObjectStatus.movedRight:
                    # controller.SetCommand(PORCENTAJE_VELOCIDAD, 0,0,0)
                    # controller.SetCommand(0,0,PORCENTAJE_VELOCIDAD,0)
                    pass

            # Movimiento en el eje Z
            elif self.objectTarget.estado == self.ObjectStatus.movedUp:
                    # controller.SetCommand(0,0,0,PORCENTAJE_VELOCIDAD)
                    pass

            elif self.objectTarget.estado == self.ObjectStatus.movedDown:
                    # controller.SetCommand(0,0,0,-1*PORCENTAJE_VELOCIDAD)
                    pass

            # backward
            elif self.objectTarget.estado == self.ObjectStatus.movedFront:
                    # controller.SetCommand(0,PORCENTAJE_VELOCIDAD, 0, 0)
                    pass

            # forward
            elif self.objectTarget.estado == self.ObjectStatus.movedBack:
                    # controller.SetCommand(0,-1*PORCENTAJE_VELOCIDAD, 0, 0)
                    pass

            elif self.objectTarget.estado == ObjectStatus.samePlace:
                    # controller.SetCommand()
                    pass
    # keyboard, keys to move the drone
    def keyPressEvent(self, event):

        key = event.key()
        if controller is not None and not event.isAutoRepeat():
            if key == QtCore.Qt.Key.Key_Space:   # space for  takeoff or land
                controller.Takeoff_Land_toggle()
            elif key == QtCore.Qt.Key_W: # key "W" for forward
                controller.SetCommand(0, PORCENTAJE_VELOCIDAD, 0, 0)
            elif key == QtCore.Qt.Key_S: # key "S" for backward
                controller.SetCommand(0, -1*PORCENTAJE_VELOCIDAD, 0, 0)
            elif key == QtCore.Qt.Key_A: # key "A" for left
                controller.SetCommand(PORCENTAJE_VELOCIDAD, 0, 0, 0)
            elif key == QtCore.Qt.Key_D: # key "D" for right
                controller.SetCommand(-1*PORCENTAJE_VELOCIDAD, 0, 0, 0)
            elif key == QtCore.Qt.Key_Right: # key "-->" for Right Vertex z
                controller.SetCommand(0, 0, PORCENTAJE_VELOCIDAD, 0)
            elif key == QtCore.Qt.Key_Left: # key "<--" for Left Vertex z
                controller.SetCommand(0, 0, -1*PORCENTAJE_VELOCIDAD, 0)
            elif key == QtCore.Qt.Key_Up: # key "Up" for up Vertex z
                controller.SetCommand(0, 0, 0, PORCENTAJE_VELOCIDAD)
            elif key == QtCore.Qt.Key_Down: # key "Down" for Down Vertex z
                controller.SetCommand(0, 0, 0, -1*PORCENTAJE_VELOCIDAD)
            elif key == QtCore.Qt.Key_Q: # key "Q" for to stay
                controller.SetCommand()
            elif key == QtCore.Qt.Key_R: # key "R" change emergency mode
                controller.SendEmergency()
            elif key == QtCore.Qt.Key_P: # key "P" change camera
                self.changeCamera_flatTrim("/ardrone/togglecam", Empty)
            elif key == QtCore.Qt.Key_B: # key "B" animation led
                self.ledAnimation("/ardrone/setledanimation", LedAnim)
            elif key == QtCore.Qt.Key_X: # key "X" FLIP animation
                self.flip_animation("/ardrone/setflightanimation", FlightAnim)
            elif key == QtCore.Qt.Key_F: # key "F" flat trim
                self.changeCamera_flatTrim("/ardrone/flattrim", Empty)
            elif key == QtCore.Qt.Key_Z: # key "z" detection
                if self.detectionObject:
                    self.detectionObject= False
                else:
                    self.detectionObject = True

    # service change camera and flat trim receive same parameter
    def changeCamera_flatTrim(self, serviceDrone, estructuraParam):
        rospy.wait_for_service(serviceDrone)
        try:
            proxyDrone = rospy.ServiceProxy(serviceDrone, estructuraParam)
            proxyDrone()
        except rospy.ServiceException as e:
            print("Failed services %s"%(e))

    def ledAnimation(self,serviceDrone, estructuraParam, typeofAnimation=1,
                     frequency=4, duration=5,):
        rospy.wait_for_service(serviceDrone)
        try:
            proxyDrone = rospy.ServiceProxy(serviceDrone, estructuraParam)
            proxyDrone(typeofAnimation, frequency, duration)
        except rospy.ServiceException as e:
            print("Failed services %s"%(e))

    def flip_animation(self,serviceDrone, estructuraParam, typeofAnimation=1, duration=0,):
        rospy.wait_for_service(serviceDrone)
        try:
            proxyDrone = rospy.ServiceProxy(serviceDrone, estructuraParam)
            proxyDrone(typeofAnimation, duration)
        except rospy.ServiceException as e:
            print("Failed services %s"%(e))

if __name__ == '__main__':
    # Starts the node
    rospy.init_node('tracking')

    #Qt and the controller starts Drone
    app = QtGui.QApplication(sys.argv)
    controller = DroneController()
    seguidor = SeguirObjeto()

    # SHOW  GUI WINDOW
    seguidor.show()
    # Initiates an execution of an application based on Qt
    status = app.exec_()

    # This happens when the window is closed
    rospy.signal_shutdown('Finish by Vito')
    sys.exit(status)
