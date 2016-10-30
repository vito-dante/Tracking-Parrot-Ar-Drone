#! /usr/bin/python

from threading import Lock

import rospy
import cv2

import sys

#TODO QtGui to put combo box for objectTarget, secondaryTarget
# TODO image_without_processing, processing_image
# Librerary for GUI (window)
from PySide import QtCore, QtGui

# status of the object(FACE,BALL,BODY)
from Detection.object_status import ObjectStatus

# status of the drone
from Drone.drone_status import DroneStatus

from Drone.drone_controller import DroneController

#Objects for detection
from Detection.ball import Ball
from Detection.body import Body
from Detection.face import Face
from Detection.qrcode import qrCode


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

    refPt = []
    cropping = False

    # status of the drone for the windows
    StatusMessages = {
        DroneStatus.emergency: 'Emergencia',
        DroneStatus.inited: 'Inciado',
        DroneStatus.landed: 'Aterrizado',
        DroneStatus.flying: 'Volando',
        DroneStatus.hovering: 'Parado en el aire',
        DroneStatus.test: 'Es un Test ?',
        DroneStatus.taking_off: 'Despegando',
        DroneStatus.go_to_hover: 'entrando en el modo parado en el aire ',
        DroneStatus.landing: 'Aterrizando',
        DroneStatus.looping: 'Realizando un Loop ?'
    }
    DisconnectedMessage = 'Desconectado'
    UnknownMessage = 'Estado Desconocido'

    # status of the object for the window
    MessageSituacion = {
        ObjectStatus.appeared: 'Aparecio',
        ObjectStatus.disapared: 'Desaparecio',
        ObjectStatus.moved_left: 'MovioParaIzquierda',
        ObjectStatus.moved_right: 'MovioParaDerecha',
        ObjectStatus.moved_up: 'MovioParaArriba',
        ObjectStatus.moved_down: 'MovioParaAbajo',
        ObjectStatus.moved_front: 'MovioParaFrente',
        ObjectStatus.moved_back: 'MovioParaAtraz',
        ObjectStatus.same_place: 'Parada'
    }

    def __init__(self):

        super(SeguirObjeto, self).__init__()

        #crop image ROI
        cv2.namedWindow("image")
        cv2.setMouseCallback("image", self.click_and_crop)

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
        self.center_on_screen()

        # to receive data from drone
        self.subNavdata = rospy.Subscriber(
            '/ardrone/navdata', Navdata, self.receive_navdata)

        # to receive images from drone
        self.subVideo = rospy.Subscriber(
            '/ardrone/image_raw', Image, self.receive_image)

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
        self.connectionTimer.timeout.connect(self.connection_callback)
        # It indicates the frequency
        self.connectionTimer.start(CONNECTION_CHECK_PERIOD)

        # timer to redraw the interface window in time to time
        self.redrawTimer = QtCore.QTimer(self)
        self.redrawTimer.timeout.connect(self.redraw_callback)
        self.redrawTimer.start(GUI_UPDATE_PERIOD)

    def center_on_screen(self):
        resolution = QtGui.QDesktopWidget().screenGeometry()
        self.move((resolution.width() / 2) - (self.frameSize().width() / 2),
                  (resolution.height() / 2) - (self.frameSize().height() / 2))

    def settings_navigation(self, detectionObject=False, objectTarget=Ball, secondaryTarget=qrCode):
        # ObjectTarget: It is the main object for tracking
        # secondaryTarget: It is used to give an animation to drone like flip
        self.detectionObject = detectionObject
        self.objectTarget = objectTarget()
        self.secondaryTarget = secondaryTarget()

    def connection_callback(self):
        self.connected = self.communicationSinceTimer
        self.communicationSinceTimer = False

        # ROS IMAGE converted to OpenCV
    def to_opencv(self, ros_image):
        try:
            self.imageOpencv = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
        except CvBridgeError as e:
            print (e)
            raise Exception("failure in convertion to OpenCV image")

    def redraw_callback(self):
        if self.image is not None:
            # By sync problems, the system requests a lock here
            # the image is updated in the window
            self.imageLock.acquire()
            try:
                self.to_opencv(self.image)#Covierte de ROS para OpenCV
                # show window from OpenCv
                cv2.imshow("image", self.imageOpencv)

                # detection object
                if self.detectionObject:
                    image = self.processing_image(self.imageOpencv)
                else:
                    image = self.image_without_processing(self.imageOpencv)
                pix = QtGui.QPixmap.fromImage(image)
            finally:
                self.imageLock.release()
            # Displays an image in the window of the GUI
            self.imageBox.setPixmap(pix)
            # self.imageBox.setMinimumSize(320, 180)
            # Motion for the drone
            self.mover_drone()

        # updates a message with the current situation of the drone
        self.statusBar().showMessage(
            self.statusMessage if self.connected else self.DisconnectedMessage)

    def click_and_crop(self,event, x, y, flags, param):
        # if the left mouse button was clicked, record the starting
        # (x, y) coordinates and indicate that cropping is being
        # performed
        if event == cv2.EVENT_LBUTTONDOWN:
            self.refPt = [(x, y)]
            self.cropping = True
        # check to see if the left mouse button was released
        elif event == cv2.EVENT_LBUTTONUP:
            # record the ending (x, y) coordinates and indicate that
            # the cropping operation is finished
            self.refPt.append((x, y))
            self.cropping = False
            # draw a rectangle around the region of interest
            cv2.rectangle(self.imageOpencv, self.refPt[0], self.refPt[1], (0, 255, 0), 2)
            cv2.imshow("image", self.imageOpencv)

    def image_without_processing(self, image):
        image = QtGui.QImage(image,
                            640,
                            360,
                            QtGui.QImage.Format_RGB888)
        return image

    def processing_image(self, image):
        frame = self.objectTarget.find_object(image)
        frame = self.secondaryTarget.find_object(frame)

        image = QtGui.QImage(frame,
                             frame.shape[1],
                             frame.shape[0],
                             QtGui.QImage.Format_RGB888)
        return image

    # Function that is called when a new image arrives
    def receive_image(self, data):
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

    def receive_navdata(self, navdata):
        # Indicates that there was communication (since data on the drone arrived)
        #  through the port 5556
        self.communicationSinceTimer = True

        # updates the status of the drone in the window
        msg = self.StatusMessages[
            navdata.state] if navdata.state in self.StatusMessages else self.UnknownMessage
        msgTarget = self.MessageSituacion[self.objectTarget.estado]

        # the object type and battery
        self.statusMessage = '{} | Target: {} Status: {} |Target2:{} |(Battery: {}%)'.format(
            msg, "FaceDetection",msgTarget,"nothign", int(navdata.batteryPercent))

    def mover_drone(self):
        # verifies if the driver is ready to use drone
        if controller is not None:
            if self.secondaryTarget.estado == self.ObjectStatus.appeared:
                    self.flip_animation("/ardrone/setflightanimation", FlightAnim)
                    # self.change_camera_flatTrim("/ardrone/togglecam", Empty)

            elif self.objectTarget.estado == self.ObjectStatus.disapared:
                    # controller.SendTakeoff_SendLand()
                    pass

            elif self.objectTarget.estado == self.ObjectStatus.appeared:
                    # controller.SendTakeoff_SendLand()
                    pass

            elif self.objectTarget.estado == self.ObjectStatus.moved_left:
                    # controller.set_command(-1*PORCENTAJE_VELOCIDAD, 0,0,0)
                    # controller.set_command(0,0,-1*PORCENTAJE_VELOCIDAD,0)
                    pass

            # similar a la funcion de arriba, con la diferencia que va para la derecha
            elif self.objectTarget.estado == self.ObjectStatus.moved_right:
                    # controller.set_command(PORCENTAJE_VELOCIDAD, 0,0,0)
                    # controller.set_command(0,0,PORCENTAJE_VELOCIDAD,0)
                    pass

            # Movimiento en el eje Z
            elif self.objectTarget.estado == self.ObjectStatus.moved_up:
                    # controller.set_command(0,0,0,PORCENTAJE_VELOCIDAD)
                    pass

            elif self.objectTarget.estado == self.ObjectStatus.moved_down:
                    # controller.set_command(0,0,0,-1*PORCENTAJE_VELOCIDAD)
                    pass

            # backward
            elif self.objectTarget.estado == self.ObjectStatus.moved_front:
                    # controller.set_command(0,PORCENTAJE_VELOCIDAD, 0, 0)
                    pass

            # forward
            elif self.objectTarget.estado == self.ObjectStatus.moved_back:
                    # controller.set_command(0,-1*PORCENTAJE_VELOCIDAD, 0, 0)
                    pass

            elif self.objectTarget.estado == ObjectStatus.same_place:
                    # controller.set_command()
                    pass
    # keyboard, keys to move the drone
    def keyPressEvent(self, event):

        key = event.key()
        if controller is not None and not event.isAutoRepeat():
            if key == QtCore.Qt.Key.Key_Space:   # space for  takeoff or land
                controller.takeoff_land_toggle()
            elif key == QtCore.Qt.Key_W: # key "W" for forward
                controller.set_command(0, PORCENTAJE_VELOCIDAD, 0, 0)
            elif key == QtCore.Qt.Key_S: # key "S" for backward
                controller.set_command(0, -1 * PORCENTAJE_VELOCIDAD, 0, 0)
            elif key == QtCore.Qt.Key_A: # key "A" for left
                controller.set_command(PORCENTAJE_VELOCIDAD, 0, 0, 0)
            elif key == QtCore.Qt.Key_D: # key "D" for right
                controller.set_command(-1 * PORCENTAJE_VELOCIDAD, 0, 0, 0)
            elif key == QtCore.Qt.Key_Right: # key "-->" for Right Vertex z
                controller.set_command(0, 0, PORCENTAJE_VELOCIDAD, 0)
            elif key == QtCore.Qt.Key_Left: # key "<--" for Left Vertex z
                controller.set_command(0, 0, -1 * PORCENTAJE_VELOCIDAD, 0)
            elif key == QtCore.Qt.Key_Up: # key "Up" for up Vertex z
                controller.set_command(0, 0, 0, PORCENTAJE_VELOCIDAD)
            elif key == QtCore.Qt.Key_Down: # key "Down" for Down Vertex z
                controller.set_command(0, 0, 0, -1 * PORCENTAJE_VELOCIDAD)
            elif key == QtCore.Qt.Key_Q: # key "Q" for to stay
                controller.set_command()
            elif key == QtCore.Qt.Key_R: # key "R" change emergency mode
                controller.send_emergency()
            elif key == QtCore.Qt.Key_P: # key "P" change camera
                self.change_camera_flatTrim("/ardrone/togglecam", Empty)
            elif key == QtCore.Qt.Key_B: # key "B" animation led
                self.led_animation("/ardrone/setledanimation", LedAnim)
            elif key == QtCore.Qt.Key_X: # key "X" FLIP animation
                self.flip_animation("/ardrone/setflightanimation", FlightAnim)
            elif key == QtCore.Qt.Key_F: # key "F" flat trim
                self.change_camera_flatTrim("/ardrone/flattrim", Empty)
            elif key == QtCore.Qt.Key_2:
                if type(self.objectTarget) is type(Ball()):
                    # if there are two reference points, then crop the region of interest
                    # from teh image and display it
                    if len(self.refPt) == 2:
                        clone = self.imageOpencv.copy()
                        roi = clone[self.refPt[0][1]:self.refPt[1][1], self.refPt[0][0]:self.refPt[1][0]]
                        min_val, max_val, _, _ = cv2.minMaxLoc(roi)
                        print(min_val)
                        print(max_val)
                        print("HERE")
                        cv2.imshow("ROI", roi)
                        # self.objectTarget.set_color_hsv([12,12,12],[24,54,65])

            elif key == QtCore.Qt.Key_Z: # key "z" detection
                if self.detectionObject:
                    self.detectionObject= False
                else:
                    self.detectionObject = True

    # service change camera and flat trim receive same parameter
    def change_camera_flatTrim(self, serviceDrone, estructuraParam):
        rospy.wait_for_service(serviceDrone)
        try:
            proxyDrone = rospy.ServiceProxy(serviceDrone, estructuraParam)
            proxyDrone()
        except rospy.ServiceException as e:
            print("Failed services %s"%(e))

    def led_animation(self, serviceDrone, estructuraParam, typeofAnimation=1,
                      frequency=4, duration=5, ):
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