#! /usr/bin/python

from threading import Lock

import rospy
import cv2

import sys

# Library for GUI (window)
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

#controlls
from controll import WindowControl

# settings
from settings import (CONNECTION_CHECK_PERIOD,
                      GUI_UPDATE_PERIOD,
                      PORCENTAJE_VELOCIDAD)

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
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()


class SeguirObjeto(QtGui.QMainWindow):

    # status of the drone for the windows
    DroneStatus = DroneStatus()
    ObjectStatus = ObjectStatus()
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
    # object detection
    list_objects = [Ball, qrCode, Face, Body]

    # key for opencv  another way convert to ASCII
    # 1048695 &0xff(hexadecimal)
    # from key to bytearray ASCII(ord) example ord('a') --> 119
    k_w = 1048695
    k_s = 1048691
    k_a = 1048673
    k_d = 1048676
    k_space = 1048608
    k_left = 1113937
    k_right = 1113939
    k_up = 1113938
    k_down = 1113940
    k_p = 1048688
    k_l = 1048684
    k_z = 1048698
    k_x = 1048696
    k_q = 1048689
    k_2 = 1048626
    k_r = 1048690
    k_f = 1048678

    # self.image: save the images captured by the drone
    image = None
    # a lock is required so that there is a synchronization
    # between the reception of the current image and the previous
    imageLock = Lock()

    # image convert to opencv
    imageOpencv = None

    roi = None # for selection image
    # To cut image from vision drone
    refPt = []
    cropping = False

    # show message in window
    statusMessage = ''
    statusConnect = ''
    statusBattery = ''

    def __init__(self):

        super(SeguirObjeto, self).__init__()

        #WINDOW1 --> Drone Vision
        cv2.namedWindow('Drone Vision', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Drone Vision', 640, 360)
        cv2.setMouseCallback("Drone Vision", self.click_and_crop)
        xcenter,ycenter = self.center_on_screen()
        cv2.moveWindow('Drone Vision',xcenter,ycenter)

        #WINDOW2 --> Vision del drone
        self.setWindowTitle('Vision del drone')
        self.controllers = WindowControl(self)
        self.controllers.object_detection_main.activated.connect(self.main_change_object)
        self.controllers.secondary_object_detection.activated.connect(self.secondary_change_object)
        self.setCentralWidget(self.controllers)

        # WINDOW3 --> ROI
        cv2.namedWindow('ROI', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('ROI', 640, 360)

        #settings objects recognition
        self.settings_navigation()

        # to receive data from drone
        self.subNavdata = rospy.Subscriber(
            '/ardrone/navdata', Navdata, self.receive_navdata)

        # to receive images from drone
        self.subVideo = rospy.Subscriber(
            '/ardrone/image_raw', Image, self.receive_image)

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
        centre = ((resolution.width() / 2) - (self.frameSize().width() / 2),
                  (resolution.height() / 2) - (self.frameSize().height() / 2))
        return centre

    def main_change_object(self, index):
        for order, object in enumerate(self.list_objects):
            if order == index:
                self.objectTarget = object()

    def secondary_change_object(self, index):
        for order, object in enumerate(self.list_objects):
            if order == index:
                self.secondaryTarget = object()

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
            # 'bgr8' | desired_encoding = "passthrough"
            self.imageOpencv = bridge.imgmsg_to_cv2(ros_image, 'bgr8')
        except CvBridgeError as e:
            raise Exception("failure in conversion OpenCV image: {}".format(e))

    def redraw_callback(self):
        if self.image is not None:
            # By sync problems, the system requests a lock here
            # the image is updated in the window
            self.imageLock.acquire()
            try:
                self.to_opencv(self.image) #Convert from ROS to OpenCV

                if self.detectionObject: # detection object OFF or ON
                    self.find_objects_all(self.imageOpencv)

                # Draw rectangle until drop it mouse
                if not self.cropping:
                    cv2.imshow('Drone Vision', self.imageOpencv)
                elif self.refPt:
                    cv2.rectangle(self.imageOpencv, self.refPt[0], self.refPt[-1], (0, 255, 0), 2)
                    cv2.imshow('Drone Vision', self.imageOpencv)  # show selection in WINDOW 'Drone Vision'

            finally:
                self.imageLock.release()

            self.something(cv2.waitKey(33))# Capture key from WINDOW --> Drone Vision

            self.mover_drone()
        self.show_status_in_window()

    def show_status_in_window(self):
        # updates a message with the current situation of the drone
        self.controllers.status_drone.setText(self.statusConnect)
        self.controllers.battery_status.setText(str(self.statusBattery))
        self.controllers.status_objects_detection.setText(self.statusMessage)

    def find_objects_all(self,image):
        self.imageOpencv = self.objectTarget.find_object(image)
        # self.imageOpencv = self.secondaryTarget.find_object(self.imageOpencv)

        # objects_to_detection = [self.objectTarget, self.secondaryTarget]
        # for object in objects_to_detection:
        #     self.imageOpencv = object.find_object(image)

    def click_and_crop(self,event, x, y, flags, param):
        # if the left mouse button was clicked, record the starting
        # (x, y) coordinates and indicate that cropping is being
        # performed
        if event == cv2.EVENT_LBUTTONDOWN:
            self.refPt = [(x, y)]
            self.cropping = True

        elif event == cv2.EVENT_MOUSEMOVE and self.cropping:
            self.refPt.append((x, y))

        # check to see if the left mouse button was released
        elif event == cv2.EVENT_LBUTTONUP:
            # record the ending (x, y) coordinates and indicate that
            # the cropping operation is finished
            self.refPt.append((x, y))
            self.cropping = False
            # # draw a rectangle around the region of interest
            cv2.rectangle(self.imageOpencv, self.refPt[0], self.refPt[-1], (0, 255, 0), 2)
            cv2.imshow('Drone Vision', self.imageOpencv) # show selection in WINDOW 'Drone Vision'

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

        msgTarget = self.MessageSituacion[self.objectTarget.estado]
        # object status
        self.statusMessage = str(msgTarget)

        # updates the status of the drone in the window
        msg = self.StatusMessages[
            navdata.state] if navdata.state in self.StatusMessages else self.UnknownMessage
        # drone "status and battery"
        self.statusConnect = str(msg)
        self.statusBattery = int(navdata.batteryPercent)

    def show_me_selection(self):
        if isinstance(self.objectTarget, Ball):
            # to cut image from point A to B
            # A = x(i),y(i)
            # B = x(i),y(i)

            # where
            # Point A
            # x(1) = refPt[0][1]
            # y(1) = refPt[1][1]
            #
            # Point B
            # x(2) = refPt[0][0]
            # y(2) = refPt[1][0]
            self.roi = self.imageOpencv[self.refPt[0][1]:self.refPt[-1][1],
                                      self.refPt[0][0]:self.refPt[-1][0]]
            # detect color and set color
            self.objectTarget.change_object_color(self.roi)
            cv2.imshow("ROI", self.roi)  # show third WINDOW --> ROI

    def detection_toggle(self):
        if self.detectionObject:
            self.detectionObject = False
        else:
            self.detectionObject = True

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
    def something(self, key):
        if key != -1:
            if key == self.k_space:  # space for  takeoff or land
                controller.takeoff_land_toggle()# necessary battery for takeoff is more 20%
            elif key == self.k_w:  # key "W" for forward
                controller.set_command(0, PORCENTAJE_VELOCIDAD, 0, 0)
            elif key == self.k_s:  # key "S" for backward
                controller.set_command(0, -1 * PORCENTAJE_VELOCIDAD, 0, 0)
            elif key == self.k_a:  # key "A" for left
                controller.set_command(PORCENTAJE_VELOCIDAD, 0, 0, 0)
            elif key == self.k_d:  # key "D" for right
                controller.set_command(-1 * PORCENTAJE_VELOCIDAD, 0, 0, 0)
            elif key == self.k_right:  # key "-->" for Right Vertex z
                controller.set_command(0, 0, PORCENTAJE_VELOCIDAD, 0)
            elif key == self.k_left:  # key "<--" for Left Vertex z
                controller.set_command(0, 0, -1 * PORCENTAJE_VELOCIDAD, 0)
            elif key == self.k_up:  # key "Up" for up Vertex z
                controller.set_command(0, 0, 0, PORCENTAJE_VELOCIDAD)
            elif key == self.k_down:  # key "Down" for Down Vertex z
                controller.set_command(0, 0, 0, -1 * PORCENTAJE_VELOCIDAD)
            elif key == self.k_q:  # key "Q" for to stay
                controller.set_command()
            elif key == self.k_r:  # key "R" change emergency mode
                controller.send_emergency()
            elif key == self.k_p:  # key "P" change camera
                self.change_camera_flatTrim("/ardrone/togglecam", Empty)
            elif key == self.k_l:  # key "L" animation led
                self.led_animation("/ardrone/setledanimation", LedAnim)
            elif key == self.k_x:  # key "X" FLIP animation
                self.flip_animation("/ardrone/setflightanimation", FlightAnim)
            elif key == self.k_f:  # key "F" flat trim
                self.change_camera_flatTrim("/ardrone/flattrim", Empty)
            elif key == self.k_2:
                self.show_me_selection()
            elif key == self.k_z:  # key "z" detection
                self.detection_toggle()

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
