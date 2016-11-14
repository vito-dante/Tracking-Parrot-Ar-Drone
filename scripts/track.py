#! /usr/bin/python

from threading import Lock

import rospy
import cv2

import sys

# Library for GUI (window)
from PySide import QtCore, QtGui

# status of the object(FACE,BALL,BODY)
from Detection.object_status import ObjectStatus


from Drone.drone_commands import DroneCommands

#Objects for detection
from Detection.ball import Ball
from Detection.body import Body
from Detection.face import Face
from Detection.qrcode import qrCode

#controlls
from window_control import WindowControl

# settings
from settings import (CONNECTION_CHECK_PERIOD,
                      GUI_UPDATE_PERIOD,
                      PORCENTAJE_VELOCIDAD)

# messages coming from ROS
#To receive video frames
from sensor_msgs.msg import Image

# convert image from ROS to opencv
from ros_opencv import to_opencv

#keyboard keys
from keyboard import Keyboard

class TrackingObject(QtGui.QMainWindow):

    # status of the object
    ObjectStatus = ObjectStatus()

    DisconnectedMessage = 'Disconnect'

    # object detection
    list_objects = [Ball, qrCode, Face, Body]

    # self.image: save the images captured by the drone
    image = None
    # a lock is required so that there is a synchronization
    # between the reception of the current image and the previous
    imageLock = Lock()

    # image convert to opencv
    imageOpencv = None

    roi = None # for selection image
    # To cut image from vision drone
    refPt = [] #Reference Points
    cropping = False # Flag for to check if you are doing a cut

    # show message in window
    statusMessage = ''

    def __init__(self):

        super(TrackingObject, self).__init__()

        #WINDOW1 --> Drone Vision
        cv2.namedWindow('Drone Vision', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Drone Vision', 640, 360)
        cv2.setMouseCallback("Drone Vision", self.click_and_crop)
        xcenter,ycenter = self.center_on_screen()
        cv2.moveWindow('Drone Vision',xcenter,ycenter)

        #WINDOW2 --> Window controller
        self.setWindowTitle('Control')
        self.controllers = WindowControl(self)
        self.setStyleSheet("background-color:white;")
        self.controllers.object_detection_main.activated.connect(self.main_change_object)
        self.controllers.secondary_object_detection.activated.connect(self.secondary_change_object)
        self.setCentralWidget(self.controllers)
        self.move(xcenter,ycenter)

        # WINDOW3 --> ROI
        cv2.namedWindow('ROI', cv2.WINDOW_NORMAL)

        #settings objects recognition
        self.settings_navigation()

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
        self.show_status_in_window()

    def redraw_callback(self):
        if self.image is not None:
            # By sync problems, the system requests a lock here
            # the image is updated in the window
            self.imageLock.acquire()
            try:
                self.imageOpencv = to_opencv(self.image) #Convert from ROS to OpenCV

                if self.detectionObject: # detection object OFF or ON
                    self.find_objects_all(self.imageOpencv,self.objects_selections())
                # Draw rectangle until drop it mouse
                if not self.cropping:
                    cv2.imshow('Drone Vision', self.imageOpencv)
                elif self.refPt:
                    cv2.rectangle(self.imageOpencv, self.refPt[0], self.refPt[-1], (0, 255, 0), 2)
                    cv2.imshow('Drone Vision', self.imageOpencv)  # show selection in WINDOW 'Drone Vision'

            finally:
                self.imageLock.release()

            self.keyboard(cv2.waitKey(33))# Capture key from WINDOW --> Drone Vision

            self.mover_drone()

    def objects_selections(self):
        if self.controllers.box_secondary_object.isChecked():
            return [self.objectTarget, self.secondaryTarget]
        else:
            return [self.objectTarget]

    def show_status_in_window(self):
        # check according to CONNECTION_CHECK_PERIOD
        if self.connected:
            self.controllers.statusLayout.setHidden(False)
            # updates a message with the current situation of the drone
            self.controllers.status_drone.setText(drone_commands.show_status_message())
            self.controllers.battery_status.setText(drone_commands.show_battery())
            self.controllers.status_objects_detection.setText(self.objectTarget.status_msg())
        else:
            self.resize(180,40)
            self.controllers.statusLayout.setHidden(True)
            self.controllers.status_drone.setText(self.DisconnectedMessage)

    def find_objects_all(self,image, objectDetections):
        for object in objectDetections:
            image = object.find_object(image)

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
        # Indicates that there was communication (since data on the drone arrived)
        #  through the port 5556
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
        if drone_commands is not None:
            if self.secondaryTarget.estado == self.ObjectStatus.appeared:
                    drone_commands.flip_animation()


            elif self.objectTarget.estado == self.ObjectStatus.disapared:
                    # drone_commands.SendTakeoff_SendLand()
                    pass

            elif self.objectTarget.estado == self.ObjectStatus.appeared:
                    # drone_commands.SendTakeoff_SendLand()
                    pass

            elif self.objectTarget.estado == self.ObjectStatus.moved_left:
                    # drone_commands.set_command(-1*PORCENTAJE_VELOCIDAD, 0,0,0)
                    # drone_commands.set_command(0,0,-1*PORCENTAJE_VELOCIDAD,0)
                    pass

            # similar a la funcion de arriba, con la diferencia que va para la derecha
            elif self.objectTarget.estado == self.ObjectStatus.moved_right:
                    # drone_commands.set_command(PORCENTAJE_VELOCIDAD, 0,0,0)
                    # drone_commands.set_command(0,0,PORCENTAJE_VELOCIDAD,0)
                    pass

            # Movimiento en el eje Z
            elif self.objectTarget.estado == self.ObjectStatus.moved_up:
                    # drone_commands.set_command(0,0,0,PORCENTAJE_VELOCIDAD)
                    pass

            elif self.objectTarget.estado == self.ObjectStatus.moved_down:
                    # drone_commands.set_command(0,0,0,-1*PORCENTAJE_VELOCIDAD)
                    pass

            # backward
            elif self.objectTarget.estado == self.ObjectStatus.moved_front:
                    # drone_commands.set_command(0,PORCENTAJE_VELOCIDAD, 0, 0)
                    pass

            # forward
            elif self.objectTarget.estado == self.ObjectStatus.moved_back:
                    # drone_commands.set_command(0,-1*PORCENTAJE_VELOCIDAD, 0, 0)
                    pass

            elif self.objectTarget.estado == ObjectStatus.same_place:
                    # drone_commands.set_command()
                    pass
    # keyboard, keys to move the drone
    def keyboard(self, key):
        if key != -1:
            if key == Keyboard.k_space:  # space for  takeoff or land
                drone_commands.takeoff_land_toggle()# necessary battery for takeoff is more 20%
            elif key == Keyboard.k_w:  # key "W" for forward
                drone_commands.set_command(0, PORCENTAJE_VELOCIDAD, 0, 0)
            elif key == Keyboard.k_s:  # key "S" for backward
                drone_commands.set_command(0, -1 * PORCENTAJE_VELOCIDAD, 0, 0)
            elif key == Keyboard.k_a:  # key "A" for left
                drone_commands.set_command(PORCENTAJE_VELOCIDAD, 0, 0, 0)
            elif key == Keyboard.k_d:  # key "D" for right
                drone_commands.set_command(-1 * PORCENTAJE_VELOCIDAD, 0, 0, 0)
            elif key == Keyboard.k_right:  # key "-->" for Right Vertex z
                drone_commands.set_command(0, 0, PORCENTAJE_VELOCIDAD, 0)
            elif key == Keyboard.k_left:  # key "<--" for Left Vertex z
                drone_commands.set_command(0, 0, -1 * PORCENTAJE_VELOCIDAD, 0)
            elif key == Keyboard.k_up:  # key "Up" for up Vertex z
                drone_commands.set_command(0, 0, 0, PORCENTAJE_VELOCIDAD)
            elif key == Keyboard.k_down:  # key "Down" for Down Vertex z
                drone_commands.set_command(0, 0, 0, -1 * PORCENTAJE_VELOCIDAD)
            elif key == Keyboard.k_q:  # key "Q" for to stay
                drone_commands.set_command()
            elif key == Keyboard.k_r:  # key "R" change emergency mode
                drone_commands.send_emergency()
            elif key == Keyboard.k_p:  # key "P" change camera
                drone_commands.change_camera()
            elif key == Keyboard.k_l:  # key "L" animation led
                drone_commands.led_animation()
            elif key == Keyboard.k_x:  # key "X" FLIP animation
                drone_commands.flip_animation()
            elif key == Keyboard.k_f:  # key "F" flat trim
                drone_commands.flatTrim()
            elif key == Keyboard.k_2:
                self.show_me_selection()
            elif key == Keyboard.k_z:  # key "z" detection
                self.detection_toggle()


if __name__ == '__main__':
    # Starts the node
    rospy.init_node('tracking')

    #Qt and the controller starts Drone
    app = QtGui.QApplication(sys.argv)
    drone_commands = DroneCommands()
    Track = TrackingObject()

    # SHOW  GUI WINDOW
    Track.show()
    # Initiates an execution of an application based on Qt
    status = app.exec_()

    # This happens when the window is closed
    rospy.signal_shutdown('Finish by Vito')
    sys.exit(status)