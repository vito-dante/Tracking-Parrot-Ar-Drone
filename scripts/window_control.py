from PySide import QtGui

class WindowControl(QtGui.QWidget):

    def __init__(self, window):
        super(WindowControl, self).__init__(window)

        # Selection object main
        self.box_target_main = QtGui.QGroupBox("Target Main")
        label_target = QtGui.QLabel("Object:")
        self.object_detection_main = QtGui.QComboBox()
        self.object_detection_main.addItem("Ball")
        self.object_detection_main.addItem("qrCode")
        self.object_detection_main.addItem("Face")
        self.object_detection_main.addItem("Body")
        label_status_object = QtGui.QLabel("status:")
        self.status_objects_detection = QtGui.QLabel()

        target_main_grid = QtGui.QGridLayout()
        target_main_grid.addWidget(label_target, 0, 0)
        target_main_grid.addWidget(self.object_detection_main, 0, 1)
        target_main_grid.addWidget(label_status_object, 1, 0)
        target_main_grid.addWidget(self.status_objects_detection, 1, 1)
        self.box_target_main.setLayout(target_main_grid)

        # Selection Secondary object
        self.box_secondary_object = QtGui.QGroupBox("Secondary object")
        self.box_secondary_object.setCheckable(True)
        self.box_secondary_object.setChecked(False)

        self.secondary_object_detection = QtGui.QComboBox()
        self.secondary_object_detection.addItem("Ball")
        self.secondary_object_detection.addItem("qrCode")
        self.secondary_object_detection.addItem("Face")
        self.secondary_object_detection.addItem("Body")
        secondary_object_grid = QtGui.QGridLayout()
        secondary_object_grid.addWidget(self.secondary_object_detection, 0, 0)
        self.box_secondary_object.setLayout(secondary_object_grid)


        self.box_battery = QtGui.QGroupBox("Battery")
        label_battery = QtGui.QLabel("Percents:")
        self.battery_status = QtGui.QLabel()
        battery_status_grid = QtGui.QGridLayout()
        battery_status_grid.addWidget(label_battery, 0, 0)
        battery_status_grid.addWidget(self.battery_status, 0, 1)
        self.box_battery.setLayout(battery_status_grid)


        self.statusLayout = QtGui.QGroupBox("DRONE")
        self.layout_window = QtGui.QGridLayout()
        self.layout_window.addWidget(self.box_target_main, 0, 0, 1,2)
        self.layout_window.addWidget(self.box_secondary_object, 1,0)
        self.layout_window.addWidget(self.box_battery, 1,1)
        self.statusLayout.setLayout(self.layout_window)


        box_drone = QtGui.QGroupBox("Status Drone")
        label_status_drone = QtGui.QLabel("status:")
        self.status_drone = QtGui.QLabel()  # connect or landed,flying,ect
        status_drone_grid = QtGui.QGridLayout()
        status_drone_grid.addWidget(label_status_drone, 0, 0)
        status_drone_grid.addWidget(self.status_drone, 0, 1)
        box_drone.setLayout(status_drone_grid)

        layout_window_container = QtGui.QGridLayout()
        layout_window_container.addWidget(self.statusLayout, 0, 0)
        layout_window_container.addWidget(box_drone, 1, 0)

        self.setLayout(layout_window_container)

