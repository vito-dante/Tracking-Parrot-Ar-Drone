from PySide import QtGui

class WindowControl(QtGui.QWidget):
    # title window
    def __init__(self, window):
        super(WindowControl, self).__init__(window)

        box_target_main = QtGui.QGroupBox("Target Main")

        label_target = QtGui.QLabel("Object:")
        self.object_detection_main = QtGui.QComboBox()
        self.object_detection_main.addItem("Ball")
        self.object_detection_main.addItem("qrCode")
        self.object_detection_main.addItem("Face")
        self.object_detection_main.addItem("Body")

        box_status_object = QtGui.QGroupBox("Status Object")

        label_status_object = QtGui.QLabel("status:")
        self.status_objects_detection = QtGui.QLabel()

        box_drone = QtGui.QGroupBox("Status Drone")

        label_status_drone = QtGui.QLabel("status:")
        self.status_drone = QtGui.QLabel()

        box_battery = QtGui.QGroupBox("Status Drone")

        label_battery = QtGui.QLabel("Battery:")
        self.battery_status = QtGui.QLabel()

        box_secondary_object = QtGui.QGroupBox("Secondary object")

        label_object = QtGui.QLabel("Object:")
        self.secondary_object_detection = QtGui.QComboBox()
        self.secondary_object_detection.addItem("Ball")
        self.secondary_object_detection.addItem("qrCode")
        self.secondary_object_detection.addItem("Face")
        self.secondary_object_detection.addItem("Body")

        target_main_grid = QtGui.QGridLayout()
        target_main_grid.addWidget(label_target, 0, 0)
        target_main_grid.addWidget(self.object_detection_main, 0, 1)
        box_target_main.setLayout(target_main_grid)

        status_object_grid = QtGui.QGridLayout()
        status_object_grid.addWidget(label_status_object, 0, 0)
        status_object_grid.addWidget(self.status_objects_detection, 0, 1)
        box_status_object.setLayout(status_object_grid)

        status_drone_grid = QtGui.QGridLayout()
        status_drone_grid.addWidget(label_status_drone, 0, 0)
        status_drone_grid.addWidget(self.status_drone, 0, 1)
        box_drone.setLayout(status_drone_grid)

        battery_status_grid = QtGui.QGridLayout()
        battery_status_grid.addWidget(label_battery, 0, 0)
        battery_status_grid.addWidget(self.battery_status, 0, 1)
        box_battery.setLayout(battery_status_grid)

        secondary_object_grid = QtGui.QGridLayout()
        secondary_object_grid.addWidget(label_object, 0, 0)
        secondary_object_grid.addWidget(self.secondary_object_detection, 0, 1)
        box_secondary_object.setLayout(secondary_object_grid)

        layout_window = QtGui.QGridLayout()
        layout_window.addWidget(box_target_main, 0, 0)
        layout_window.addWidget(box_status_object, 1, 0)
        layout_window.addWidget(box_secondary_object, 0, 1)
        layout_window.addWidget(box_drone, 1, 1)
        layout_window.addWidget(box_battery, 2, 0)
        self.setLayout(layout_window)