
# API de ROS for python
import rospy
# Type data used to receive messages on data states drone
from ardrone_autonomy.msg import Navdata
# Type of data used to send messages to drone movement
from geometry_msgs.msg import Twist
# Type of data used to send messages to land
from std_msgs.msg import Empty
#animation for the drone
from ardrone_autonomy.srv import FlightAnim,LedAnim
# Message parameter for changing camera
from std_srvs.srv import Empty as empty_service

from .drone_status import DroneStatus

DroneStatus = DroneStatus()

# Time in milliseconds that determine a frequency in the command will be sent
COMMAND_PERIOD = 40

class DroneCommands(object):

    StatusMessages = {
        DroneStatus.emergency: 'Emergencia',
        DroneStatus.inited: 'Inciado',
        DroneStatus.landed: 'Aterrizado',
        DroneStatus.flying: 'Volando',
        DroneStatus.hovering: 'Hover',
        DroneStatus.test: 'Es un Test ?',
        DroneStatus.taking_off: 'Despegando',
        DroneStatus.go_to_hover: 'entrando en modo Hover',
        DroneStatus.landing: 'Aterrizando',
        DroneStatus.looping: 'Realizando un Loop ?'
    }

    UnknownMessage = 'Estado Desconocido'



    def __init__(self):
        # Current status drone
        self.status = -1

        # subscription for receive data navigation from drone
        self.subNavdata = rospy.Subscriber(
            '/ardrone/navdata', Navdata, self.receive_navdata)

        # publication to send the order to land the drone
        self.pubLand = rospy.Publisher(
            '/ardrone/land', Empty, queue_size=30)
        self.pubTakeoff = rospy.Publisher(
            '/ardrone/takeoff', Empty, queue_size=30)
        self.pubReset = rospy.Publisher(
            '/ardrone/reset', Empty, queue_size=30)

        # Publication. to send specific commands to the drone(roll,pitch,yaw)
        self.pubCommand = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Initializes the type to be used (twist) to publish motion commands to the drone
        self.command = Twist()

        # timer, to send a command and then another and another .....
        self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD / 1000.0),
                                        self.send_command)
        # Lands drone if it receives a command to terminate the program
        rospy.on_shutdown(self.send_land)

    # object status
    # self.statusMessage = self.MessageSituacion[self.objectTarget.estado]

    # Function called when new data were received they drone
    def receive_navdata(self, navdata):

        # Indicates that there was communication (since data on the drone arrived)
        #  through the port 5556
        self.communicationSinceTimer = True

        self.status = navdata.state
        self.battery = navdata.batteryPercent

        # For show message
        # updates the status of the drone in the window
        self.status_msg = self.StatusMessages[self.status] \
            if self.status in self.StatusMessages else self.UnknownMessage

    def show_status_message(self):
        return self.status_msg

    def show_battery(self):
        return str(int(self.battery))

    def send_land(self):
        if (self.status == DroneStatus.flying or self.status == DroneStatus.hovering or
                    self.status == DroneStatus.go_to_hover):
            self.pubLand.publish(Empty())
            return True
        return False

    def send_takeoff(self):
        if (self.status == DroneStatus.landed):
            self.pubTakeoff.publish(Empty())
            return True
        return False

    def takeoff_land_toggle(self):
        if not self.send_takeoff():
            self.send_land()
        return True

    # Sends a signal to switch to emergency mode
    def send_emergency(self):
        self.pubReset.publish(Empty())

    # set values
    def set_command(self, roll=0, pitch=0, yaw_velocity=0, z_velocity=0):
        #Movement for left and right
        self.command.linear.y = roll
        #Movement for the front and back
        self.command.linear.x = pitch
        #Rotational movement around its own axis
        self.command.angular.z = yaw_velocity
        #Up and down
        self.command.linear.z = z_velocity

    def send_command(self, event):
        if self.status == DroneStatus.flying or self.status == DroneStatus.hovering:
            self.pubCommand.publish(self.command)


    # service change camera and flat trim receive same parameter
    def change_camera(self):
        rospy.wait_for_service("/ardrone/togglecam")
        try:
            proxyDrone = rospy.ServiceProxy("/ardrone/togglecam", empty_service)
            proxyDrone()
        except rospy.ServiceException as e:
            print("Failed services %s"%(e))

    def flatTrim(self):
        rospy.wait_for_service("/ardrone/flattrim")
        try:
            proxyDrone = rospy.ServiceProxy("/ardrone/flattrim", empty_service)
            proxyDrone()
        except rospy.ServiceException as e:
            print("Failed services %s"%(e))

    def led_animation(self, typeofAnimation=1,
                      frequency=4, duration=5, ):
        rospy.wait_for_service("/ardrone/setledanimation")
        try:
            proxyDrone = rospy.ServiceProxy("/ardrone/setledanimation", LedAnim)
            proxyDrone(typeofAnimation, frequency, duration)
        except rospy.ServiceException as e:
            print("Failed services %s"%(e))

    def flip_animation(self, typeofAnimation=1, duration=0,):
        rospy.wait_for_service("/ardrone/setflightanimation")
        try:
            proxyDrone = rospy.ServiceProxy("/ardrone/setflightanimation", FlightAnim)
            proxyDrone(typeofAnimation, duration)
        except rospy.ServiceException as e:
            print("Failed services %s"%(e))