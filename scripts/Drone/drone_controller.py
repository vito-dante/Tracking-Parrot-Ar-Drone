
# API de ROS for python
import rospy
# Type data used to receive messages on data states drone
from ardrone_autonomy.msg import Navdata
# Type of data used to send messages to drone movement
from geometry_msgs.msg import Twist
# Type of data used to send messages to land
from std_msgs.msg import Empty

from .drone_status import DroneStatus

DroneStatus = DroneStatus()

# Time in milliseconds that determine a frequency in the command will be sent
COMMAND_PERIOD = 40

class DroneController(object):

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


    # Function called when new data were received they drone
    def receive_navdata(self, navdata):
        self.status = navdata.state

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