
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
            '/ardrone/navdata', Navdata, self.ReceiveNavdata)

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
                                        self.SendCommand)
        # Lands drone if it receives a command to terminate the program
        rospy.on_shutdown(self.SendLand)


    # Function called when new data were received they drone
    def ReceiveNavdata(self, navdata):
        self.status = navdata.state

    # Send a signal to take off
    def SendTakeoff(self):
        #(before test if the drone is not in the air)
        if(self.status == DroneStatus.landed):
            self.pubTakeoff.publish(Empty())

    # Send a signal to land
    def SendLand(self):
        if(self.status != DroneStatus.landed):
            self.pubLand.publish(Empty())

    # Sends a signal to switch to emergency mode
    def SendEmergency(self):
            self.pubReset.publish(Empty())

    # set values
    def SetCommand(self, roll=0, pitch=0, yaw_velocity=0, z_velocity=0):
        #Movement for the front and back
        self.command.linear.x = pitch
        #Movement for left and right
        self.command.linear.y = roll
        #Up and down
        self.command.linear.z = z_velocity
        #Rotational movement around its own axis
        self.command.angular.z = yaw_velocity

    def SendCommand(self, event):
        if self.status == DroneStatus.flying or self.status == DroneStatus.hovering:
            self.pubCommand.publish(self.command)


