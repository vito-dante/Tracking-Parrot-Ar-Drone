
# Importa ROS para python y para ardrone_autonomy(que hace un puente con el drone)
import rospy
# Tipo de datos usados para recibir mensajes sobre los estados del drone
from ardrone_autonomy.msg import Navdata
# Tipo de datos usados para enviar mensajes de movimiento al drone
from geometry_msgs.msg import Twist
# Tipo de datos usados para enviar mensajes para aterrizar
# y despegar(no precisan parametros)
from std_msgs.msg import Empty

from .drone_status import DroneStatus

DroneStatus = DroneStatus()

# Tiempo en milisegundos que determinan una frequencia en los comando seran enviados
COMMAND_PERIOD = 40
# Coloque un True para evitar que los comandos de vuelo sean pesados para el drone.
# usado para probar los algoritmos de vision computacional sin riesgos de dejar el drone loco

# Indica que los comandos no deben ser efectivamente enviandos
#  al drone(usados en True para depuerar el modulo) de vision
# computacional, por tanto en True continua recibiendo las imagenes).
# los comandos por el teclado continuan funcionando sin embargo.
SOLO_CAPTURA_IMAGENS = False

# clase que especifica el controlador del drone. objetos
#  de esta clase son usados para pasar comandos al drone y recibir datos del drone
class DroneController(object):

    # inicializa algunas variables y tambien algunos "suscriptores"(leer mensajes)
    # publicadores" (publicar mensajes).
    def __init__(self):
        # Estado actual del drone
        self.status = -1

        # Suscribe al topico donde son publicada los mensajes con informaciones
        # sobre la situacion del drone. Usa una funcion para ReceiveNavdata
        # como callback (ME SUSCRIBO Y POR MEDIO DEL CALLBACK RECIBO LOS DATOS)
        # es asincrono
        self.subNavdata = rospy.Subscriber(
            '/ardrone/navdata', Navdata, self.ReceiveNavdata)

        # avisa que el DroneController va publicar mensajes para despegar,
        # aterrizar y resetear(crear "publicaciones" para eso)
        self.pubLand = rospy.Publisher(
            '/ardrone/land', Empty, queue_size=30)
        self.pubTakeoff = rospy.Publisher(
            '/ardrone/takeoff', Empty, queue_size=30)
        self.pubReset = rospy.Publisher(
            '/ardrone/reset', Empty, queue_size=30)

        # avisa que seran publicadas mensajes para controlar la posicion
        # y otras cosas del drone. cmd_vel es un topico de ROS
        # que puede ser usado con otros tipos de robos y
        # por esto no tiene un "/ardrone" antes.
        self.pubCommand = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Inicializa el tipo a ser usado (twist) para publicar
        # los comandos de movimiento para el drone
        self.command = Twist()

        # Determina que los comandos para el drone deben ser enviados
        # regularmente en la frequencia
        # indicada para la variable  COMMAND_PERIOD
        # (convierte milisegundos en segundos por que
        # es lo que espera rospy.Timer). Pasa una funcion
        # SendCommand como callback(funcion que debe ser llamada
        # toda vez que el tiempo del temporizador fuese alcanzado)
        self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD / 1000.0),
                                        self.SendCommand)

        # Aterriza el drone si recibe un comando para terminar el programa
        rospy.on_shutdown(self.SendLand)


    # Funcion llamada cuando nuevos datos del drone fuesen recibidos
    def ReceiveNavdata(self, navdata):
        self.status = navdata.state

    # Envia una senal para despegar(antes prueba para ver si esta aterrizando)
    def SendTakeoff(self):
        if not SOLO_CAPTURA_IMAGENS:
            if(self.status == DroneStatus.landed):
                self.pubTakeoff.publish(Empty())

    # Envia una senal para aterrizar
    def SendLand(self):
        if(self.status != DroneStatus.landed):
            self.pubLand.publish(Empty())

    # Envia senal para reinicializar el drone (apaga los motores si estuviese volando)
    def SendEmergency(self):
            self.pubReset.publish(Empty())

    # actualiza los parametros que seran utilizados en el proximo comando de movimiento del drone
    def SetCommand(self, roll=0, pitch=0, yaw_velocity=0, z_velocity=0):
        self.command.linear.x = pitch  # Movimiento para el frente y para atras
        self.command.linear.y = roll  # Movimiento para los lados
        self.command.linear.z = z_velocity  # Para arriba para abajo
        self.command.angular.z = yaw_velocity  # Movimiento de rotacion en
        # torno a su propio eje

    # Envia un comando de movimiento con los parametros actuales si
    # el drone estuviese volando
    def SendCommand(self, event):

        if not SOLO_CAPTURA_IMAGENS:
            if self.status == DroneStatus.flying or self.status == DroneStatus.hovering:
                self.pubCommand.publish(self.command)


