
class DroneStatus(object):

     """this class present first property after setter"""

     _instance = None

     def __new__(class_, *args, **kwargs):
         if not isinstance(class_._instance, class_):
             class_._instance = object.__new__(class_, *args, **kwargs)
         return class_._instance

     def __init__(self):
         self.__emergency = 0
         self.__inited = 1
         self.__landed = 2
         self.__flying = 3
         self.__hovering = 4
         self.__test = 5
         self.__taking_off = 6
         self.__go_to_hover = 7
         self.__landing = 8
         self.__looping = 9

    # ================ PROPERTY ALL ATTRIB===========
     @property
     def emergency(self):
         """get the current status emergency"""
         return self.__emergency

     @property
     def inited(self):
         """get the current status inited"""
         return self.__inited

     @property
     def landed(self):
         """get the current status landed"""
         return self.__landed

     @property
     def flying(self):
         """get the current status flying"""
         return self.__flying

     @property
     def hovering(self):
         """get the current status hovering"""
         return self.__hovering

     @property
     def test(self):
         """get the current status test"""
         return self.__test

     @property
     def taking_off(self):
         """get the current status taking_off"""
         return self.__taking_off

     @property
     def go_to_hover(self):
         """get the current status go_to_hover"""
         return self.__go_to_hover

     @property
     def landing(self):
         """get the current status landing"""
         return self.__landing

     @property
     def looping(self):
         """get the current status looping"""
         return self.__looping

