
class ObjectStatus(object):

     """The situation of the object is described with the following
        attributes"""
     def __init__(self):
        self.__appeared = 0
        self.__disapared = 1
        self.__movedLeft = 2
        self.__movedRight = 3
        self.__movedUp = 4
        self.__movedDown = 5
        self.__movedFront = 6
        self.__movedBack =7
        self.__samePlace = 8

     @property
     def appeared(self):
         return self.__appeared

     @property
     def disapared(self):
         return self.__disapared

     @property
     def movedLeft(self):
         return self.__movedLeft

     @property
     def movedRight(self):
         return self.__movedRight

     @property
     def movedUp(self):
        return self.__movedUp

     @property
     def movedDown(self):
        return self.__movedDown

     @property
     def movedFront(self):
        return self.__movedFront

     @property
     def movedBack(self):
        return self.__movedBack

     @property
     def samePlace(self):
        return self.__samePlace

