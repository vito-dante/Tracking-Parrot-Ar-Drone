

class ObjectStatus(object):
     '''
        all status of the drone is represent here
     '''
     _instance = None

     def __new__(class_, *args, **kwargs):
        if not isinstance(class_._instance, class_):
            class_._instance = object.__new__(class_, *args, **kwargs)
        return class_._instance

     def __init__(self):
        self.__appeared = 0
        self.__diapered = 1
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
         return self.__diapered

     @property
     def moved_left(self):
         return self.__movedLeft

     @property
     def moved_right(self):
         return self.__movedRight

     @property
     def moved_up(self):
        return self.__movedUp

     @property
     def moved_down(self):
        return self.__movedDown

     @property
     def moved_front(self):
        return self.__movedFront

     @property
     def moved_back(self):
        return self.__movedBack

     @property
     def same_place(self):
        return self.__samePlace

