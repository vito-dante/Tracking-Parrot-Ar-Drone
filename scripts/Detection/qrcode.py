import cv2
import zbar
from PIL import Image
from .figure import FigureStatus

class qrCode(FigureStatus):
    def __init__(self):
        super(qrCode, self).__init__()
        self.scanner = zbar.ImageScanner()
        self.scanner.parse_config('enable')
        self.myName = "Vito Marca Vilte"

    def findObject(self, image):
        image_copy = image
        gray = cv2.cvtColor(image_copy, cv2.COLOR_BGR2GRAY, dstCn=0)
        pil = Image.fromarray(gray)
        width, height = pil.size
        raw = pil.tobytes()
        pil = zbar.Image(width, height, 'Y800', raw)
        self.scanner.scan(pil)
        for symbol in pil:
            if symbol.data==self.myName:
                points = symbol.location
                #Draw square detection RED
                for i in xrange(3):
                    cv2.line(image, points[i], points[i+1], (255, 0, 0), 5)
                cv2.line(image, points[3], points[0], (255, 0, 0), 5)
                a = points[2][0]
                c = points[0][0]
                d = points[2][1]
                e = points[0][1]
                pointLocationX = ((a - c )/2.0) + c
                pointLocationY = ((d - e)/2.0) + e
                #TODO SIZE relative change to size fixed
                size = a - c

                self.actualizarSituacion(pointLocationX, pointLocationY, size)
                return image
        self.actualizarSituacion(-1,-1,-1)
        return image
