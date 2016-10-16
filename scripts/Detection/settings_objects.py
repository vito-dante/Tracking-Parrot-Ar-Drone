# -*- coding: utf-8 -*-
# ==================== SCREEN SETTINGS ======================
# Las variables utilizadas para establecer el tamano de los margenes que se utilizaran para detectar el movimiento
# Un movimiento hacia la izquierda, por ejemplo, se activara cuando la esfera este cerca del margen izquierdo de manera similar, identificaremos movimientos a la izquierda, arriba y abajo
# El tamano del margen es una relacion (PORC_MARGEN) tamano de la imagen (verticalmente, Y, y horizontalmente, X)

MAX_X = 640  # Max x value(tamano de la imagen capturada por la camara Horizontal del ARDrone 2.0)
MAX_Y = 360  # Max y value (tamano de la imagen capturada por la camara Horizontal del ARDrone 2.0

# Guarda los fotogramas de la imagen en el ruta ~/.ros
GUARDAR_IMAGEN = False  # Coloque en False si no quieres que guarde las imagenes en disco(para depuracion )
# ==================== END SCREEN SETTINGS ======================
