# -*- coding:utf-8 -*-

import cv2
import numpy as np
from matplotlib import pyplot as plt
import time

'''Imports for ros'''
import rospy
import tf
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros


def identifica_cor(frame, cor_menor, cor_maior):
    '''
    Segmenta o maior objeto cuja cor é parecida com cor_h (HUE da cor, no espaço HSV).
    '''

    # No OpenCV, o canal H vai de 0 até 179, logo cores similares ao
    # vermelho puro (H=0) estão entre H=-8 e H=8.
    # Veja se este intervalo de cores está bom
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    frame = cv2.blur(frame,(5,5)) # Tira ruido

    #cor_menor = np.array([80, int(0.5*255), int(0.2*255)])
    #cor_maior = np.array([120, 255, 255])
    segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)

    # A operação MORPH_CLOSE fecha todos os buracos na máscara menores
	# que um quadrado 1x1. É muito útil para juntar vários
	# pequenos contornos muito próximos em um só.
    kernel = np.ones((1, 1))
    segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,kernel)
    # segmentado_cor = cv2.morphologyEx(segmentado_cor, cv2.MORPH_OPEN, kernel)

    # Encontramos os contornos na máscara e selecionamos o de maior área
    img_out, contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    cor = None
    maior_contorno = None
    maior_contorno_area = 0

    for cnt in contornos:
	    area = cv2.contourArea(cnt)
	    if area > maior_contorno_area:
	        maior_contorno = cnt
	        maior_contorno_area = area


    # Encontramos o centro do contorno fazendo a média de todos seus pontos.
    if not maior_contorno is None : #Para não dar erro caso não ache o maior_contorno
        # print(maior_contorno)
        maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
        media = maior_contorno.mean(axis=0)
        media = media.astype(np.int32)
        # print(media) #Tupla onde o primeiro inteiro é a posição media na
                     #horizontal e o segunodo é na vertical
                     #1o varia de 0(esquerda) a 1000(direita)
        cor = "Azul"
        cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 255], 5)
        cv2.circle(frame, tuple(media), 5, [0, 255, 0])

    else:
        media = (0, 0)

    if not maior_contorno is None:
        cv2.putText(frame, "Area:{:0.1f}".format(maior_contorno_area), (10,300),cv2.FONT_HERSHEY_SIMPLEX,1.5,color=(0,0,0))
        cv2.putText(frame, "Horizontal: {0}".format(media[0]), (10,350),cv2.FONT_HERSHEY_SIMPLEX,1.5,color=(0,0,0))
        cv2.putText(frame, "Vertical: {0}".format(media[1]), (10,400),cv2.FONT_HERSHEY_SIMPLEX,1.5,color=(0,0,0))

    # cv2.imshow('', frame)
    #cv2.imshow('imagem in_range', segmentado_cor)
    cv2.waitKey(1)

    centro = (frame.shape[0]//2, frame.shape[1]//2)

    return media, centro, maior_contorno_area, cor
