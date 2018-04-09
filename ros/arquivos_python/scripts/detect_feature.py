#! /usr/bin/env python
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

import matplotlib.cm as cm
import math
from math import pi
from __future__ import print_function
import sys


def matches(frame):
    imagem = 'leite.jpg'

    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame = cv2.blur(frame,(5,5)) # Tira ruido

    centro = (frame.shape[0]//2, frame.shape[1]//2)



    MIN_MATCH_COUNT = 20
    img1 = cv2.imread(imagem,0)          # Imagem a procurar
    img2 = frame

    # Initiate SIFT detector
    sift = cv2.xfeatures2d.SIFT_create()

    # find the keypoints and descriptors with SIFT in each image
    kp1, des1 = sift.detectAndCompute(img1,None)
    kp2, des2 = sift.detectAndCompute(img2,None)

    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)

    # Configura o algoritmo de casamento de features
    flann = cv2.FlannBasedMatcher(index_params, search_params)

    # Tenta fazer a melhor comparacao usando o algoritmo
    matches = flann.knnMatch(des1,des2,k=2)

    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)

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
    else:
         media = (0, 0)

    centro = (frame.shape[0]//2, frame.shape[1]//2)


    return media, centro, good
