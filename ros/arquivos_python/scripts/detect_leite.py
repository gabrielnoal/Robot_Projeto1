#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import tf
import math
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import detect_feature

import cv2
import numpy as np
from matplotlib import pyplot as plt
import time
import matplotlib.cm as cm
from math import pi
import sys
import math

bridge = CvBridge()
cv_image = None
imagem_leite = None
good_matches = []
atraso = 3E9 # 1 segundo e meio. Em nanossegundos
check_delay = True # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. Descarta imagens que chegam atrasadas demais

def roda_todo_frame(imagem): 
	print("frame")
	global cv_image
	global good_matches
	global imagem_leite

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.nsecs
	print("delay ", "{:.3f}".format(delay/1.0E9))
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame:", delay)
		return
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		good_matches =  detect_feature.matches(cv_image, imagem_leite)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)

if __name__=="__main__":

    rospy.init_node("detect_leite")

    imagem_leite = 'leite.jpg'
    imagem_leite = cv2.imread(imagem_leite,0)
    imagem_leite = cv2.cvtColor(imagem_leite, cv2.COLOR_BGR2GRAY)  


    # Para usar a Raspberry Pi
    topico_raspberry_camera = "/raspicam_node/image/compressed"
    # Para usar a webcam
    topico_webcam = "/cv_camera/image_raw/compressed"
    topico_imagem = topico_raspberry_camera
    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24) 
    print("Usando ", topico_imagem)
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)


    try:

        while not rospy.is_shutdown():
            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            if good_matches > 20:
		print(good_matches)
	        vel = Twist(Vector3(0.3,0,0), Vector3(0,0,0))

            else:   # Vira a esquerda
	        vel = Twist(Vector3(0,0,0), Vector3(0,0,0.3))

            velocidade_saida.publish(vel)
            rospy.sleep(0.01)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
