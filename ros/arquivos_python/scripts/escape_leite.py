#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import detect_feature


bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 3E9

area = 0.0 # Variavel com a area do maior contorno

check_delay = True # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. Descarta imagens que chegam atrasadas demais



def roda_todo_frame(imagem):
	# print("frame")
	global cv_image
	global media
	global centro
    global good_matches 

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
		media, centro, good_matches =  matches(cv_image)
		depois = time.clock()
		# cv2.putText(cv_image, "Distancia: {0}".format(distancia_cm), (10,300),cv2.FONT_HERSHEY_SIMPLEX,1.5,color=(255,255,255))
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)



if __name__=="__main__":

	rospy.init_node("escape_leite")

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
			if len(media) != 0 and len(centro) != 0 and len(good_matches)>20 :
				dif_x = media[0]-centro[0]
				dif_y = media[1]-centro[1]
				cv2.putText(cv_image, "dif_x: {0}".format(dif_x), (10,200),cv2.FONT_HERSHEY_SIMPLEX,1.5,color=(255,255,255))
				cv2.putText(cv_image, "dif_y: {0}".format(dif_y), (10,300),cv2.FONT_HERSHEY_SIMPLEX,1.5,color=(255,255,255))

				if math.fabs(dif_x)<40: # Se a media estiver muito proxima do centro anda para tras
					vel = Twist(Vector3(0.3,0,0), Vector3(0,0,0))

				else:
					if dif_x > 0: # Vira a direita
						vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.3))
					else: # Vira a esquerda
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0.3))

			velocidade_saida.publish(vel)
			rospy.sleep(0.01)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")
