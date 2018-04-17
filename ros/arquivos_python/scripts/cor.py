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
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cor_capa
import detect_feature


bridge = CvBridge()
cv_image = None

media = []
centro = []
atraso = 1E9
cor = None
cor_menor = np.array([80, int(0.5*255), int(0.2*255)])
cor_maior = np.array([120, 255, 255])

#imagem_leite = 'leite.png' #Png está em menor resolução que a jpg
imagem_leite = cv2.imread("leite.png",0)
# Initiate SIFT detector
sift = cv2.xfeatures2d.SIFT_create()

kp1, des1 = sift.detectAndCompute(imagem_leite,None)
good_matches = []

atencao = 0.3
vai_bater = False
lista_valores = []
distancia = None
area = 0.0 # Variavel com a area do maior contorno

check_delay = True # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. Descarta imagens que chegam atrasadas demais



def roda_todo_frame(imagem):
	# print("frame")
	global cv_image
	global media
	global centro
	global cor
	global good_matches

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.nsecs
	#print("delay ", "{:.3f}".format(delay/1.0E9))
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame:", delay)
		return
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		cv_image2 = cv_image.copy()
		good_matches = detect_feature.matches(cv_image2, kp1, des1, sift)
		media, centro, area = cor_capa.identifica_cor(cv_image,cor_menor, cor_maior)
		depois = time.clock()
		#cv2.putText(cv_image, "Area: {0}".format(area), (10,300),cv2.FONT_HERSHEY_SIMPLEX,1.5,color=(255,255,255))
		#cv2.imshow("Camera_feature", cv_image2)
		cv2.circle(cv_image, tuple(media), 5, [0, 255, 0])
		cv2.imshow("Camera_Cor", cv_image)

	except CvBridgeError as e:
		print('ex', e)

def scaneou(dado):
	global vai_bater
	global lista_valores
	global distancia
	#print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	lista_valores = np.array(dado.ranges).round(decimals = 2)
	iteracoes = 0
	
	for i in range(len(lista_valores)):
		if lista_valores[i] == 0:
			lista_valores[i] = 33
		else:
			iteracoes += 1

	distancia = min(lista_valores)
	#print("Distancia do Objeto mais Proximo:", distancia)
	if distancia < atencao:
		vai_bater = True
		return
		
	else:
		#print("OK")
		iteracoes += 1
		vai_bater = False
		return

if __name__=="__main__":
	
	
	rospy.init_node("cor")

	# Para usar a Raspberry Pi
	topico_raspberry_camera = "/raspicam_node/image/compressed"
	# Para usar a webcam
	topico_webcam = "/cv_camera/image_raw/compressed"


	topico_imagem = topico_raspberry_camera

	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

	print("Usando ", topico_imagem)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	try:

		while not rospy.is_shutdown():
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))			
			if vai_bater == True:
				for i in range(len(lista_valores)):
					if 0 < i < 90 and lista_valores[i] == distancia:
						vel = Twist(Vector3(-2, 0, 0), Vector3(0, 0, -2))
						print("VAI BATER - NW")

					elif 90 < i < 180 and lista_valores[i] == distancia:
						vel = Twist(Vector3(2, 0, 0), Vector3(0, 0, -2))
						print("VAI BATER - SW")

					elif 180 < i < 270 and lista_valores[i] == distancia:
						vel = Twist(Vector3(2, 0, 0), Vector3(0, 0, 2))
						print("VAI BATER - SE")

					elif 270 < i < 360 and lista_valores[i] == distancia:
						vel = Twist(Vector3(-2, 0, 0), Vector3(0, 0, 2))
						print("VAI BATER - NE")
			else:
				if len(good_matches) > 30:
					print("Achei")
					vel = Twist(Vector3(-0.2,0,0), Vector3(0,0,0))
				elif len(media) != 0 and len(centro) != 0:
					dif_x = media[0]-centro[0]
					dif_y = media[1]-centro[1]
					#cv2.putText(cv_image, "dif_x: {0}".format(dif_x), (10,300),cv2.FONT_HERSHEY_SIMPLEX,1.5,color=(0,0,0))
					#cv2.putTex
					# t(cv_image, "dif_y: {0}".format(dif_y), (10,300),cv2.FONT_HERSHEY_SIMPLEX,1.5,color=(255,255,255))
					#print(dif_x)
					if math.fabs(dif_x)<50: # Se a media estiver muito proxima do centro anda para frente
						vel = Twist(Vector3(0.2,0,0), Vector3(0,0,0))
					
					else:
						if dif_x > 0: # Vira a direita
							vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
						else: # Vira a esquerda
							vel = Twist(Vector3(0,0,0), Vector3(0,0,0.2))

			velocidade_saida.publish(vel)
			rospy.sleep(0.01)

	except rospy.ROSInterruptException:
		print("Ocorreu uma exceção com o rospy")
