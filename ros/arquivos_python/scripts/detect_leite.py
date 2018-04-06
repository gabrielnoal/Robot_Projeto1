import rospy
import tf
import math
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cormodule

import cv2
import numpy as np
from matplotlib import pyplot as plt
import time
import matplotlib.cm as cm
from math import pi
from __future__ import print_function
import sys
import math


media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
area = 0.0 # Variavel com a area do maior contorno
check_delay = False # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. Descarta imagens que chegam atrasadas demais


imagem = 'leite.jpg'
leite = cv2.imread(imagem, cv2.IMREAD_GRAYSCALE)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


def roda_todo_frame(imagem):  ### DESCOBRIR SE ESSA FUNÇÃO É NECESSÁRIA
	print("frame")
	global cv_image
	global media
	global centro

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
		media, centro, area =  cormodule.identifica_cor(cv_image)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)

if __name__=="__main__":
    '''
    rospy.init_node(" ")
    '''

    # Para usar a Raspberry Pi
	topico_raspberry_camera = "/raspicam_node/image/compressed"
	# Para usar a webcam 
	topico_webcam = "/cv_camera/image_raw/compressed"


    topico_imagem = topico_raspberry_camera

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24) #ONDE USAR?#

	print("Usando ", topico_imagem)
    
    # Capture frame-by-frame
    ret, frame = cap.read()  ### DESCOBRIR O QUE É O RECEBEDOR E SE É POSSÍVEL MUDAR O CÓDIGO AQUI ###

    
    MIN_MATCH_COUNT = 20
    img1 = cv2.imread(imagem,0)          # Imagem a procurar
    img2 = frame # Imagem do cenario - puxe do video para fazer isto

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


    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    try:
		while not rospy.is_shutdown():
            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            if len(good)>MIN_MATCH_COUNT:
				vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
            velocidade_saida.publish(vel)
			rospy.sleep(0.01)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")
        