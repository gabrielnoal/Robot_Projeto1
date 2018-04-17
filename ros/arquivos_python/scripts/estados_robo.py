#! /usr/bin/env python
# -*- coding:utf-8 -*-

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
import smach
import smach_ros

import cv2
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.cm as cm
from math import pi
import sys
import math
from sensor_msgs.msg import LaserScan

import cor_capa
import detect_feature
import le_scan

bridge = CvBridge()
cv_image = None
media = []
centro = []
imagem_leite = None
good_matches = []
atraso = 1E9 # 1 segundo e meio. Em nanossegundos
area = 0.0 # Variavel com a area do maior contorno
check_delay = True # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. Descarta imagens que chegam atrasadas demais


def roda_todo_frame(imagem):
	global cv_image
	global good_matches
	global imagem_leite
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
		good_matches =  detect_feature.matches(cv_image, kp1, des1, sift)
        media, centro, area =  cor_capa.identifica_cor(cv_image,False)
        Perigo = le_scan.scaneou(dado)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)
## Classes - estados
class Searching(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['achou_leite', 'achou_breja', 'achou_obstaculo', 'procurando'])
	def execute(self, userdata):
        if len(media) != 0 and len(centro) != 0:
			return 'achou_breja'
            
        else if len(good_matches) > 30:
            return 'achou_leite'

        else if Perigo = True:
            return 'achou_obstaculo'

		else:
	        vel = Twist(Vector3(0,0,0), Vector3(0,0,0.3))
			velocidade_saida.publish(vel)
			return 'procurando'

class Retreat(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fugir', 'procurar', 'achou_obstaculo', 'seguir'])
    def execute(self, userdata):
        if len(good_matches) > 30:
            vel = Twist(Vector3(-0.3,0,0), Vector3(0,0,0))
            velocidade_saida.publish(vel)
            rospy.sleep(0.01)
            return 'fugir'

        else if Perigo = True:
            return 'achou_obstaculo'

        else if len(media) != 0 and len(centro) != 0: 
            return 'seguir'

        else:
            return 'procurar'

class Follow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['seguir', 'procurar', 'achou_obstaculo', 'fugir'])
    def execute(self, userdata):
        if len(media) != 0 and len(centro) != 0:
            dif_x = media[0]-centro[0]
            dif_y = media[1]-centro[1]
            cv2.putText(cv_image, "dif_x: {0}".format(dif_x), (10,200),cv2.FONT_HERSHEY_SIMPLEX,1.5,color=(255,255,255))
            cv2.putText(cv_image, "dif_y: {0}".format(dif_y), (10,300),cv2.FONT_HERSHEY_SIMPLEX,1.5,color=(255,255,255))

            if math.fabs(dif_x)<40: # Se a media estiver muito proxima do centro anda para tras
                vel = Twist(Vector3(-0.3,0,0), Vector3(0,0,0))
                velocidade_saida.publish(vel)
                rospy.sleep(0.01)
                return 'seguir'
            #PENSAR EM DIFS ACEITÁVEIS PARA ELE CONTINUAR SEGUINDO O OBJETO
            
        else:
            return 'procurar'

class Survive(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['sobreviver', 'procurar'])
    def execute(self, userdata):
        lista_valores = np.array(dado.ranges).round(decimals = 2)
        iteracoes = 0
        atencao = 0.3

        for i in range(len(lista_valores)):
            if lista_valores[i] == 0:
                lista_valores[i] = 33
            else:
                iteracoes += 1

        velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        velocidade_saida.publish(velocidade)

        distancia = min(lista_valores)
        if distancia < atencao:
            for i in range(len(lista_valores)):
                if 0 < i < 90 and lista_valores[i] == distancia:
                    velocidade = Twist(Vector3(-2, 0, 0), Vector3(0, 0, -2))
                    return 'sobreviver'

                elif 90 < i < 180 and lista_valores[i] == distancia:
                    velocidade = Twist(Vector3(2, 0, 0), Vector3(0, 0, -2))
                    return 'sobreviver'

                elif 180 < i < 270 and lista_valores[i] == distancia:
                    velocidade = Twist(Vector3(2, 0, 0), Vector3(0, 0, 2))
                    return 'sobreviver'


                elif 270 < i < 360 and lista_valores[i] == distancia:
                    velocidade = Twist(Vector3(-2, 0, 0), Vector3(0, 0, 2))
                    return 'sobreviver'
                
                velocidade_saida.publish(velocidade)

        else:
            iteracoes += 1
            return 'procurar'




# main
def main():
	global velocidade_saida
	rospy.init_node('unico_estado')

	sift = cv2.xfeatures2d.SIFT_create()

    imagem_leite = 'leite.png'  #Png está em menor resolução que a jpg
    imagem_leite = cv2.imread(imagem_leite,0)

    kp1, des1 = sift.detectAndCompute(imagem_leite,None)

    # Para usar a Raspberry Pi
    topico_raspberry_camera = "/raspicam_node/image/compressed"
    # Para usar a webcam
    topico_webcam = "/cv_camera/image_raw/compressed"
    topico_imagem = topico_raspberry_camera
    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)
    print("Usando ", topico_imagem)
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	# Cria uma máquina de estados
	sm = smach.StateMachine(outcomes=['fim_geral'])

	# Preenche a Smach com os estados
	with sm:
	    smach.StateMachine.add('SEARCHING', Searching(),
	    	transitions={'procurando': 'SEARCHING',
	    	'achou_leite':'RETREAT', 'achou_breja': 'FOLLOW'
            'achou_obstaculo : SURVIVE'})
        smach.StateMachine.add('RETREAT', Retreat(), 
            transitions={'fugir': 'RETREAT',
            'procurar': 'SEARCHING'
            'achou_obstaculo: SURVIVE'})
        smach.StateMachine.add('FOLLOW', Follow(),
            transitions={'procurar': 'SEARCHING',
            'seguir': 'FOLLOW'
            'achou_obstaculo': 'SURVIVE')
        smach.StateMachine.add('SURVIVE', Survive(),
            transitions={'procurar' : 'SEARCHING',
            'sobreviver': 'SURVIVE'})

	# Executa a máquina de estados
	outcome = sm.execute()

	print("Execute finished")


if __name__ == '__main__':
	print("Main")
	main()