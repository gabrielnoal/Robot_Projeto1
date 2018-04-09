#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


def scaneou(dado):
	print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	print("Leituras:")
	iteracoes = 0
	distancia = dado.range_min
	atencao = 0.15
	for i in len(dado.ranges):
		if dado.ranges[i] == dado.range_min:
			if dado.ranges[i] != 0 and 0 < i < 90:
				velocidade = Twist(Vector3(0, 0, 2), Vector3(0, 0, 2))
				velocidade_saida.publish(velocidade)

			elif dado.ranges[i] != 0 and 90 < i < 180:
				velocidade = Twist(Vector3(0, 0, 2), Vector3(0, 0, 2))
				velocidade_saida.publish(velocidade)

			elif dado.ranges[i] != 0 and 180 < i < 270:
				velocidade = Twist(Vector3(0, 0, 2), Vector3(0, 0, 2))
				velocidade_saida.publish(velocidade)

			elif dado.ranges[i] != 0 and 270 < i < 360:
				velocidade = Twist(Vector3(0, 0, 2), Vector3(0, 0, 2))
				velocidade_saida.publish(velocidade)

			else:
				iteracoes += 1
		print (np.array(dado.ranges).round(decimals=3))
#		else:
#			iteracoes += 1
		# elif 0 < i < 90:
		# 	if 0 < dado.ranges[i] < atencao:
		# elif 90 < i < 180:
		# 	if 0 < dado.ranges[i] < atencao:
		# 		velocidade = Twist(Vector3(0, 0, 2), Vector3(0, 0, 2))
		# 		velocidade_saida.publish(velocidade)
		# elif 180 < i < 270:
		# 	if 0 < dado.ranges[i] < atencao:
		# 		velocidade = Twist(Vector3(0, 0, 2), Vector3(0, 0, 2))
		# 		velocidade_saida.publish(velocidade)
		# elif 270 < i < 360:
		# 	if 0 < dado.ranges[i] < atencao:
		# 		velocidade = Twist(Vector3(0, 0, 2), Vector3(0, 0, 2))
		# 		velocidade_saida.publish(velocidade)
		# else:
		# 	iteracoes += 1
#	print(np.array(dado.ranges).round(decimals=3))




#			velocidade = Twist(Vector3(0, 0, 2), Vector3(0, 0, 2))
#			velocidade_saida.publish(velocidade)
#	else:
#		iteracoes += 1

if __name__=="__main__":

	rospy.init_node("le_scan")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)



	while not rospy.is_shutdown():
		#print("Oeee")
		velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(velocidade)
		rospy.sleep(2)
