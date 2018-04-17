#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

def scaneou(dado):
	print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
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
	print("Distancia do Objeto mais Proximo:", distancia)
	if distancia < atencao:
		for i in range(len(lista_valores)):
			if 0 < i < 90 and lista_valores[i] == distancia:
				velocidade = Twist(Vector3(-2, 0, 0), Vector3(0, 0, -2))
				print("VAI BATER - NW")

			elif 90 < i < 180 and lista_valores[i] == distancia:
				velocidade = Twist(Vector3(2, 0, 0), Vector3(0, 0, -2))
				print("VAI BATER - SW")

			elif 180 < i < 270 and lista_valores[i] == distancia:
				velocidade = Twist(Vector3(2, 0, 0), Vector3(0, 0, 2))
				print("VAI BATER - SE")

			elif 270 < i < 360 and lista_valores[i] == distancia:
				velocidade = Twist(Vector3(-2, 0, 0), Vector3(0, 0, 2))
				print("VAI BATER - NE")
		
		return True
	else:
		#print("OK")
		iteracoes += 1
		return False

if __name__=="__main__":

	rospy.init_node("le_scan")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)



	while not rospy.is_shutdown():
		rospy.sleep(1)
