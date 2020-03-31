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
import cormodule_mod


bridge = CvBridge()

cv_image = None
media = []
centro = []

print("Escolha uma das cores abaixo.")
cor = input("Azul: 0; Verde: 1; Roxo: 2. -> ")

cor = int(cor)

roxo = "#4c015b"
verde = "#006507"
azul = "#06355c"

lista_cores = [azul, verde, roxo]

dist = 0

atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos

area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    print("frame")
    global cv_image
    global media
    global centro
    global maior_area

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    # print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        #cv_image = cv2.flip(cv_image, -1)
        media, centro, maior_area =  cormodule_mod.identifica_cor(cv_image,lista_cores[cor])
        depois = time.clock()
        cv2.imshow("Camera", cv_image)
    except CvBridgeError as e:
        print('ex', e)

def scaneou(dado):
    global dist
    # print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
    # print("Leituras:")
    dists = []
    indices = [-5,-4,-3,-2,0,1,2,3,4,5]
    for e in indices:
        dists.append((np.array(dado.ranges).round(decimals=2))[e])
    
    dist = np.amin(dists)
    #print("Intensities")
    #print(np.array(dado.intensities).round(decimals=2))
    
if __name__=="__main__":
    rospy.init_node("cor")

    # topico_imagem = "/kamera"
    topico_imagem = "/camera/rgb/image_raw/compressed"
    # Para renomear a *webcam*
    #   Primeiro instale o suporte https://github.com/Insper/robot19/blob/master/guides/debugar_sem_robo_opencv_melodic.md
    #
    #	Depois faça:
    #	
    #	rosrun cv_camera cv_camera_node
    #
    # 	rosrun topic_tools relay  /cv_camera/image_raw/compressed /kamera
    #
    # 
    # Para renomear a câmera simulada do Gazebo
    # 
    # 	rosrun topic_tools relay  /camera/rgb/image_raw/compressed /kamera
    # 
    # Para renomear a câmera da Raspberry
    # 
    # 	rosrun topic_tools relay /raspicam_node/image/compressed /kamera
    # 

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)


    try:

        while not rospy.is_shutdown():
            vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
            if len(media) != 0 and len(centro) != 0:
                # Calcula a diferença entre o centro do creeper e o centro da tela
                dif = int(media[0]) - int(centro[0])
                
                if dist > 0.8 or media[0] == 0:
                    if -30 < dif and dif < 30:
                        vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
                        
                    elif dif < -30:
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))

                    elif dif > 30:
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))

                
                elif dist > 0.3 and dist <= 0.8:
                    vel = Twist(Vector3(0.05,0,0), Vector3(0,0,0))

                else:
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                    
                print("dists: {0}".format(media[0]))
                print("Diferenca: {0}".format(dif))
                print("Distancia: {0}".format(dist))
                # print("Area:{0}".format(maior_area))
                # print("Média cor: {0}, {1}".format(media[0], media[1]))
                # print("Centro tela: {0}, {1}".format(centro[0], centro[1]))
            velocidade_saida.publish(vel)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
