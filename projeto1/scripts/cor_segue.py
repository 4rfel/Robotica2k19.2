#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Manuela Castilla", "Rafael Santos", "Fabio Miranda"]


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
import cormodule


bridge = CvBridge()

cv_image = None
ccaixa = []
cimagem = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos

area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 
from math import pi

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    print("frame")
    global cv_image
    global ccaixa
    global cimagem

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        cv_image = cv2.flip(cv_image, -1) 
        ccaixa, cimagem, area =  cormodule.identifica_cor(cv_image)
        depois = time.clock()
        cv2.imshow("Camera", cv_image)
    except CvBridgeError as e:
        print('ex', e)
    
if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/kamera"
    
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
    print("Usando ", topico_imagem)

    output = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    def centralizar(c_caixa, c_imagem, pub):
        tolerancia = 50
        vel_frente = Twist(Vector3(0.1,0,0), Vector3(0,0, 0))
        vel_direita = Twist(Vector3(0,0,0), Vector3(0,0,-pi/8))
        vel_esquerda = Twist(Vector3(0,0,0), Vector3(0,0,pi/8))

        xmin = c_imagem[0]-tolerancia
        xmax = c_imagem[0]+tolerancia
        if xmin < c_caixa[0] < xmax:
            pub.publish(vel_frente)
        elif c_caixa[0] > xmax:
            pub.publish(vel_direita)
        elif c_caixa[0] < xmin:
            pub.publish(vel_esquerda)




    try:

        while not rospy.is_shutdown():
            if len(ccaixa) != 0:
                centralizar(ccaixa, cimagem, output)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")


