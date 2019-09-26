#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Manuela Castilla", "Rafael Santos", "Fabio Miranda"]

### aceitar o timer
### ir mais para tras
### prioridade dos comportamentos

import rospy
import numpy as np
import tf
import math
import cv2
import time
from turtlebot3_msgs.msg import Sound
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
import cormodule
from math import pi
from std_msgs.msg import UInt8
import visao_module



bridge = CvBridge()

classe = None
cv_image = None
viu_dog = False
ccaixa = []
cimagem = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
tolerancia = 50

area = 0.0 # Variavel com a area do maior contorno
som = None
# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 


nbumper = -1

def bateu(dado):
    global nbumper
    nbumper = dado.data
    #print("Intensities")
    #print(np.array(dado.intensities).round(decimals=2))

def reagir(pub):
    global nbumper

    frente = Vector3(0.1, 0, 0)
    tras = Vector3(-0.1, 0, 0)

    direita = Vector3(0, 0, -pi/4)
    esquerda = Vector3(0, 0, pi/4)

    t1 = Twist(tras, esquerda)
    t2 = Twist(tras, direita)
    t3 = Twist(frente, direita)
    t4 = Twist(frente, esquerda)


    if nbumper == 1:
        pub.publish(t1)
    if nbumper == 2:
        pub.publish(t2)
    if nbumper == 3:
        pub.publish(t3)
    if nbumper == 4:
        pub.publish(t4)
        som.publish(2)
        
    rospy.sleep(1)

    nbumper = -1

def documenta_visual(resultados):
    global viu_dog
    if resultados is None or len(resultados) == 0:
        return
    for r in resultados:
            # print(r) - print feito para documentar e entender
            # o resultado
            if r[0] == "cat":
                viu_dog = True

def reagir_visual(pub, som):
    global viu_dog
    vel = Twist(Vector3(0,0,0), Vector3(0,0, 0))
    viu_dog = False
    som.publish(3)
    rospy.sleep(2)

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    print("frame")
    global cv_image
    global ccaixa
    global cimagem
    global classe

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
        centro, imagem, classe =  visao_module.processa(cv_image)
        documenta_visual(classe)
        ccaixa, cimagem, area =  cormodule.identifica_cor(cv_image)


        depois = time.clock()
        cv2.line(cv_image,(cimagem[0]-tolerancia,0),(cimagem[0]-tolerancia,cv_image.shape[0]-1),(255,0,0),5)  
        cv2.line(cv_image,(cimagem[0]+tolerancia,0),(cimagem[0]+tolerancia,cv_image.shape[0]-1),(255,0,0),5)       

        cv2.imshow("Camera", cv_image)
    except CvBridgeError as e:
        print('ex', e)
    
if __name__=="__main__":
    rospy.init_node("cor")
    rospy.Subscriber("/bumper", UInt8, bateu)
    topico_imagem = "/kamera"
    
    # Para renomear a *webcam
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
    # v2
    #  rosrun topic_tools relay /raspicam/image_raw/compressed /kamera
    #  rostopic pub /sound turtlebot3_msgs/Sound  1


    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    print("Usando ", topico_imagem)
    global som
    output = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    som = rospy.Publisher("/sound", Sound, queue_size=1)#latch=True)

    def centralizar(c_caixa, c_imagem, pub):
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
            if nbumper != -1:
                reagir(output)
            if viu_dog:
                reagir_visual(output, som)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
