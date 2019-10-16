#! /usr/bin/env python
# -*- coding:utf-8 -*-

"""
    Para funcionar o drone ja ter decolado
    Opere o aparelho via teleop

"""


import rospy

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry


topico_odom = "/bebop/odom"

x = -1000
y = -1000
z = -1000


empty_msg = Empty()


def recebeu_leitura(dado):
    """
        Grava nas variáveis x,y,z a posição extraída da odometria
        Atenção: *não coincidem* com o x,y,z locais do drone
    """
    global x
    global y 
    global z 

    x = dado.pose.pose.position.x
    y = dado.pose.pose.position.y
    z = dado.pose.pose.position.z



if __name__=="__main__":

    rospy.init_node("print_odom")

    # Cria um subscriber que chama recebeu_leitura sempre que houver nova odometria
    recebe_scan = rospy.Subscriber(topico_odom, Odometry , recebeu_leitura)
    vel_pub = rospy.Publisher("bebop/cmd_vel", Twist, queue_size = 1)
    takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size = 1, latch=True)
    landing = rospy.Publisher('bebop/land', Empty, queue_size = 1, latch=True)

    zerov = Twist(Vector3(0,0,0), Vector3(0,0,0))


    v = 0.3  # Velocidade linear
    vel = Twist(Vector3(v,0,0), Vector3(0,0,0))

    count = 15
    x0 = -1000
    y0 = -1000

    try:




        while not rospy.is_shutdown():

            velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            vel_pub.publish(velocidade)
            print("x {} y {} z {}".format(x, y, z))
            rospy.sleep(2)

            rospy.sleep(3.0)
            takeoff.publish(empty_msg)
            rospy.sleep(5.0)
            d = 0
            print(d)


            while d < 5:

                if x0 == -1000:
                    x0 = x
                    y0 = y
                    z0 = z

                vel_pub.publish(vel)
                rospy.sleep(0.3)
                d = ((x-x0)**2+(y-y0)**2)**0.5
                print(x, x0)

                print(d)

            vel_pub.publish(zerov)

            rospy.sleep(3.0)
            landing.publish(empty_msg)
            rospy.sleep(5.0)
            break

    except rospy.ROSInterruptException:
        vel_pub.publish(vel)
        rospy.sleep(1.0)