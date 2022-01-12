#!/usr/bin/env python
#
# ESCUELA DE INVIERNO DE ROBOTICA 2022 - FEDERACION MEXICANA DE ROBOTICA
# EJERCICIO FINAL - PLANEACION DE ACCIONES CON MAQUINAS DE ESTADOS
#

import rospy
import tf
import math
from std_msgs.msg import String
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point
from sound_play.msg import SoundRequest
from custom_msgs.srv import InverseKinematics, FindObject

def callback_recognized_speech(msg):
    global recognized_speech
    recognized_speech = msg.data

def main():
    global pub_cmd_vel, loop, listener
    print "EJERCICIO FINAL - PLANEACION DE ACCIONES CON MAQUINAS DE ESTADOS "
    rospy.init_node("final_exercise")
    rospy.Subscriber('/recognized', String, callback_recognized_speech)
    pub_goal_pose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    pub_say       = rospy.Publisher('/robotsound', SoundRequest, queue_size=10)
    listener = tf.TransformListener()
    print("Waiting for services...")
    rospy.wait_for_service('/manipulation/la_inverse_kinematics')
    rospy.wait_for_service('/vision/find_object')
    print("Services are now available.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
