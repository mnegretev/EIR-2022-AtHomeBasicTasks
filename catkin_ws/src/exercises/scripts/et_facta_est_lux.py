#!/usr/bin/env python
#
# ESCUELA DE INVIERNO DE ROBOTICA 2022 - FEDERACION MEXICANA DE ROBOTICA
# EJERCICIO FINAL - PLANEACION DE ACCIONES CON MAQUINAS DE ESTADOS
#

import rospy
import tf
import math
import time
from std_msgs.msg import String, Float64, Bool
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanRequest
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, PointStamped
from sound_play.msg import SoundRequest
from custom_msgs.srv import *

def callback_recognized_speech(msg):
    global recognized_speech, new_task, executing_task
    if executing_task:
        return
    new_task = True
    recognized_speech = msg.data

def callback_goal_reached(msg):
    global goal_reached
    goal_reached = msg.data

def parse_command(cmd):
    obj = "pringles" if "PRINGLES" in cmd else "drink"
    loc = [8.0,8.5] if "TABLE" in cmd else [3.22, 9.72]
    return obj, loc

def move_left_arm(q1,q2,q3,q4,q5,q6,q7):
    global pubLaAngle1,pubLaAngle2,pubLaAngle3,pubLaAngle4,pubLaAngle5,pubLaAngle6,pubLaAngle7
    pubLaAngle1.publish(q1)
    pubLaAngle2.publish(q2)
    pubLaAngle3.publish(q3)
    pubLaAngle4.publish(q4)
    pubLaAngle5.publish(q5)
    pubLaAngle6.publish(q6)
    pubLaAngle7.publish(q7)
    time.sleep(2.0)

def move_left_gripper(q):
    global pubLaAngleGl,pubLaAngleGr
    pubLaAngleGl.publish(q)
    pubLaAngleGr.publish(q)
    time.sleep(1.0)

def move_right_arm(q1,q2,q3,q4,q5,q6,q7):
    global pubRaAngle1,pubRaAngle2,pubRaAngle3,pubRaAngle4,pubRaAngle5,pubRaAngle6,pubRaAngle7
    pubRaAngle1.publish(q1)
    pubRaAngle2.publish(q2)
    pubRaAngle3.publish(q3)
    pubRaAngle4.publish(q4)
    pubRaAngle5.publish(q5)
    pubRaAngle6.publish(q6)
    pubRaAngle7.publish(q7)
    time.sleep(2.0)

def move_right_gripper(q):
    global pubRaAngleGl,pubRaAngleGr
    pubRaAngleGl.publish(q)
    pubRaAngleGr.publish(q)
    time.sleep(1.0)

def move_head(pan, tilt):
    global pubHdPan,pubHdTilt
    pubHdPan.publish(pan)
    pubHdTilt.publish(tilt)
    time.sleep(1.0)

def move_base(linear, angular, t):
    global pubCmdVel
    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    pubCmdVel.publish(cmd)
    time.sleep(t)
    pubCmdVel.publish(Twist())

def go_to_goal_pose(goal_x, goal_y):
    global pubGoalPose
    goal_pose = PoseStamped()
    goal_pose.pose.orientation.w = 1.0
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    pubGoalPose.publish(goal_pose)

def say(text):
    global pubSay
    msg = SoundRequest()
    msg.sound   = -3
    msg.command = 1
    msg.volume  = 1.0
    msg.arg2    = "voice_kal_diphone"
    msg.arg = text
    pubSay.publish(msg)

def main():
    global new_task, recognized_speech, executing_task, goal_reached
    global pubLaAngle1,pubLaAngle2,pubLaAngle3,pubLaAngle4,pubLaAngle5,pubLaAngle6,pubLaAngle7
    global pubRaAngle1,pubRaAngle2,pubRaAngle3,pubRaAngle4,pubRaAngle5,pubRaAngle6,pubRaAngle7
    global pubLaAngleGl,pubLaAngleGr,pubRaAngleGl,pubRaAngleGr,pubHdPan,pubHdTilt
    global pubGoalPose, pubCmdVel, pubSay
    print "EJERCICIO FINAL - PLANEACION DE ACCIONES CON MAQUINAS DE ESTADOS "
    rospy.init_node("final_exercise")
    rospy.Subscriber('/recognized', String, callback_recognized_speech)
    rospy.Subscriber('/navigation/goal_reached', Bool, callback_goal_reached)
    pubGoalPose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    pubCmdVel   = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pubSay      = rospy.Publisher('/robotsound', SoundRequest, queue_size=10)
    pubLaAngle1 = rospy.Publisher("/la_1_controller/command", Float64, queue_size=10);
    pubLaAngle2 = rospy.Publisher("/la_2_controller/command", Float64, queue_size=10);
    pubLaAngle3 = rospy.Publisher("/la_3_controller/command", Float64, queue_size=10);
    pubLaAngle4 = rospy.Publisher("/la_4_controller/command", Float64, queue_size=10);
    pubLaAngle5 = rospy.Publisher("/la_5_controller/command", Float64, queue_size=10);
    pubLaAngle6 = rospy.Publisher("/la_6_controller/command", Float64, queue_size=10);
    pubLaAngle7 = rospy.Publisher("/la_7_controller/command", Float64, queue_size=10);
    pubRaAngle1 = rospy.Publisher("/ra_1_controller/command", Float64, queue_size=10);
    pubRaAngle2 = rospy.Publisher("/ra_2_controller/command", Float64, queue_size=10);
    pubRaAngle3 = rospy.Publisher("/ra_3_controller/command", Float64, queue_size=10);
    pubRaAngle4 = rospy.Publisher("/ra_4_controller/command", Float64, queue_size=10);
    pubRaAngle5 = rospy.Publisher("/ra_5_controller/command", Float64, queue_size=10);
    pubRaAngle6 = rospy.Publisher("/ra_6_controller/command", Float64, queue_size=10);
    pubRaAngle7 = rospy.Publisher("/ra_7_controller/command", Float64, queue_size=10);
    pubLaAngleGl = rospy.Publisher("/la_grip_left_controller/command" , Float64, queue_size=10);
    pubLaAngleGr = rospy.Publisher("/la_grip_right_controller/command", Float64, queue_size=10);
    pubRaAngleGl = rospy.Publisher("/ra_grip_left_controller/command" , Float64, queue_size=10);
    pubRaAngleGr = rospy.Publisher("/ra_grip_right_controller/command", Float64, queue_size=10);
    pubHdPan    = rospy.Publisher("/head_pan_controller/command" , Float64, queue_size=10);
    pubHdTilt   = rospy.Publisher("/head_tilt_controller/command", Float64, queue_size=10);
    
    listener = tf.TransformListener()
    loop = rospy.Rate(10)
    print("Waiting for services...")
    rospy.wait_for_service('/manipulation/la_inverse_kinematics')
    rospy.wait_for_service('/vision/find_object')
    print("Services are now available.")
    clt_la_inverse_kin = rospy.ServiceProxy("/manipulation/la_inverse_kinematics", InverseKinematics)
    clt_ra_inverse_kin = rospy.ServiceProxy("/manipulation/ra_inverse_kinematics", InverseKinematics)
    clt_find_object = rospy.ServiceProxy("/vision/find_object", FindObject)

    new_task = False
    executing_task = False
    recognized_speech = ""
    goal_reached = False

    current_state = "SM_INIT"
    requested_object   = ""
    requested_location = [0,0]

    #
    # EJERCICIO FINAL
    # Agregue funciones 'say' en los estados de modo que el robot
    # indique por voz la parte de la tarea que se esta ejecutando
    #
    
    while not rospy.is_shutdown():
        if current_state == "SM_INIT":
            print("Waiting for new task")
            current_state = "SM_WAITING_NEW_TASK"
        elif current_state == "SM_WAITING_NEW_TASK":
            if new_task:
                requested_object, requested_location = parse_command(recognized_speech)
                print("New task received: " + requested_object + " to  " + str(requested_location))
                say("Executing the command, " + recognized_speech)
                current_state = "SM_MOVE_HEAD"
                new_task = False
                executing_task = True
                
        elif current_state == "SM_MOVE_HEAD":
            print("Moving head to look at table...")
            move_head(0, -0.9)
            current_state = "SM_FIND_OBJECT"
            
        elif current_state == "SM_FIND_OBJECT":
            print("Trying to find object: " + requested_object)
            req_find_object = FindObjectRequest()
            req_find_object.cloud = rospy.wait_for_message("/kinect/points", PointCloud2)
            req_find_object.name  = requested_object
            resp_find_object = clt_find_object(req_find_object)
            print("Object found at: " + str([resp_find_object.x, resp_find_object.y, resp_find_object.z]))
            current_state = "SM_INVERSE_KINEMATICS"
            
        elif current_state == "SM_INVERSE_KINEMATICS":
            obj_p = PointStamped()
            obj_p.header.frame_id = "kinect_link"
            obj_p.header.stamp = rospy.Time(0)
            obj_p.point.x, obj_p.point.y, obj_p.point.z,  = resp_find_object.x, resp_find_object.y, resp_find_object.z
            target_frame = "shoulders_left_link" if requested_object == "pringles" else "shoulders_right_link"
            print("Transforming " + requested_object + " position to " + target_frame)
            obj_p = listener.transformPoint(target_frame, obj_p)
            print("Trying to get inverse kinematics for pose " + str([obj_p.point.x,obj_p.point.y,obj_p.point.z]))
            req_ik = InverseKinematicsRequest()
            req_ik.x = obj_p.point.x + 0.05
            req_ik.y = obj_p.point.y
            req_ik.z = obj_p.point.z + 0.1
            req_ik.roll  = 3.0
            req_ik.pitch = -1.57
            req_ik.yaw   = -3.0
            if requested_object == "pringles":
                resp_ik = clt_la_inverse_kin(req_ik)
            else:
                resp_ik = clt_ra_inverse_kin(req_ik)
            current_state = "SM_MOVE_LEFT_ARM" if requested_object == "pringles" else "SM_MOVE_RIGHT_ARM"
            
        elif current_state == "SM_MOVE_LEFT_ARM":
            print("Moving left manipulator to stand by position")
            move_left_arm(-0.6, 0, 0, 1.8, 0, 0.6, 0)
            move_left_gripper(0.7)
            print("Moving left manipulator to object position")
            move_left_arm(resp_ik.q1, resp_ik.q2, resp_ik.q3, resp_ik.q4, resp_ik.q5, resp_ik.q6, resp_ik.q7)
            move_left_gripper(-0.3)
            move_left_arm(resp_ik.q1, resp_ik.q2, resp_ik.q3, resp_ik.q4+0.1, resp_ik.q5, resp_ik.q6+0.1, resp_ik.q7)
            print("Moving backwards")
            move_base(-0.3, 0, 2.0)
            move_left_arm(-0.6, 0, 0, 1.8, 0, 0.6, 0)
            current_state = "SM_START_NAVIGATION"
            
        elif current_state == "SM_MOVE_RIGHT_ARM":
            print("Moving right manipulator to stand by position")
            move_right_arm(-0.6, 0, 0, 1.8, 0, 0.6, 0)
            move_right_gripper(0.7)
            print("Moving right manipulator to object position")
            move_right_arm(resp_ik.q1, resp_ik.q2, resp_ik.q3, resp_ik.q4, resp_ik.q5, resp_ik.q6, resp_ik.q7)
            move_right_gripper(-0.3)
            move_right_arm(resp_ik.q1, resp_ik.q2, resp_ik.q3, resp_ik.q4+0.1, resp_ik.q5, resp_ik.q6+0.1, resp_ik.q7)
            print("Moving backwards")
            move_base(-0.3, 0, 2.)
            move_right_arm(-0.6, 0, 0, 1.8, 0, 0.6, 0)
            current_state = "SM_START_NAVIGATION"
            
        elif current_state == "SM_START_NAVIGATION":
            print("Sending goal position to " + str(requested_location))
            go_to_goal_pose(requested_location[0], requested_location[1])
            goal_reached = False
            current_state = "SM_WAIT_FOR_MOVEMENT_FINISHED"
            
        elif current_state == "SM_WAIT_FOR_MOVEMENT_FINISHED":
            if goal_reached:
                print("Goal point reached")
                goal_reached = False
                current_state = "SM_LEAVE_OBJECT"

        elif current_state == "SM_LEAVE_OBJECT":
            if requested_object == 'pringles':
                move_left_arm(resp_ik.q1, resp_ik.q2, resp_ik.q3, resp_ik.q4, resp_ik.q5, resp_ik.q6, resp_ik.q7)
                move_left_gripper(0.5)
                move_left_arm(-0.6, 0, 0, 1.8, 0, 0.6, 0)
                move_left_gripper(0.0)
                move_left_arm(0,0,0,0,0,0,0)
            else:
                move_right_arm(resp_ik.q1, resp_ik.q2, resp_ik.q3, resp_ik.q4, resp_ik.q5, resp_ik.q6, resp_ik.q7)
                move_right_gripper(0.5)
                move_right_arm(-0.6, 0, 0, 1.8, 0, 0.6, 0)
                move_right_gripper(0.0)
                move_right_arm(0,0,0,0,0,0,0)
            go_to_goal_pose(3.39, 6.56)
            goal_reached = False
            current_state = "SM_WAIT_FOR_RETURN"

        elif current_state == "SM_WAIT_FOR_RETURN":
            if goal_reached:
                print("Return point reached")
                goal_reached = False
                current_state = "SM_APPROACH_TO_TABLE"

        elif current_state == "SM_APPROACH_TO_TABLE":
            go_to_goal_pose(3.39, 5.76)
            goal_reached = False
            current_state = "SM_WAIT_FOR_APPROACHING"

        elif current_state == "SM_WAIT_FOR_APPROACHING":
            if goal_reached:
                print("Start point reached")
                goal_reached = False
                move_base(0.2, 0, 1.3)
                current_state = "SM_INIT"
                executing_task = False
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
