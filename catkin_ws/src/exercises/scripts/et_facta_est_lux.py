#!/usr/bin/env python
#
# ESCUELA DE INVIERNO DE ROBOTICA 2022 - FEDERACION MEXICANA DE ROBOTICA
# EJERCICIO FINAL - PLANEACION DE ACCIONES CON MAQUINAS DE ESTADOS
#

import rospy
import tf
import math
import time
from std_msgs.msg import String, Float64
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

def parse_command(cmd):
    obj = "pringles" if "PRINGLES" in cmd else "drink"
    loc = [8.5,8.5] if "TABLE" in cmd else [5.66, 6.12]
    return obj, loc



def main():
    global new_task, recognized_speech, executing_task
    print "EJERCICIO FINAL - PLANEACION DE ACCIONES CON MAQUINAS DE ESTADOS "
    rospy.init_node("final_exercise")
    rospy.Subscriber('/recognized', String, callback_recognized_speech)
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

    current_state = "SM_INIT"
    requested_object   = ""
    requested_location = [0,0]
    goal_pose = PoseStamped()
    goal_pose.pose.orientation.w = 1.0
    while not rospy.is_shutdown():
        if current_state == "SM_INIT":
            print("Waiting for new task")
            current_state = "SM_WAITING_NEW_TASK"
        elif current_state == "SM_WAITING_NEW_TASK":
            if new_task:
                requested_object, requested_location = parse_command(recognized_speech)
                print("New task received: " + requested_object + " to  " + str(requested_location))
                current_state = "SM_MOVE_HEAD"
                new_task = False
                executing_task = True
        elif current_state == "SM_MOVE_HEAD":
            print("Moving head to look at table...")
            pubHdPan.publish(0.0)
            pubHdTilt.publish(-0.9)
            time.sleep(2.0)
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
            obj_point = PointStamped()
            obj_point.header.frame_id = "kinect_link"
            obj_point.header.stamp = rospy.Time(0)
            obj_point.point.x = resp_find_object.x
            obj_point.point.y = resp_find_object.y
            obj_point.point.z = resp_find_object.z
            target_frame = "shoulders_left_link" if requested_object == "pringles" else "shoulders_right_link"
            print("Transforming " + requested_object + " position to " + target_frame)
            obj_point = listener.transformPoint(target_frame, obj_point)
            print("Trying to get inverse kinematics for pose " + str([obj_point.point.x,obj_point.point.y,obj_point.point.z]))
            req_inverse_kin = InverseKinematicsRequest()
            req_inverse_kin.x = obj_point.point.x + 0.05
            req_inverse_kin.y = obj_point.point.y
            req_inverse_kin.z = obj_point.point.z + 0.1
            req_inverse_kin.roll  = 3.0
            req_inverse_kin.pitch = -1.57
            req_inverse_kin.yaw   = -3.0
            if requested_object == "pringles":
                resp_inverse_kin = clt_la_inverse_kin(req_inverse_kin)
            else:
                resp_inverse_kin = clt_ra_inverse_kin(req_inverse_kin)
            current_state = "SM_MOVE_LEFT_ARM" if requested_object == "pringles" else "SM_MOVE_RIGHT_ARM"
        elif current_state == "SM_MOVE_LEFT_ARM":
            print("Moving left manipulator to stand by position")
            pubLaAngle1.publish(-0.9)
            pubLaAngle2.publish(0.0)
            pubLaAngle3.publish(0.0)
            pubLaAngle4.publish(1.8)
            pubLaAngle5.publish(0.0)
            pubLaAngle6.publish(0.6)
            pubLaAngle7.publish(0.0)
            time.sleep(2.0)
            print("Moving left manipulator to object position")
            pubLaAngle1.publish(resp_inverse_kin.q1)
            pubLaAngle2.publish(resp_inverse_kin.q2)
            pubLaAngle3.publish(resp_inverse_kin.q3)
            pubLaAngle4.publish(resp_inverse_kin.q4)
            pubLaAngle5.publish(resp_inverse_kin.q5)
            pubLaAngle6.publish(resp_inverse_kin.q6)
            pubLaAngle7.publish(resp_inverse_kin.q7)
            time.sleep(2.0)
            current_state = "SM_START_MOVEMENT"
        elif current_state == "SM_MOVE_RIGHT_ARM":
            print("Moving right manipulator to stand by position")
            pubRaAngle1.publish(-0.9)
            pubRaAngle2.publish(0.0)
            pubRaAngle3.publish(0.0)
            pubRaAngle4.publish(1.8)
            pubRaAngle5.publish(0.0)
            pubRaAngle6.publish(0.6)
            pubRaAngle7.publish(0.0)
            time.sleep(2.0)
            print("Moving right manipulator to object position")
            pubRaAngle1.publish(resp_inverse_kin.q1)
            pubRaAngle2.publish(resp_inverse_kin.q2)
            pubRaAngle3.publish(resp_inverse_kin.q3)
            pubRaAngle4.publish(resp_inverse_kin.q4)
            pubRaAngle5.publish(resp_inverse_kin.q5)
            pubRaAngle6.publish(resp_inverse_kin.q6)
            pubRaAngle7.publish(resp_inverse_kin.q7)
            time.sleep(2.0)
            current_state = "SM_START_MOVEMENT"
        elif current_state == "SM_START_MOVEMENT":
            print("Sending goal position to " + str(requested_location))
            pubHdPan.publish(0.0)
            pubHdTilt.publish(-0.7)
            goal_pose.pose.position.x = requested_location[0]
            goal_pose.pose.position.y = requested_location[1]
            pubGoalPose.publish(goal_pose)
            current_state = "SM_WAIT_FOR_MOVEMENT_FINISHED"
        elif current_state == "SM_WAIT_FOR_MOVEMENT_FINISHED":
            current_state == "SM_FINISHED"
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
