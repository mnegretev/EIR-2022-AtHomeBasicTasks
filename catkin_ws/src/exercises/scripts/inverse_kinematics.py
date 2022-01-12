#!/usr/bin/env python
#
# ESCUELA DE INVIERNO DE ROBOTICA 2022 - FEDERACION MEXICANA DE ROBOTICA
# EJERCICIO 7 - CINEMATICA INVERSA POR NEWTON-RAPHSON
#

import math
import rospy
import tf
import tf.transformations as tft
import numpy
import urdf_parser_py.urdf
from geometry_msgs.msg import PointStamped
from custom_msgs.srv import *

def get_model_info():
    global joints, transforms
    robot_model = urdf_parser_py.urdf.URDF.from_parameter_server()
    joints = {'left': [None for i in range(8)], 'right': [None for i in range(8)]}
    transforms = {'left':[], 'right':[]}
    for joint in robot_model.joints:
        for i in range(1,8):
            joints['left' ][i-1] = joint if joint.name == ('la_'+str(i)+'_joint') else joints['left' ][i-1]
            joints['right'][i-1] = joint if joint.name == ('ra_'+str(i)+'_joint') else joints['right'][i-1]
        joints['left' ][7] = joint if joint.name == 'la_grip_center_joint' else joints['left' ][7]
        joints['right'][7] = joint if joint.name == 'ra_grip_center_joint' else joints['right'][7]
    for joint in joints['left']:
        T = tft.translation_matrix(joint.origin.xyz)
        R = tft.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2])
        transforms['left'].append(tft.concatenate_matrices(T,R))
    for joint in joints['right']:
        T = tft.translation_matrix(joint.origin.xyz)
        R = tft.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2])
        transforms['right'].append(tft.concatenate_matrices(T,R))
    
def forward_kinematics(q, Ti, Wi):
    #
    # EJERCICIO:
    # Inspeccione el siguiente codigo y compare con las ecuaciones
    # para el calculo del cinematica directa.
    #
    H  = tft.identity_matrix()
    for i in range(len(q)):
        H  = tft.concatenate_matrices(H, Ti[i], tft.rotation_matrix(q[i], Wi[i]))
    H  = tft.concatenate_matrices(H, Ti[7])
    x,y,z = H[0,3], H[1,3], H[2,3]          # Get xyz from resulting H
    R,P,Y = list(tft.euler_from_matrix(H))  # Get RPY from resulting H
    return numpy.asarray([x,y,z,R,P,Y])

def jacobian(q, Ti, Wi):
    delta_q = 0.000001
    #
    # EJERCICIO:
    # Inspeccione el siguiente codigo y compare con las ecuaciones
    # para el calculo de la matriz jacobiana.
    #
    J = numpy.asarray([[0.0 for a in q] for i in range(6)])            # J 6x7 full of zeros
    qn = numpy.asarray([q,]*len(q)) + delta_q*numpy.identity(len(q))   # q_next
    qp = numpy.asarray([q,]*len(q)) - delta_q*numpy.identity(len(q))   # q_prev
    for i in range(len(q)):
        J[:,i] = (forward_kinematics(qn[i], Ti, Wi) - forward_kinematics(qp[i], Ti, Wi))/delta_q/2.0
    return J

def inverse_kinematics_xyzrpy(x, y, z, roll, pitch, yaw, Ti, Wi):
    pd = numpy.asarray([x,y,z,roll,pitch,yaw])  # Posicion deseada
    tolerance = 0.01
    max_iterations = 20
    iterations = 0
    #
    # EJERCICIO:
    # Inspeccione el siguiente codigo y compare con el algoritmo
    # de Newton-Raphson para el calculo de la cinematica inversa
    #
    q = numpy.asarray([-0.5, 0.6, 0.3, 2.0, 0.3, 0.2, 0.3])  # Initial guess
    p = forward_kinematics(q, Ti, Wi)
    err = p - pd
    err[3:6] = (err[3:6] + math.pi)%(2*math.pi) - math.pi
    while numpy.linalg.norm(err) > tolerance and iterations < max_iterations:
        J = jacobian(q, Ti, Wi)
        q = (q - numpy.dot(numpy.linalg.pinv(J), err) + math.pi)%(2*math.pi) - math.pi
        p = forward_kinematics(q, Ti, Wi)
        err = p - pd
        err[3:6] = (err[3:6] + math.pi)%(2*math.pi) - math.pi
        iterations +=1
    if iterations < max_iterations:
        print("InverseKinematics.->IK solved after " + str(iterations) + " iterations: " + str(q))
        return q
    else:
        print("InverseKinematics.->Cannot solve IK. Max attempts exceeded. ")
        return None

def callback_la_ik_for_pose(req):
    global transforms, joints
    Ti = transforms['left']                               
    Wi = [joints['left'][i].axis for i in range(len(joints['left']))]  
    q = inverse_kinematics_xyzrpy(req.x, req.y, req.z, req.roll, req.pitch, req.yaw, Ti, Wi)
    if q is None:
        return None
    resp = InverseKinematicsResponse()
    [resp.q1,resp.q2,resp.q3,resp.q4,resp.q5,resp.q6,resp.q7] = [q[0], q[1], q[2], q[3], q[4], q[5], q[6]]
    return resp

def callback_ra_ik_for_pose(req):
    global transforms, joints
    Ti = transforms['right']                               
    Wi = [joints['right'][i].axis for i in range(len(joints['right']))]  
    q = inverse_kinematics_xyzrpy(req.x, req.y, req.z, req.roll, req.pitch, req.yaw, Ti, Wi)
    if q is None:
        return False
    resp = InverseKinematicsResponse()
    [resp.q1,resp.q2,resp.q3,resp.q4,resp.q5,resp.q6,resp.q7] = [q[0], q[1], q[2], q[3], q[4], q[5], q[6]]
    return resp

def callback_la_fk(req):
    global transforms, joints
    Ti = transforms['left']                               
    Wi = [joints['left'][i].axis for i in range(len(joints['left']))]  
    x = forward_kinematics([req.q1, req.q2, req.q3, req.q4, req.q5, req.q6, req.q7], Ti, Wi)
    resp = DirectKinematicsResponse()
    [resp.x, resp.y, resp.z, resp.roll, resp.pitch, resp.yaw] = x
    return resp

def callback_ra_fk(req):
    global transforms, joints
    Ti = transforms['right']                               
    Wi = [joints['right'][i].axis for i in range(len(joints['right']))]  
    x = forward_kinematics([req.q1, req.q2, req.q3, req.q4, req.q5, req.q6, req.q7], Ti, Wi)
    resp = DirectKinematicsResponse()
    [resp.x, resp.y, resp.z, resp.roll, resp.pitch, resp.yaw] = x
    return resp

def main():
    print("EJERCICIO 7 - CINEMATICA INVERSA POR NEWTON-RAPHSON")
    rospy.init_node("ik_geometric")
    get_model_info()
    rospy.Service("/manipulation/la_inverse_kinematics", InverseKinematics, callback_la_ik_for_pose)
    rospy.Service("/manipulation/ra_inverse_kinematics", InverseKinematics, callback_ra_ik_for_pose)
    rospy.Service("/manipulation/la_direct_kinematics", ForwardKinematics, callback_la_fk)
    rospy.Service("/manipulation/ra_direct_kinematics", ForwardKinematics, callback_ra_fk)
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    main()
