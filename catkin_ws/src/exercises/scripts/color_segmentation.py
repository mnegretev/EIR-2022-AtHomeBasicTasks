#!/usr/bin/env python
#
# ESCUELA DE INVIERNO DE ROBOTICA 2022 - FEDERACION MEXICANA DE ROBOTICA
# EJERCICIO 5 - SEGMENTACION POR COLOR
#

import numpy
import cv2
import ros_numpy
import rospy
import math
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point
from custom_msgs.srv import FindObject, FindObjectResponse

def segment_by_color(img_bgr, points, obj_name):
    #
    # EJERCICIO
    # Complete el codigo para asignar los limites de color
    # segun el objeto buscado (variable 'obj_name')
    # Para 'pringles': [25, 50, 50] - [35, 255, 255]
    # Para 'drink':    [10,200, 50] - [20, 255, 255]
    #
    lower_limit = numpy.array([25, 50, 50])
    upper_limit = numpy.array([35,255,255])
    #
    # FIN DEL EJERCICIO
    #
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    img_bin = cv2.inRange(img_hsv, lower_limit, upper_limit)
    idx = cv2.findNonZero(img_bin)
    [img_x, img_y, a, b] = cv2.mean(idx)
    [x,y,z] = [0,0,0]
    counter = 0
    for [[c, r]] in idx:
        xt, yt, zt = points[r,c][0], points[r,c][1], points[r,c][2]
        if math.isnan(xt) or math.isnan(yt) or math.isnan(zt):
            continue
        [x,y,z,counter] = [x+xt, y+yt, z+zt, counter+1]
    [x,y,z] = [x/counter, y/counter, z/counter] if counter > 0 else [0,0,0]
    return [img_x, img_y, x,y,z]

def callback_find_object(req):
    global pub_point, img_bgr
    print("Trying to find object: " + req.name)
    arr = ros_numpy.point_cloud2.pointcloud2_to_array(req.cloud)
    rgb_arr = arr['rgb'].copy()
    rgb_arr.dtype = numpy.uint32
    r,g,b = ((rgb_arr >> 16) & 255), ((rgb_arr >> 8) & 255), (rgb_arr & 255)
    img_bgr = cv2.merge((numpy.asarray(b,dtype='uint8'),numpy.asarray(g,dtype='uint8'),numpy.asarray(r,dtype='uint8')))
    [r, c, x, y, z] = segment_by_color(img_bgr, arr, req.name)
    hdr = Header(frame_id='kinect_link', stamp=rospy.Time.now())
    pub_point.publish(PointStamped(header=hdr, point=Point(x=x, y=y, z=z)))
    cv2.circle(img_bgr, (int(r), int(c)), 20, [0, 255, 0], thickness=3)
    resp = FindObjectResponse()
    resp.x, resp.y, resp.z = x, y, z
    return resp

def main():
    global pub_point, img_bgr
    print "EJERCICIO 5 - SEGMENTACION POR COLOR"
    rospy.init_node("color_segmentation")
    rospy.Service("/vision/find_object", FindObject, callback_find_object)
    pub_point = rospy.Publisher('/detected_object', PointStamped, queue_size=10)
    img_bgr = numpy.zeros((480, 640, 3), numpy.uint8)
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        cv2.imshow("Color Segmentation", img_bgr)
        cv2.waitKey(1)
        loop.sleep()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

