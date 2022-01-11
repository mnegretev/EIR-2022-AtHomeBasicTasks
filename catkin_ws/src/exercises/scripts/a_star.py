#!/usr/bin/env python
#
# ESCUELA DE INVIERNO DE ROBOTICA 2022 - FEDERACION MEXICANA DE ROBOTICA
# EJERCICIO 2 - PLANEACION DE RUTAS CON A* (A ESTRELLA)
#

import numpy
import heapq
import rospy
import math
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Path
from nav_msgs.srv import *
from collections import deque

msg_path = Path()

def a_star(start_r, start_c, goal_r, goal_c, grid_map):
    open_list   = []
    in_closed_list = numpy.full( grid_map.shape, False)
    in_open_list   = numpy.full( grid_map.shape, False)
    g_values       = numpy.full( grid_map.shape, float("inf"))
    f_values       = numpy.full( grid_map.shape, float("inf"))
    previous       = numpy.full((grid_map.shape[0], grid_map.shape[0], 2), -1)
    #
    # EJERCICIO:
    # Modifique la lista de nodos adyacentes para usar conectividad ocho
    # en lugar de conectividad 4
    adjacents      = [[1,0],[0,1],[-1,0],[0,-1]]
    #adjacents      = [[1,0],[0,1],[-1,0],[0,-1], [1,1], [-1,1], [-1,-1],[1,-1]]
    #
    # FIN DEL EJERCICIO
    #
    heapq.heappush(open_list, (0, [start_r, start_c]))
    in_open_list[start_r, start_c] = True
    g_values[start_r, start_c] = 0
    f_values[start_r, start_c] = 0
    [row, col] = [start_r, start_c]

    while len(open_list) > 0 and [row, col] != [goal_r, goal_c]:
        [row, col] = heapq.heappop(open_list)[1]
        in_closed_list[row,col] = True
        adjacents_nodes = [[row+i, col+j] for [i,j] in adjacents]
        for [r,c] in adjacents_nodes:
            if grid_map[r,c] != 0 or in_closed_list[r,c]:
                continue

            #
            # EJERCICIO:
            # Modifique el calculo del valor 'g' y la heuristica 'h' para utilizar
            # distancia euclidiana en lugar de distancia de Manhattan.
            g = g_values[row, col] + abs(row-r) + abs(col-c)
            h = abs(goal_r - r) + abs(goal_c - c)
            # g = g_values[row, col] + math.sqrt((row-r)**2 + (col - c)**2)
            # h = math.sqrt((goal_r-r)**2 + (goal_c - c)**2)
            #
            # FIN DEL EJERCICIO
            #
            
            f = g + h
            if g < g_values[r,c]:
                g_values[r,c] = g
                f_values[r,c] = f
                previous[r,c] = [row,col]
            if not in_open_list[r,c]:
                heapq.heappush(open_list, (f_values[r,c], [r,c]))
                in_open_list[r,c] = True
                
    if [row,col] != [goal_r, goal_c]:
        print("Cannot calculate path. :'(")
        return []
    print("Path calculated succesfully :D ")
    path = []
    while[row, col] != [-1,-1]:
        path.insert(0,[row, col])
        [row, col] = previous[row, col]
    return path

def get_maps():
    print("Getting inflated and cost maps...")
    clt_static_map = rospy.ServiceProxy("/static_map"  , GetMap)
    clt_inflated   = rospy.ServiceProxy("/inflated_map", GetMap)
    try:
        static_map = clt_static_map().map
    except:
        print("Cannot get static map. Terminating program. ")
        exit()
    try:
        inflated_map = clt_inflated().map
        print("Using inflated map with " +str(len(inflated_map.data)) + " cells.")
    except:
        inflated_map = static_map
        print("Cannot get augmented maps. Using static map instead.")
    inflated_map = numpy.reshape(numpy.asarray(inflated_map.data), (static_map.info.height, static_map.info.width))
    return [static_map, inflated_map]

def callback_a_star(req):
    [s_map, inflated_map] = get_maps()
    res = s_map.info.resolution
    [sx, sy] = [req.start.pose.position.x, req.start.pose.position.y]
    [gx, gy] = [req.goal .pose.position.x, req.goal .pose.position.y]
    [zx, zy] = [s_map.info.origin.position.x, s_map.info.origin.position.y]
    print("Calculating path by A* from " + str([sx, sy])+" to "+str([gx, gy]))
    path = a_star(int((sy-zy)/res), int((sx-zx)/res), int((gy-zy)/res), int((gx-zx)/res), inflated_map)
    msg_path.poses = []
    for [r,c] in path:
        msg_path.poses.append(PoseStamped(pose=Pose(position=Point(x=(c*res + zx), y=(r*res + zy)))))
    return GetPlanResponse(msg_path)

def main():
    print "EXERCISE 02 - PATH PLANNING BY A*"
    rospy.init_node("a_star_path_planning")
    rospy.wait_for_service('/static_map')
    rospy.Service('/path_planning/a_star_search'  , GetPlan, callback_a_star)
    pub_path = rospy.Publisher('/path_planning/a_star_path', Path, queue_size=10)
    loop = rospy.Rate(2)
    msg_path.header.frame_id = "map"
    while not rospy.is_shutdown():
        pub_path.publish(msg_path)
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
