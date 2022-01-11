#!/usr/bin/env python
#
# ESCUELA DE INVIERNO DE ROBOTICA 2022 - FEDERACION MEXICANA DE ROBOTICA
# EJERCICIO 4 - CONTROL PARA SEGUIMIENTO DE RUTAS
#

import rospy
import tf
import math
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanRequest
from custom_msgs.srv import SmoothPath, SmoothPathRequest
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point

pub_cmd_vel = None
loop        = None
listener    = None

def calculate_control(robot_x, robot_y, robot_a, goal_x, goal_y):
    cmd_vel = Twist()
    
    #
    # EJERCICIO
    # Modifique las constantes v_max, w_max, alpha y beta y observe
    # el cambio en el comportamiento del robot.
    #
    v_max = 0.4
    w_max = 0.5
    alpha = 1.0
    beta  = 0.1
    [error_x, error_y] = [goal_x - robot_x, goal_y - robot_y]
    error_a = (math.atan2(error_y, error_x) - robot_a + math.pi)%(2*math.pi) - math.pi    
    cmd_vel.linear.x  = v_max*math.exp(-error_a*error_a/alpha)
    cmd_vel.angular.z = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
    
    return cmd_vel

def follow_path(path):
    current_point = 0
    [local_xg,  local_yg ] = path[current_point]
    [global_xg, global_yg] = path[len(path)-1]
    [robot_x, robot_y, robot_a]    = get_robot_pose(listener)
    global_error = math.sqrt((global_xg-robot_x)**2 + (global_yg-robot_y)**2)
    local_error  = math.sqrt((local_xg-robot_x) **2 + (local_yg-robot_y) **2)
    
    while not rospy.is_shutdown() and global_error > 0.2:
        pub_cmd_vel.publish(calculate_control(robot_x, robot_y, robot_a, local_xg, local_yg))
        loop.sleep()
        [robot_x, robot_y, robot_a] = get_robot_pose(listener)
        local_error  = math.sqrt((local_xg-robot_x) **2 + (local_yg-robot_y) **2)
        current_point = min(current_point+1, len(path)-1) if local_error < 0.3 else current_point
        [local_xg,  local_yg ] = path[current_point]
        global_error = math.sqrt((global_xg-robot_x)**2 + (global_yg-robot_y)**2)
    pub_cmd_vel.publish(Twist())
    return
    
def callback_global_goal(msg):
    print "Calculating path from robot pose to " + str([msg.pose.position.x, msg.pose.position.y])
    [robot_x, robot_y, robot_a] = get_robot_pose(listener)
    req = GetPlanRequest(goal=PoseStamped(pose=msg.pose))
    req.start.pose.position = Point(x=robot_x, y=robot_y)
    path = rospy.ServiceProxy('/path_planning/a_star_search', GetPlan)(req).plan
    print "Following path with " + str(len(path.poses)) + " points..."
    follow_path([[p.pose.position.x, p.pose.position.y] for p in path.poses])
    print "Global goal point reached"

def get_robot_pose(listener):
    try:
        ([x, y, z], rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        a = 2*math.atan2(rot[2], rot[3])
        a = a - 2*math.pi if a > math.pi else a
        return [x, y, a]
    except:
        pass
    return [0,0,0]

def main():
    global pub_cmd_vel, loop, listener
    print "EJERCICIO 4 - CONTROL PARA SEGUIMIENTO DE RUTAS"
    rospy.init_node("exercise4")
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_global_goal)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    listener = tf.TransformListener()
    loop = rospy.Rate(10)
    print("Waiting for service for path planning...")
    rospy.wait_for_service('/path_planning/a_star_search')
    print("Service for path planning is now available.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
