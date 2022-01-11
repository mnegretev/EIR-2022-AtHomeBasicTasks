# EJERCICIO
# Modifique las constantes v_max, w_max, alpha y beta y observe
# el cambio en el comportamiento del robot.
v_max = 0.4
w_max = 0.5
alpha = 1.0
beta  = 0.1
[error_x, error_y] = [goal_x - robot_x, goal_y - robot_y]
error_a = (math.atan2(error_y, error_x) - robot_a + math.pi)%(2*math.pi) - math.pi    
cmd_vel.linear.x  = v_max*math.exp(-error_a*error_a/alpha)
cmd_vel.angular.z = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
