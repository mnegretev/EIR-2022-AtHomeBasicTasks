# EJERCICIO:
# Modifique el calculo del valor 'g' y la heuristica 'h' para 
# utilizar distancia euclidiana en lugar de distancia de Manhattan.
g = g_values[row, col] + abs(row-r) + abs(col-c)
h = abs(goal_r - r) + abs(goal_c - c)
# g = g_values[row, col] + math.sqrt((row-r)**2 + (col - c)**2)
# h = math.sqrt((goal_r-r)**2 + (goal_c - c)**2)
