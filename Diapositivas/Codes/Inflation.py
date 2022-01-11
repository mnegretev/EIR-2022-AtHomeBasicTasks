# EJERCICIO:                                                                                                                    
for i in range(height):                                                                                                        
  for j in range(width):                                                                                                     
    if static_map[i,j] > 50:                                                                                               
      for k1 in range(-inflation_cells, inflation_cells+1):                                                              
        for k2 in range(-inflation_cells, inflation_cells+1):                                                          
          inflated[i+k1, j+k2] = 100  
