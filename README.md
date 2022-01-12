# Tareas Básicas en Robots de Servicio Doméstico
###Escuela de Invierno de Robótica, 2022.


## Requerimientos

* Ubuntu 18.04
* ROS Melodic http://wiki.ros.org/melodic/Installation/Ubuntu

## Instalación

Nota: se asume que ya se tiene instalado Ubuntu y ROS.

* $ cd
* $ git clone https://github.com/mnegretev/EIR-2022-AtHomeBasicTasks
* $ cd EIR-2022-AtHomeBasicTasks
* $ ./Setup.sh
* $ cd catkin_ws
* $ catkin_make -j2 -l2

Para probar que todo se instaló y compiló correctamente:

* $ cd 
* $ source EIR-2022-AtHomeBasicTasks/catkin_ws/devel/setup.bash
* $ roslaunch bring_up path_planning.launch

Si todo se instaló y compiló correctamente, se debería ver un RViz como el siguiente:

<img src="https://github.com/mnegretev/EIR-2022-AtHomeBasicTasks/blob/master/Media/rviz.png" alt="RViz" width="639"/>

Un Gazebo como el siguiente:

<img src="https://github.com/mnegretev/EIR-2022-AtHomeBasicTasks/blob/master/Media/gazebo.png" alt="Gazebo" width="631"/>

Y una GUI como la siguiente:

<img src="https://github.com/mnegretev/EIR-2022-AtHomeBasicTasks/blob/master/Media/gui.png" alt="GUI" width="328"/>

## Máquina Virtual

Se puede descargar una máquina virtual con todo lo necesario ya instalado de <br>
[Esta dirección](https://drive.google.com/drive/folders/1hhylxy_V94zMYp2UaCKhEbK1hR3pjPlN?usp=sharing) <br>
En esa misma carpeta hay un video con instrucciones para usar la máquina virtual. <br>
Usuario: Neo <br>
Contraseña: fidelio

## Contacto
Dr. Marco Negrete<br>
Profesor Asociado C<br>
Departamento de Procesamiento de Señales<br>
Facultad de Ingeniería, UNAM <br>
[mnegretev.info](http://mnegretev.info)<br>
marco.negrete@ingenieria.unam.edu<br>
