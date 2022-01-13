# EJERCICIO FINAL
# Agregue funciones 'say' en los estados de modo que el robot
# indique por voz la parte de la tarea que se esta ejecutando
while not rospy.is_shutdown():
    if current_state == "SM_INIT":
        print("Waiting for new task")
        current_state = "SM_WAITING_NEW_TASK"
    elif current_state == "SM_WAITING_NEW_TASK":
        if new_task:
            req_object, req_loc = parse_command(recognized_speech)
            say("Executing the command, " + recognized_speech)
            current_state = "SM_MOVE_HEAD"
            new_task = False
            executing_task = True
