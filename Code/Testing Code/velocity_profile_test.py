def trap_velocity(motor_name, direction, target_speed, max_accel):
    delta_T = 0.2 #sec
    if motor_name == 0:
        target_mot = mot_A
    elif motor_name == 1:
        target_mot = mot_B
    elif motor_name == 2:
        #idk do something ig
        pass

    current_speed = target_mot.get_encoder().get_velocity()
    
    step = max_accel * delta_T
    if current_speed+step < target_speed:
        current_speed += step
    elif current_speed-step > target_speed:
        current_speed -= step
    else:
        current_speed = target_speed

    target_mot.set_velocity(current_speed)
    state = 2
    yield 


mot_A_state = 1
mot_B_state = 2
state = mot_A_state << 4 | mot_B_state
print(state)