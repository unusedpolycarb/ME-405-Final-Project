#notes about lab structure:


#Use the cotask class to create a list of tasks (which are generator methods) that run cooperatively
#Each generator method must take in a tuple of (share, queue)
#Each generator method must yield the next state number for that task, this history stored in Task.trace
#Because it is a generator method, the state variable continues to exist within that task between function calls
#Both share and queue function as queues. Idk how they're different



#import statements
from pyb import Pin, Timer, I2C, UART
from src.BNO055_communication import BNO055
from src.romi_flags import RomiFlags
from src.IR_array import IR_array, IR_sensor
import utime
import struct
#import keyboard
from src.motor3 import L6206
from src import cotask, task_share
#credit to https://github.com/spluttflob/ME405-Support/blob/main/examples/basic_tasks.py for code in this lab
import gc


#how many different time loops are in our code currently?
# motor PWM
# encoder update
# checking for button press (user input)
# data collection



#setup method: runs once at beginning of program
def setup():
    pass
    #not sure if we should do all the definitions here, or in state 0 of task 2?
    #create motor objects
    #create encoder objects
    #enable everything, but don't run the motors


def bandpass(values, floor_thresholds, ceil_thresholds):
    for i in range(len(values)):
        values[i] = 1 - (floor_thresholds[i] - values[i]) / (floor_thresholds[i] - ceil_thresholds[i])
        if values[i] < 0:
            values[i] = 0
        if values[i] > 1:
            values[i] = 1
    return values


def normalize_list(values):
    min_val = min(values)
    max_val = max(values)

    if max_val - min_val == 0:
        return [0 for _ in values]
    
    total = sum(values)
    
    return [val/total for val in values]


def task1(shares):
    '''@brief: controller task'''
    
    ceil_thresholds = [3840, 3830, 3850, 3840, 3830]
    floor_thresholds = [2800, 3200, 2955, 3300, 2900]
    weights = [-22, -11, 0, 11, 22]

    DEAD_ZONE = 6
    MAX_DEV = 22
    V_TAN = 7.2 #rad/s
    delta_T = 0.02
    MAX_STEP = 1 #rad/s^2  
    MAX_YAW = 0.8
    MAX_YAW_MAX_DEV = 2.1


    IR_1 = IR_sensor(Pin.cpu.C2,3840)
    IR_2 = IR_sensor(Pin.cpu.C3,3830)
    IR_3 = IR_sensor(Pin.cpu.A4,3850)
    IR_4 = IR_sensor(Pin.cpu.B0,3840)
    IR_5 = IR_sensor(Pin.cpu.C1,3830)

    my_array = IR_array([IR_1, IR_2, IR_3, IR_4, IR_5])

    TRACK_WIDTH = 5.5 #in 
    CORRECTIVE_YAW = 2 #rad/s
    WHEEL_RADIUS = 1.375 #in
    CORRECTIVE_VEL = CORRECTIVE_YAW*TRACK_WIDTH/2/WHEEL_RADIUS #rad/s, evaluates to 4 rad/s
    
    NOMINAL_WHEEL_VEL = 20 #rad/s


    max_accel = 8 #rad/s^2
    state = 1

    mot_A_vel = NOMINAL_WHEEL_VEL #rad/s
    mot_B_vel = NOMINAL_WHEEL_VEL #rad/s
    yaw_rate = 0
  
    counter = 0
    counter_two = 0

    yield state
    while True:
        my_share, my_queue = shares
        flag = my_share.get()

        line_results = my_array.get_vals()
        deviation = 0
        line_results = bandpass(line_results, floor_thresholds, ceil_thresholds)
        line_results = normalize_list(line_results)

        for i in range(len(line_results)):
            deviation += line_results[i] * weights[i]

        if state == 1 and flag == RomiFlags.nucleo_button_press:
            print("yaw capture flag")
            my_share.put(RomiFlags.initial_yaw_capture)
            flag = None
            #state = 20
            state = 20

        if state == 20 and flag == RomiFlags.none:
            #start both motors at a reasonable speed
            print("A")
            my_share.put(RomiFlags.trap_velocity)
            my_queue.put(2)
            my_queue.put(int(mot_A_vel*1000*0.5))
            my_queue.put(1)
            my_queue.put(int(max_accel*1000))
            my_queue.put(int(mot_B_vel*1000*0.5))
            my_queue.put(1)
            my_queue.put(int(max_accel*1000))
            yaw_rate = 0
            state = 2
        
        

        if state == 2:
            #print(deviation)

            if abs(deviation) < DEAD_ZONE:
                step = 0
            
            elif counter > 0:
                counter = 0
                step = MAX_STEP * 20 * deviation / MAX_DEV * delta_T 
                yaw_rate += step

                #saturate yaw rate 
                if yaw_rate > MAX_YAW:
                    yaw_rate = MAX_YAW
                if yaw_rate < -1 * MAX_YAW:
                    yaw_rate = -1 * MAX_YAW

                #print(f"rate: {yaw_rate}, step: {step} dev: {deviation}")

                if deviation > 16:
                    yaw_rate = MAX_YAW_MAX_DEV
                
                if deviation < -16:
                    yaw_rate = -MAX_YAW_MAX_DEV
                   
                
                
                my_share.put(RomiFlags.yaw_and_speed)
                my_queue.put(abs(int(V_TAN*1000)))
                my_queue.put(1)
                my_queue.put(int(8*1000))
                my_queue.put(int(abs(yaw_rate)*1000))
                
                if yaw_rate < 0:
                    my_queue.put(0)
                else:
                    my_queue.put(1)

            else:
                counter += 1

        #if button flag raised     
        if flag == RomiFlags.switch_A or flag == RomiFlags.switch_B:
            print("wall hit")
            my_share.put(RomiFlags.soft_disable_motor)
            my_queue.put(2)
            counter = 0
            state = 3
            #stop both motors
            #proceed to state 3

        #just a wait state
        if state == 3:
            counter += 1
            if counter > 10:
                state = 4

        if state == 4 and flag == RomiFlags.none:
            print("backing up")
            my_share.put(RomiFlags.trap_velocity)
            my_queue.clear()
            my_queue.put(2)
            my_queue.put(int(2*1000))
            my_queue.put(0)
            my_queue.put(int(max_accel*1000))
            my_queue.put(int(2*1000))
            my_queue.put(0)
            my_queue.put(int(max_accel*1000))
            counter = 0
            state = 5
        
        if state == 5:
            counter += 1
            if counter > 50:
                state = 6

        if state == 6:
            my_share.put(RomiFlags.soft_disable_motor)
            flag = None
            my_queue.put(2)
            state = 7

        if state == 7 and flag == RomiFlags.none:
            my_share.put(RomiFlags.turn_90)
            my_queue.put(1)
            state = 8
            print("turning 90")
            #turn 90 degrees

        if state == 8 and flag == RomiFlags.soft_disable_motor:
            print("recognized arrival")
            state = 9

        if state == 9 and flag == RomiFlags.none:
            #set an alarm
            print("set the yaw alarm")
            my_share.put(RomiFlags.yaw_alarm)
            flag = None
            my_queue.clear()
            my_queue.put(150)
            my_queue.put(0)
            state = 10

        
        if state == 10 and flag == RomiFlags.none:
            
            TURN_RADIUS = 13.7 #in
            WHEEL_VEL = 14 #rad/s
            TANGENTIAL_VEL = WHEEL_VEL*WHEEL_RADIUS

            TRACK_WIDTH = 5.5 #in   
            WHEEL_RADIUS = 1.375 #in

            YAW_RATE = TANGENTIAL_VEL/TURN_RADIUS

            my_share.put(RomiFlags.yaw_and_speed)
            my_queue.clear()
            my_queue.put(int(WHEEL_VEL*1000))
            my_queue.put(1)
            my_queue.put(int(8*1000))
            my_queue.put(abs(int(YAW_RATE*1000)))
            my_queue.put(0)
            flag = None
            state = 11

        if state == 11:
            if flag == RomiFlags.yaw_alarm_return:
                print("target heading reached")
                my_share.put(RomiFlags.soft_disable_motor)
                my_queue.put(2)
                state = 12
        
        if state == 12 and flag == RomiFlags.none:
            print("reached here")
            my_share.put(RomiFlags.trap_velocity)
            my_queue.put(2)
            my_queue.put(int(mot_A_vel*1000))
            my_queue.put(1)
            my_queue.put(int(max_accel*1000))
            my_queue.put(int(mot_B_vel*1000))
            my_queue.put(1)
            my_queue.put(int(max_accel*1000))
            yaw_rate = 0
            counter_two = 0
            state = 13
        
        if state == 13:
            #print("line follow again")
            if abs(deviation) < DEAD_ZONE:
                step = 0
            
            elif counter > 0:
                counter = 0
                step = MAX_STEP * 20 * deviation / MAX_DEV * delta_T 
                yaw_rate += step

                #saturate yaw rate 
                if yaw_rate > MAX_YAW:
                    yaw_rate = MAX_YAW
                if yaw_rate < -1 * MAX_YAW:
                    yaw_rate = -1 * MAX_YAW

                #print(f"dev: {deviation}")

                if deviation > 16:
                    yaw_rate = MAX_YAW_MAX_DEV
                
                if deviation < -16:
                    yaw_rate = -MAX_YAW_MAX_DEV
                   
                
                
                my_share.put(RomiFlags.yaw_and_speed)
                my_queue.clear()
                my_queue.put(abs(int(V_TAN*1000)))
                my_queue.put(1)
                my_queue.put(int(8*1000))
                my_queue.put(int(abs(yaw_rate)*1000))
                
                if yaw_rate < 0:
                    my_queue.put(0)
                else:
                    my_queue.put(1)

            else:
                counter += 1
            
            counter_two += 1
            if counter_two > 120:
                number_of_true_sens = 0
                for val in my_array.get_line():
                    if val is True:
                        number_of_true_sens += 1
                
                if number_of_true_sens >= 4:
                    print("finish detected")
                    my_share.put(RomiFlags.yaw_and_speed)
                    my_queue.clear()
                    my_queue.put(abs(int(V_TAN*1000)))
                    my_queue.put(1)
                    my_queue.put(int(8*1000))
                    my_queue.put(int(0))
                    my_queue.put(0)
                    state = 14
                    counter = 0
       
        if state == 14:
            print("state 14")
            

            counter += 1
            if counter > 40:
                my_share.put(RomiFlags.soft_disable_motor)
                my_queue.put(2)
                state = 15
                counter = 0

        if state == 15 and flag == RomiFlags.none:
            my_share.put(RomiFlags.return_to_initial_yaw)
      
            state = 16
            print("state 15")

        if state == 16 and flag == RomiFlags.soft_disable_motor:
            print("state 16")
            state = 17
        
        if state == 17 and flag == RomiFlags.none:
            print("should be facing correctly now")
            state = 18

        if state == 18:
            # my_share.put(RomiFlags.yaw_and_speed)
            # my_queue.clear()
            # my_queue.put(abs(int(V_TAN*1000)))
            # my_queue.put(1)
            # my_queue.put(int(8*1000))
            # my_queue.put(int(0))
            # my_queue.put(0)

            my_share.put(RomiFlags.set_motor_duty)
            my_queue.clear()
            my_queue.put(2)
            my_queue.put(40)
            my_queue.put(1)
            my_queue.put(42)
            my_queue.put(1)
            state = 19
            counter = 0
            
        if state == 19:
            counter += 1
            if counter > 20:
                state = 30
            
        if state == 30:
            if False not in my_array.get_line():
                state = 31
                counter = 0

        if state == 31:
            counter += 1
            if counter > 40:
                my_share.put(RomiFlags.soft_disable_motor)
                my_queue.put(2)
                state = 32
        if state == 32 and flag == RomiFlags.none:
            state = 1

        yield state




def maxacceltest(shares):
    '''@brief: controller task'''
   
    max_accel = 2 #rad/s^2
    state = 1
    print("Zero")
    yield state
    while True:
        my_share, my_queue = shares
        flag = my_share.get()
        if state == 1 and flag == RomiFlags.nucleo_button_press:
            print("A")
            my_share.put(RomiFlags.trap_velocity)
            flag = None
            my_queue.put(2)
            my_queue.put(int(15*1000))
            my_queue.put(1)
            my_queue.put(int(max_accel*1000))
            my_queue.put(int(15*1000))
            my_queue.put(1)
            my_queue.put(int(max_accel*1000))
            state = 2

        if state == 2 and flag == RomiFlags.nucleo_button_press:
            print("C")
            #soft disable both motors
            my_share.put(RomiFlags.soft_disable_motor)
            my_queue.put(2)
            state = 3

        if state == 3 and flag == RomiFlags.none:
            #calculate new max accel
            max_accel += 2
            print(f"new max accel: {max_accel}")
            state = 1

        yield state

def task1deprecated(shares):
    '''@brief: controller task'''
    my_share, my_queue = shares

    #Calculate mot_A and mot_B speeds
    tangential_speed = 2 #in/s
    radius = 12 #in
    WHEELBASE = 2.75 #in
    WHEEL_RADIUS = 1.375 #in
    THRESHOLD = 100 #EULER YAW (ticks)
    SEMICIRCLE_ANGLE = 2880 #(ticks)

    chirality = "CCW"
    v_A = tangential_speed/radius*(radius+WHEELBASE)/WHEEL_RADIUS
    v_B = tangential_speed/radius*(radius-WHEELBASE)/WHEEL_RADIUS


    if chirality == "CW":
        v_A, v_B = v_B, v_A

    i2c = I2C(1, I2C.CONTROLLER, baudrate=100000)
    utime.sleep_ms(100)
    imu = BNO055(i2c)

    initial_eul_yaw = 0
    print("Done with Initial Calibration and Setup")
    imu.change_mode("IMU")
    completed_half_circle = False

    state = 1
    yield state

    while True:
        flag = my_share.get()

        #if button is pressed and unpressed, get current Euler and set speed for both motors
        if state == 1:
            #print(imu.get_cal_status())
            #polls the EULER HEADING, or yaw, everytime that the system enters this part of the loop
            #print(f"EUL yaw: {EUL_yaw}")
            pass

        #in this state, the system will wait for the nucleo button to be pressed before it sets Motor A on
        if state == 1 and flag == RomiFlags.nucleo_button_press:
            #takes initial EULER YAW measurement to compare with future values for stopping the system
            initial_eul_yaw = imu.get_EUL_heading()
            my_share.put(RomiFlags.set_motor_speed)
            my_queue.put(0)
            my_queue.put(int(v_A*1000))
            my_queue.put(1)
            state = 2
            #automatic transition to set Motor B on as well

        #happens seamlessly
        if state == 2 and flag == RomiFlags.none:
            my_share.put(RomiFlags.set_motor_speed)
            my_queue.put(1)
            my_queue.put(int(v_B*1000))
            my_queue.put(1)
            state = 3


        #this state keeps track of Romi's progress around the circle
        if state == 3:
            #check for semicircle travel
            eul_yaw = imu.get_EUL_heading()
            
            diff_yaw = eul_yaw - initial_eul_yaw

            if (diff_yaw > SEMICIRCLE_ANGLE):
                diff_yaw -= SEMICIRCLE_ANGLE*2
            elif (diff_yaw < -1 * SEMICIRCLE_ANGLE):
                diff_yaw += SEMICIRCLE_ANGLE*2

            withinThreshold = abs(diff_yaw) < THRESHOLD
            print(f"EUL: {eul_yaw}, init: {initial_eul_yaw}, diff: {diff_yaw}, semi:{completed_half_circle}, within: {withinThreshold}")
            if abs(diff_yaw) >= 400:
                completed_half_circle = True

            if completed_half_circle and withinThreshold:
                print("AROUND THE WORLD")
                state = 4

        #in the case that the motors were indeed turned on, the motors now need to be turned off because
        #the ROMI has now reached the EULER Yaw of 0, which indicates a full rotation around the circle.
        if state == 4 and flag == RomiFlags.none:
            print("stop A")
            my_share.put(RomiFlags.disable_motor)
            my_queue.put(2)
            state = 5
        
        if state == 5 and flag == RomiFlags.none:
            completed_half_circle = False
            state = 1

        yield state

#task 2: motor pwm setting, encoder updates, and CL control calculations
def task2(shares):
    '''@brief: task for motor pwm setting, encoder updates, and CL control calculations'''
    state = 0
    my_share, my_queue = shares
    
#Motor A
    #Left Side Encoder
       #PA0 Tim5.1, 
       #PA1 Tim5.2 

    #Left Side Motor
       #PC6 Tim3.1
       #PB13  Enable
       #PB14  Direction

#Motor B
    #Right Side Encoder
       #PB4 Tim3.1
       #PB5 Tim3.2 

    #Right Side Motor 
       #PB6 Tim4.1
       #PB11  Enable
       #PB12  Direction 

    tim_A = Timer(2, freq = 20_000)
    mot_A = L6206(tim_A, Pin.cpu.A5, Pin.cpu.B13, Pin.cpu.B14)

    tim_B = Timer(4, freq = 20_000)
    mot_B = L6206(tim_B, Pin.cpu.B6, Pin.cpu.B11, Pin.cpu.B12)
   
    mot_A.set_encoder(5, Pin.cpu.A0, Pin.cpu.A1)
    mot_B.set_encoder(3, Pin.cpu.B4, Pin.cpu.B5)
    mot_A.enable()
    mot_B.enable()

    i2c = I2C(1, I2C.CONTROLLER, baudrate=100000)
    utime.sleep_ms(100)
    imu = BNO055(i2c)

    print("Done with Initial Calibration and Setup")
    imu.change_mode("IMU")
    error_sum = 0

    stream_output = False
    mot_A_state = 1
    mot_B_state = 1

    print_count = 0
    alarm_heading = 0
    target_alarm_heading = 0

    state = mot_A_state << 4 | mot_B_state
    while True:  
        flag = my_share.get()
        mot_A.update()
        mot_B.update()

        #get the imu rotational vel
        #print(f"{imu.get_GYR_Z()}")


        

        if stream_output:
            print(f"enc A: {mot_A.get_encoder().get_speed()}")
            print(f"enc B: {mot_B.get_encoder().get_speed()}")

        if flag == RomiFlags.none:
            #print("None")
            #donothing
            pass
        elif flag == RomiFlags.start_motor:
            print("start")
            motor_name = my_queue.get()
            if motor_name == 0:
                mot_A.enable()
                mot_A_state = 1
            if motor_name == 1:
                mot_B.enable()
                mot_B_state = 1
            if motor_name == 2:
                mot_A.enable()
                mot_B.enable()
                mot_A_state = 1
                mot_B_state = 1
            my_share.put(RomiFlags.none)
        
        elif flag == RomiFlags.set_motor_duty:
            print("set duty")
            motor_name = my_queue.get()
            duty_cycle = my_queue.get()
            direction = my_queue.get()

           
            if direction == 1:
                duty_cycle = -duty_cycle
            if motor_name ==  0:
                if mot_A_state != 0:
                    mot_A.soft_disable()
                    mot_A.set_duty(duty_cycle)
                    mot_A_state = 1
                else:
                    print("motor A currently disabled")
            if motor_name == 1:
                if mot_B_state != 0:
                    mot_B.soft_disable()
                    mot_B.set_duty(duty_cycle)
                    mot_B_state = 1
                else:
                    print("motor B currently disabled")
            if motor_name == 2:
                mot_A.soft_disable()
                mot_A.set_duty(duty_cycle)
                mot_A_state = 1
                duty_cycle_2 = my_queue.get()
                direction_2 = my_queue.get()
                if direction_2 == 1:
                    duty_cycle_2 = -duty_cycle_2
                mot_B.soft_disable()
                mot_B.set_duty(duty_cycle_2)
                mot_B_state = 1
            else:
                print("motor name error")
            my_share.put(RomiFlags.none)
            print("set finished")

        elif flag == RomiFlags.set_motor_speed:
            print("set speed")
            motor_name = my_queue.get()
            #divide by 1000 here because it was multiplied by 1000 earlier
            speed = my_queue.get()/1000
            direction = my_queue.get()
            

            if direction == 1:
                speed = -speed

            if motor_name ==  0:
                if mot_A_state != 0:
                    mot_A.set_speed(speed)
                    mot_A.reset_PID_error()
                    mot_A_state = 1
                else:
                    print("motor A currently disabled")
            if motor_name == 1:
                if mot_B_state != 0:
                    mot_B.set_speed(speed)
                    mot_B.reset_PID_error()
                    mot_B_state = 1
                else:
                    print("motor B currently disabled")


            my_share.put(RomiFlags.none)
            print("set finished")

        
        elif flag == RomiFlags.disable_motor:
            print("stop")
            motor_name = my_queue.get()
            if motor_name == 0:
                    mot_A.disable()
                    mot_A_state = 0
            if motor_name == 1:
                    mot_B.disable()
                    mot_B_state = 0
            if motor_name == 2:
                    mot_A.disable()
                    mot_B.disable()
                    mot_A_state = 0
                    mot_B_state = 0
            my_share.put(RomiFlags.none)

    
        elif flag == RomiFlags.soft_disable_motor:
            print("stop")
            motor_name = my_queue.get()
            if motor_name == 0:
                    mot_A.soft_disable()
                    mot_A_state = 1
            if motor_name == 1:
                    mot_B.soft_disable()
                    mot_B_state = 1
            if motor_name == 2:
                    mot_A.soft_disable()
                    mot_B.soft_disable()
                    mot_A_state = 1
                    mot_B_state = 1
            my_share.put(RomiFlags.none)

        elif flag == RomiFlags.toggle_output_stream:
            stream_output = not stream_output
            my_share.put(RomiFlags.none)
        
        elif flag == RomiFlags.turn_90:
            #mark current heading
            #calc target heading
            #set alarm (internal mode)
            direction = my_queue.get()
            current_heading = imu.get_EUL_heading()
            if direction == 1:
                alarm_heading = (current_heading + 1440)
            elif direction == 0:
                alarm_heading = (current_heading - 1440)

            if alarm_heading > 5760:
                alarm_heading -= 5760
            elif alarm_heading < 0:
                alarm_heading += 5760
                

            #set yaw and speed
            my_share.put(RomiFlags.yaw_and_speed)
            my_queue.clear()
            my_queue.put(int(0))
            my_queue.put(1)
            my_queue.put(int(8*1000))
            my_queue.put(int(2*1000))
            my_queue.put(1)
            #soft disable once the thing is done
    

        elif flag == RomiFlags.initial_yaw_capture:
            initial_yaw = imu.get_EUL_heading()+20
            my_share.put(RomiFlags.none)
          
        elif flag == RomiFlags.return_to_initial_yaw:
            #mark current heading
            #calc target heading
            #set alarm (internal mode)
            alarm_heading = initial_yaw
                

            #set yaw and speed
            my_share.put(RomiFlags.yaw_and_speed)
            my_queue.clear()
            my_queue.put(int(0))
            my_queue.put(1)
            my_queue.put(int(8*1000))
            my_queue.put(int(1*1000))
            my_queue.put(1)
            #soft disable once the thing is done


        elif flag == RomiFlags.yaw_alarm:
            degrees = my_queue.get()
            direction = my_queue.get()

            if direction == 0:
                degrees = -degrees
            
            current_alarm_heading = imu.get_EUL_heading()
            target_alarm_heading = current_alarm_heading + degrees*16


            #print(f"current: {current_alarm_heading}, target: {target_alarm_heading}")

            if target_alarm_heading > 5760:
                target_alarm_heading -= 5760
            elif target_alarm_heading < 0:
                target_alarm_heading += 5760
            my_share.put(RomiFlags.none)

        
        elif flag == RomiFlags.trap_velocity:
            #extract the following: motor_name, target speed, direction,  max accel
            
            motor_name = my_queue.get()
            target_speed = my_queue.get()/1000 #rad/s
            direction = my_queue.get()
            max_accel = my_queue.get()/1000 #rad/s^2
            delta_T = 0.020 #sec
            target_speed_two = 0

            if motor_name == 2:
                #print("sensed double set")
                target_speed_two = my_queue.get()/1000 #rad/s
                direction_two = my_queue.get()
                max_accel_two = my_queue.get()/1000 #rad/s^2
                if direction_two == 1:
                    target_speed_two = -target_speed_two
            
            print(f"Command received.\nMotor B, speed {target_speed_two}, max accel {max_accel}")
            my_share.put(RomiFlags.none)
            
            print(f"Command received.\nMotor {motor_name}, speed {target_speed}, max accel {max_accel}")
            if direction == 1:
                target_speed = -target_speed

            if motor_name == 0 or motor_name == 2:
                speed_tracker_A = mot_A.get_encoder().get_speed()
                step_A = delta_T * max_accel
                target_speed_A = target_speed
                mot_A.reset_PID_error()
                mot_A_state = 2
            if motor_name == 1:
                speed_tracker_B = mot_B.get_encoder().get_speed()
                step_B = delta_T * max_accel
                target_speed_B = target_speed
                mot_B.reset_PID_error()
                mot_B_state = 2
            elif motor_name == 2:
                speed_tracker_B = mot_B.get_encoder().get_speed()
                step_B = delta_T * max_accel_two
                target_speed_B = target_speed_two
                mot_B.reset_PID_error()
                mot_B_state = 2


        elif flag == RomiFlags.yaw_and_speed:
            
            
            TRACK_WIDTH = 5.5 #in   
            WHEEL_RADIUS = 1.375 #in

            target_speed = my_queue.get()/1000 #rad/s
            direction = my_queue.get()
            max_accel = my_queue.get()/1000 #rad/s^2

            if direction == 1:
                target_speed = - target_speed
            
            target_yaw_rate = my_queue.get()/1000 #rad/s
            yaw_rate_direction = my_queue.get()

           

            if yaw_rate_direction == 1:
                target_yaw_rate = -target_yaw_rate

            
            

            #calculate average and differential velocity of wheels
            differential_vel = target_yaw_rate*TRACK_WIDTH/2/WHEEL_RADIUS #rad/s
            average_vel = target_speed/WHEEL_RADIUS  #rad/s

            delta_T = 0.020 #sec


            mot_A_state = 3
            mot_B_state = 3

            speed_tracker_A = mot_A.get_encoder().get_speed()
            speed_tracker_B = mot_B.get_encoder().get_speed()

            target_speed_A = average_vel + differential_vel
            target_speed_B = average_vel - differential_vel

            mot_A.set_speed(target_speed_A)
            mot_B.set_speed(target_speed_B)
            #print(f"target A: {target_speed_A}, target B: {target_speed_B}")

            step_A = delta_T * max_accel
            step_B = delta_T * max_accel
            my_share.put(RomiFlags.none)

        

        if mot_A_state == 3 and mot_B_state == 3:
            K_i = 0.02
            K_p = 0.1

            print_count += 1
            if print_count == 10:

                max_effort = 5 #rad/s
                current_yaw_rate = imu.get_GYR_Z()
                error = target_yaw_rate-current_yaw_rate #rad/s
                error_sum += error
                effort = K_p * error + K_i*error_sum

                #saturate
                if effort > max_effort:
                    effort = max_effort
                if effort < -max_effort:
                    effort = -max_effort


                target_speed_A = average_vel + differential_vel + effort
                target_speed_B = average_vel - differential_vel - effort

                mot_A.set_speed(target_speed_A)
                mot_B.set_speed(target_speed_B)
                
                #print(f"error: {error}, effort: {effort}, yaw_rate: {current_yaw_rate}")
                print_count = 0


        if mot_A_state == 2:
            if speed_tracker_A+step_A < target_speed_A:
                speed_tracker_A += step_A
                mot_A.reset_PID_error()
            elif speed_tracker_A - step_A > target_speed_A:
                speed_tracker_A -= step_A
                mot_A.reset_PID_error()
            else:
                speed_tracker_A = target_speed_A
            #print(f"A speed: {speed_tracker_A}, target {target_speed_A}, step: {step_A}")
            mot_A.set_speed(speed_tracker_A)
        
            
        if mot_B_state == 2:
            if speed_tracker_B+step_B < target_speed_B:
                speed_tracker_B += step_B
                mot_B.reset_PID_error()
            elif speed_tracker_B - step_B > target_speed_B:
                speed_tracker_B -= step_B
                mot_B.reset_PID_error()
            else:
                speed_tracker_B = target_speed_B
            #print(f"B speed: {speed_tracker_B}, target: {target_speed_B}, step: {step_B}")
            mot_B.set_speed(speed_tracker_B)
        

        if alarm_heading != 0:
            poll_heading = imu.get_EUL_heading()
            #print(f"current: {poll_heading}, target: {alarm_heading}")
            if alarm_heading + 50 > poll_heading and alarm_heading - 50 < poll_heading:
               print(f"arrived! at: {alarm_heading}, actually {poll_heading}")
               alarm_heading = 0
               my_share.put(RomiFlags.soft_disable_motor)
               my_queue.put(2)
        

        if target_alarm_heading != 0:
            poll_heading = imu.get_EUL_heading()
            #print(f"current: {poll_heading}, target: {alarm_heading}")
            if target_alarm_heading + 100 > poll_heading and target_alarm_heading - 100 < poll_heading:
                print("arrived at alarm")
                target_alarm_heading = 0
                my_share.put(RomiFlags.yaw_alarm_return)

        state = mot_A_state << 4 | mot_B_state
        yield state

def task3(shares):
    '''@brief: User input stuff'''
    my_share, my_queue = shares
    flag = my_share.get()
    
    state = 1
    Button = Pin(Pin.cpu.C13, mode = Pin.IN)  #This pin is effectively pulled high when the button isn't pressed
    switch_A = Pin(Pin.cpu.H0, mode = Pin.IN, pull=Pin.PULL_DOWN)
    switch_B = Pin(Pin.cpu.H1, mode = Pin.IN, pull=Pin.PULL_DOWN)
    switch_A_pressed = 0
    switch_B_pressed = 0

    yield state
    while True:

         #if button is pressed and unpressed, send flag to 
        if state == 1 and Button.value() == 0:
            state = 2
        if state == 2 and Button.value() == 1:
            if my_share.get() == RomiFlags.none:
                state = 1
                print("placed flag")
                my_share.put(RomiFlags.nucleo_button_press)
            else:
                print("Attempting to set button flag but share is full")
            
        if switch_A.value() == 1 and switch_A_pressed == 0:
            switch_A_pressed = 1
        if switch_B.value() == 1 and switch_B_pressed == 0:
            switch_B_pressed = 1

        if switch_A_pressed is 1: 
            if flag == RomiFlags.none:
                my_share.put(RomiFlags.switch_A)
                flag = None 
                switch_A_pressed = 2
            else:
                print("switch A pressed but flag full")
        if switch_A_pressed == 2 and switch_A.value() == 0:
            switch_A_pressed = 0
            switch_B_pressed = 0

        if switch_B_pressed is 1:
            if flag == RomiFlags.none:
                my_share.put(RomiFlags.switch_B)
                flag = None
                switch_B_pressed = 2
            else:
                print("switch B is pressed but flag full")
        if switch_B_pressed == 2 and switch_B.value() == 0:
            switch_B_pressed = 0
            switch_A_pressed = 0

        yield state


#continue function, repeatedly calls schedule (useful for continuing program after keyboard interrupt)
def continue_run():
    '''!@brief Method which continues calling the scheduler'''
    while True:
        try:
            cotask.task_list.pri_sched()
        except KeyboardInterrupt:
            print("Detected a keyboard break \nCall continue_run() to continue program")
            break

  
def toggle_output_stream():
    share0.put(RomiFlags.toggle_output_stream)
    continue_run()

def start_motor(motor_name):
    '''!@brief Sets flags for task2 to enable the specified motor'''
    print("START")
    if type(motor_name) is not str:
        raise TypeError("motor name must be string")
    share0.put(RomiFlags.start_motor)
    if motor_name == 'A':
        q0.put(0)
    else:
        q0.put(1)
    continue_run()

def set_motor_duty(motor_name, duty_cycle):
    '''!@brief Sets flags for task2 to set PWM for specified motor'''
    print("DUTY")
    if type(motor_name) is not str:
        raise TypeError("motor name must be string")

    share0.put(RomiFlags.set_motor_duty)
    #fill queue params [motor_name, duty_cycle, direction]
    if motor_name == 'A':
        q0.put(0)
    else:
        q0.put(1)
    q0.put(abs(duty_cycle))
    if duty_cycle < 0:
        q0.put(0)
    else:
        q0.put(1)
    continue_run()

def set_motor_speed(motor_name, speed):
    '''!@brief Sets flags for task2 to set PID control target for specified motor'''
    print("SPEED")
    if type(motor_name) is not str:
        raise TypeError("motor name must be string")
    
    share0.put(RomiFlags.set_motor_speed)
    #fill queue params [motor_name, speed, direction]
    if motor_name == 'A':
        q0.put(0)
    else:
        q0.put(1)

    #multiple by 1000 now, divide by 1000 later
    speed = int(speed*1000)
    
    q0.put(abs(speed))
    if speed < 0:
        q0.put(0)
    else:
        q0.put(1)
    continue_run()

def disable_motor(motor_name):
    share0.put(RomiFlags.disable_motor)
    if motor_name == 'A':
        q0.put(0)
    else:
        q0.put(1)
    continue_run()

def soft_disable_motor(motor_name):
    share0.put(RomiFlags.soft_disable_motor)
    if motor_name == 'A':
        q0.put(0)
    elif motor_name == 'B':
        q0.put(1)
    else:
        q0.put(2)
    continue_run()

def set_trap_speed(motor_name, target_speed, max_accel):
    '''@brief: set motor task to follow a trapezoidal speed profile '''

    share0.put(RomiFlags.trap_velocity)
    if motor_name == 'A':
        q0.put(0)
    else:
        q0.put(1)

    target_speed = int(target_speed*1000)
    q0.put(abs(target_speed))

    if target_speed < 0:
        q0.put(0)
    else:
        q0.put(1)

    max_accel = int(max_accel*1000)
    q0.put(abs(max_accel))
    continue_run()

def set_yaw_and_speed(target_yaw_rate, target_speed):
    max_accel = 8 #rad/s^2
    share0.put(RomiFlags.yaw_and_speed)
    q0.put(int(abs(target_speed)*1000))
    if target_speed < 0:
        q0.put(0)
    else:
        q0.put(1)
    q0.put(int(max_accel*1000))

    q0.put(int(abs(target_yaw_rate)*1000))
    if target_yaw_rate < 0:
        q0.put(0)
    else:
        q0.put(1)
    continue_run()




def print_trace():
    print('\n' + str (cotask.task_list))
    print(task_share.show_all())
    print(task_1.get_trace())
    print('')

def test():
    print(hex(id(share0)))
#main
if __name__ == '__main__':

    setup()

    # UART 1 Uses B6 and B7 by default
    ser = UART(3, 115200 , timeout=1000)

    # Deconfigure default pins
    Pin(Pin.cpu.C4,  mode=Pin.ANALOG)     # Set pin modes back to default
    Pin(Pin.cpu.C5,  mode=Pin.ANALOG)

    # Configure the selected pins in coordination with the alternate function table
    Pin(Pin.cpu.C4,  mode=Pin.ALT, alt=7) # Set pin modes to UART matching column 7 in alt. fcn. table
    Pin(Pin.cpu.C5, mode=Pin.ALT, alt=7)
    pyb.repl_uart(ser)


    #Create a share and a queue
    share0 = task_share.Share('h', thread_protect=False, name="Share 0")
    print(hex(id(share0)))
    test()
    q0 = task_share.Queue('L', 16, thread_protect=False, overwrite=False,
                          name="Queue 0")
    #Create tasks
    task_1 = cotask.Task(task1, name="Task_1", priority=1, period=20,
                        profile=True, trace=False, shares=(share0, q0))
    task_2 = cotask.Task(task2, name="Task_2", priority=2, period=20,
                        profile=True, trace=False, shares=(share0, q0))
    task_3 = cotask.Task(task3, name="Task_3", priority=2, period=200,
                        profile=True, trace=False, shares=(share0, q0))
    
    cotask.task_list.append(task_1)
    cotask.task_list.append(task_2)
    cotask.task_list.append(task_3)

    gc.collect()

    #try repeatedly call scheduler
    #if keyboard interrupt, stop running program
    while True:
        try:
            cotask.task_list.pri_sched()
        except KeyboardInterrupt:
            print("Detected an keyboard break \nCall continue_run() to continue program")
            break
    
    
   
#the current issue: I want to expose certain functions such as setting PWM 
#so that the user can exert control when they choose to. This is useful for debugging

#however, we are running into the issue of variable scope, 
#since certain variables can only be accessed within the task

#solution: use share and queue for flags and data respectively. 
#create methods that populate the flags and data that are public




#PB8 SCL, PB9 SDA