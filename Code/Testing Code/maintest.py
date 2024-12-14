#notes about lab structure:


#Use the cotask class to create a list of tasks (which are generator methods) that run cooperatively
#Each generator method must take in a tuple of (share, queue)
#Each generator method must yield the next state number for that task, this history stored in Task.trace
#Because it is a generator method, the state variable continues to exist within that task between function calls
#Both share and queue function as queues. Idk how they're different



#PSEUDO CODE  

#import statements
from pyb import Pin, Timer
from src.romi_flags import RomiFlags
#import keyboard
from src.motor2 import L6206
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
    #keyboard.on_press (on_keypress)
    global pushed_key
    pushed_key=''
    pass
    #not sure if we should do all the definitions here, or in state 0 of task 2?
    #create motor objects
    #create encoder objects
    #enable everything, but don't run the motors
   

#task 1: user input
def task1(shares):
    '''@brief: user input task'''
    my_share, my_queue = shares

   
    state = 1
    while True:
        #state 1:
        #if pushed_key = a, print to serial a list of input options the user has
        #set state to 2
        if pushed_key == 'w':
            print("you have pressed the wake key, further functionality has not been implemented")
            state = 2
            #pause the program so user can type commands?
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
       #PA5 Tim2.1
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
    mot_A = L6206(tim_A, Pin.cpu.A5, Pin.cpu.B14, Pin.cpu.B13)

    tim_B = Timer(4, freq = 20_000)
    mot_B = L6206(tim_B, Pin.cpu.B6, Pin.cpu.B12, Pin.cpu.B11)
   
    mot_A.set_encoder(5, Pin.cpu.A0, Pin.cpu.A1)
    mot_B.set_encoder(3, Pin.cpu.B4, Pin.cpu.B5)
    mot_A.enable()
    mot_B.enable()

    stream_output = True

    state = 1
    while True:  
        flag = my_share.get()
        mot_A.update()
        mot_B.update()
        print("about to check")


        if stream_output:
            print(f"enc A: {mot_A.get_encoder().get_delta()}")
            print(f"enc B: {mot_B.get_encoder().get_delta()}")
        if flag == RomiFlags.none:
            print("None")
            #donothing
            pass
        elif flag == RomiFlags.start_motor:
            print("start")
            motor_name = my_queue.get()
            if motor_name == 0:
                mot_A.enable()
            if motor_name == 1:
                mot_B.enable()
            my_share.put(RomiFlags.none)
        
        elif flag == RomiFlags.set_motor:
            print("set")
            motor_name = my_queue.get()
            duty_cycle = my_queue.get()
            if motor_name ==  0:
                mot_A.set_duty(duty_cycle)
            if motor_name == 1:
                mot_B.set_duty(duty_cycle)
            my_share.put(RomiFlags.none)
            print("set finished")
        
        elif flag == RomiFlags.disable_motor:
            print("stop")
            motor_name = my_queue.get()
            if motor_name == 0:
                    mot_A.disable()
            if motor_name == 1:
                    mot_B.disable()
            my_share.put(RomiFlags.none)
        elif flag == RomiFlags.toggle_output_stream:
            stream_output = not stream_output
    
        yield state

def task3(shares):
    '''@brief: IMU stuff'''
    print("periodic test")
    

#various callbacks here
def on_keypress(event):
    global pushed_key
    pushed_key = event.name


#continue function, repeatedly calls schedule (useful for continuing program after keyboard interrupt)
def continue_run():
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
    print("START")
    if type(motor_name) is not str:
        raise TypeError("motor name must be string")
    share0.put(RomiFlags.start_motor)
    if motor_name == 'A':
        q0.put(0)
    else:
        q0.put(1)
    continue_run()

def set_motor(motor_name, duty_cycle):
    print("SET")
    if type(motor_name) is not str:
        raise TypeError("motor name must be string")
    
    share0.put(RomiFlags.set_motor)
    if motor_name == 'A':
        q0.put(0)
    else:
        q0.put(1)
    
    q0.put(duty_cycle)
    print(q0.any())
    continue_run()

def disable_motor(motor_name):
    share0.put(RomiFlags.disable_motor)
    if motor_name == 'A':
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
    

    #Create a share and a queue
    share0 = task_share.Share('h', thread_protect=False, name="Share 0")
    print(hex(id(share0)))
    test()
    q0 = task_share.Queue('L', 16, thread_protect=False, overwrite=False,
                          name="Queue 0")
    #Create tasks
    task_1 = cotask.Task(task1, name="Task_1", priority=1, period=400,
                        profile=True, trace=False, shares=(share0, q0))
    task_2 = cotask.Task(task2, name="Task_2", priority=2, period=1500,
                        profile=True, trace=False, shares=(share0, q0))
    
    cotask.task_list.append(task_1)
    cotask.task_list.append(task_2)

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