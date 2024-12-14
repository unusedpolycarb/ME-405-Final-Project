'''!@file encoder.py
@brief A driver for reading from Quadrature Encoders
@details
@author Ritvik Dutta, Vincent Ngo
@date October 10, 2024
'''
from pyb import Pin, Timer, disable_irq, enable_irq
import time


class L6206:
    '''DC Motor driver, uses PWM, you can set the duty cycle'''

    def __init__ (self, PWM_tim, IN1_pin, IN2_pin, EN_pin):
    #Defines global objects for rest of class to use
    #Inputs required:
    #   Timer that already has a frequency
    #   Three pins already defined

        disable_irq() #prevent silly nonsense

        self.IN1_pin = Pin(IN1_pin, mode=Pin.OUT_PP)
        self.IN2_pin = Pin(IN2_pin, mode=Pin.OUT_PP)
        self.EN_pin = Pin(EN_pin, mode=Pin.OUT_PP)
        self.EN_pin.low()  #good safety measure, ensure pin off
        #create timer channel from timer
        self.PID_enable = False
        self.PWM1 = PWM_tim.channel(1, mode = Timer.PWM, pin=self.IN1_pin, pulse_width_percent=0)
        self.PWM2 = PWM_tim.channel(2, mode = Timer.PWM, pin=self.IN2_pin, pulse_width_percent=0)
        self.duty = 0
        enable_irq(True)

    def set_duty (self, duty):
        '''!@brief Set the PWM duty cycle for the DC motor.
        @details This method sets the duty cycle to be sent
        to the L6206 to a given level. Positive values
        cause effort in one direction, negative values
        in the opposite direction.
        @param duty A signed number holding the duty
        cycle of the PWM signal sent to the L6206
        '''
        self.duty = duty
        #if duty is positive, set In2 to zero (reverses)
        if(duty >= 0 ):
            self.PWM2.pulse_width_percent(0)
            self.PWM1.pulse_width_percent(abs(duty))
        #if duty is negative, set IN1 to zero (reverses)
        else:
            self.PWM1.pulse_width_percent(0)
            self.PWM2.pulse_width_percent(abs(duty))
    


    def enable(self):
        #turns the enable pin on
        self.EN_pin.value(1)

    def disable(self):
        #turns the enable pin off
        self.EN_pin.value(0)

    class Encoder:
        '''!@brief Interface with quadrature encoders
        @details Utilizes external timers to count encoder ticks
        update() must be called at least once per timer overflow for accurate readings
        '''
        AR = 65355
        PS = 0
        

        def __init__(self, timernumber, pin1, pin2):
            '''!@brief Constructs an encoder object
            @details
            @param
            @param
            @param
            '''

            #sets timer
            self.timernumber = timernumber
            self.timer = Timer(timernumber, period = L6206.Encoder.AR, prescaler = L6206.Encoder.PS)
            #sets timer channels, which connects pins to timer
            self.timer.channel(1, pin = pin1, mode = Timer.ENC_AB)
            self.timer.channel(2, pin = pin2, mode = Timer.ENC_AB)

            #zeros the position
            self.abs_position = 0
            self.prev_position = 0
            self.cur_position = 0
            self.delta = 0

        def update(self):
            '''!@brief Updates encoder position and delta, should be called often
            @details If PID is enabled, will also calculate correction of speed
            '''
            self.prev_position = self.abs_position   #moves last reading into previous position variable 
            self.cur_position = self.timer.counter()   #takes new position reading
            self.delta = self.cur_position - self.prev_position   #calculates delta between current and previous reading

            if self.delta > (L6206.Encoder.AR+1)//2:
                self.delta -= (L6206.Encoder.AR+1)

            elif self.delta < (L6206.Encoder.AR+1)//2:
                self.delta += (L6206.Encoder.AR+1)

            self.abs_position += self.delta            #

            self.update_PID()

        def update_PID(self):

            pass
        def get_position(self):
            '''!@brief Gets the most recent encoder position
            '''
            return self.abs_position

        def get_delta(self):
            '''!@brief Gets the most recent encoder delta           
            '''
            return self.delta

        def zero(self):
            '''!@brief Resets the encoder position to zero
            '''
            self.abs_position = 0

    def set_encoder(self, timernumber, pin1, pin2):
        self.encoder = L6206.Encoder(timernumber, pin1, pin2)

    def get_encoder(self):
        return self.encoder
    
    def update(self):
        self.encoder.update()

    def set_speed(self, speed):
        '''@brief Sets PWM according to target speed (in rad/s), begins PID control'''
        if self.encoder is None:
            return ValueError("No encoder attached")
        self.PID_enable = True
        self.target = speed #TODO
    





##def timercb(timer):
##    global encoder_A, encoder_B
##    encoder_B.update()
##    encoder_A.update()
    
def collectCallback(timer):
    #callback to collect and print data
    global timestamp, dataStore, encTarget
    
    if timestamp < 3000:
        print(f"{timestamp/1000},{encTarget.get_position()},{encTarget.get_delta()}")
    timestamp += 1

def collectData(target):
    global timestamp, dataStore, encTarget
    #call this method to set duty and collect data
    timer7 = Timer(7, freq = 1_000)
    timestamp = 0
    if target == 'A':
            mot_A.set_duty(30)
            encTarget = encoder_A
            timer7.callback(collectCallback)
    elif target == 'B':
            mot_B.set_duty(40) 
            encTarget = encoder_B
            timer7.callback(collectCallback)
    
    time.sleep(2)
    timer7.deinit()
    mot_A.set_duty(0)
    mot_B.set_duty(0)



if __name__ == '__main__':

    tim_B = Timer(2, freq = 20_000)
    tim_A = Timer(3, freq = 20_000)

    mot_B = L6206(tim_B, Pin.cpu.A0, Pin.cpu.A1, Pin.cpu.C1)
    mot_A = L6206(tim_A, Pin.cpu.B4, Pin.cpu.B5, Pin.cpu.A10)
    mot_A.set_encoder(8, Pin.cpu.C6, Pin.cpu.C7)
    mot_B.set_encoder(4, Pin.cpu.B6, Pin.cpu.B7)

    mot_A.enable()
    mot_B.enable()

    interval = 1_000_000
    poll_delay_ms = 1

    Button = Pin(Pin.cpu.C13, mode = Pin.IN)  #This pin is effectively pulled high when the button isn't pressed
    button_toggle = 0

    while True:
        mot_A.update()
        mot_B.update()
        
        #senses button press and release
        if Button.value() == 0:
            button_toggle = 1
        if Button.value() == 1 and button_toggle == 1:
            start    = time.ticks_us()   
            deadline = time.ticks_add(start, interval)
            mot_B.set_duty(100)
            data_store = []
            button_toggle = 2

        if button_toggle == 2:  
            time.sleep_ms(poll_delay_ms) 
            now = time.ticks_us()
            #print(mot_A.get_encoder().get_position())
            data_store.append((time.ticks_diff(now, start), mot_B.get_encoder().get_position(), mot_B.get_encoder().get_delta()))
            
            if time.ticks_diff(deadline, now) <= 0:
                
                button_toggle = 0
               
                mot_B.set_duty(0)
                #print(data_store)

                print("BEGIN FILE")
                for entry in data_store:
                    print(f"{entry[0]}, {entry[1]}, {entry[2]}")

    #deadline = time.ticks_add(start, interval)       # first run deadline
    
    #encTarget = encoder_A