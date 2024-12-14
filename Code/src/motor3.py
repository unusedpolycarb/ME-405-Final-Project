'''!@file encoder.py
@brief A driver for reading from Quadrature Encoders
@details
@author Ritvik Dutta, Vincent Ngo
@date October 10, 2024
'''
from pyb import Pin, Timer, disable_irq, enable_irq
import utime


class L6206:
    '''DC Motor driver, uses PWM, you can set the duty cycle and speed'''

    K_V = 4 #rad/s-V
    SUPPLY_VOLTAGE = 7.2 #Volts
    MAX_VEL = SUPPLY_VOLTAGE*K_V

    #K_ff = 0
    #K_p = 0.6
    #K_i = 0.6
    #K_d = 0.1

    K_ff = 1
    K_p = 1.8
    K_i = 1.1
    K_d = 0.01
    PERIOD = 0.2 #sec

    MAX_CONTRL_EFFORT = MAX_VEL
    def __init__ (self, PWM_tim, IN1_pin, DIR_pin, EN_pin):
    #Defines global objects for rest of class to use
    #Inputs required:
    #   Timer that already has a frequency
    #   Three pins already defined

        disable_irq() #prevent silly nonsense

        self.IN1_pin = Pin(IN1_pin, mode=Pin.OUT_PP)
        self.DIR_pin = Pin(DIR_pin, mode=Pin.OUT_PP)
        self.EN_pin = Pin(EN_pin, mode=Pin.OUT_PP)
        self.EN_pin.low()  #good safety measure, ensure pin off
        self.DIR_pin.high()
        self.encoder = None
        
        #Set PID control vars
        self.PID_enable = False
        self.target_speed = 0
        self.error_was = 0
        self.error_sum = 0

        #create timer channel from timer
        self.PWM1 = PWM_tim.channel(1, mode = Timer.PWM, pin=self.IN1_pin, pulse_width_percent=0)
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
        #print(f"duty: {duty}")
        #if duty is positive, set DIR to one 
        if(duty >= 0 ):
            self.DIR_pin.high()
            self.PWM1.pulse_width_percent(abs(duty))
        #if duty is negative, set DIR to zero (reverses)
        else:
            self.DIR_pin.low()
            self.PWM1.pulse_width_percent(abs(duty))



    def enable(self):
        #turns the enable pin on
        self.EN_pin.value(1)

    def disable(self):
        #turns the enable pin off
        self.PID_enable = False
        self.set_duty(0)
        self.target_speed = 0
        self.EN_pin.value(0)

    def soft_disable(self):
        self.PID_enable = False
        self.set_duty(0)
        self.target_speed = 0

    class Encoder:
        '''!@brief Interface with quadrature encoders
        @details Utilizes external timers to count encoder ticks
        update() must be called at least once per timer overflow for accurate readings
        '''
        AR = 65355
        PS = 0
        TICKS_PER_REV = 1440
        PI = 3.14159

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
            self.speed = 0

            #vars for calculating speed
            self.time_was = utime.ticks_ms()
            self.time_is = None
            self.delta_T = None

        def update(self):
            '''!@brief Updates encoder position and delta, should be called often
            @details
            '''
            self.prev_position = self.cur_position   #moves last reading into previous position variable 
            self.cur_position = self.timer.counter()   #takes new position reading
            self.delta = self.cur_position - self.prev_position   #calculates delta between current and previous reading

            if self.delta > (L6206.Encoder.AR+1)//2:
                self.delta -= (L6206.Encoder.AR+1)

            elif self.delta < -1 * (L6206.Encoder.AR+1)//2:
                self.delta += (L6206.Encoder.AR+1)

            self.abs_position += self.delta            #

            self.time_is = utime.ticks_ms()
            self.delta_T = utime.ticks_diff(self.time_is, self.time_was)/1000 #sec
            #print(f"delta T: {self.delta_T}")
            self.time_was = utime.ticks_ms() #cant assign to time_is because object references!!

            self.speed = self.delta/self.TICKS_PER_REV*2*self.PI/self.delta_T


        def get_position(self):
            '''!@brief Gets the most recent encoder position
            '''
            return self.abs_position

        def get_delta(self):
            '''!@brief Gets the most recent encoder delta           
            '''
            return self.delta

        def get_speed(self):
            '''!@brief Gets the most recent speed data in Rad/sec'''
            return self.speed

        def get_delta_T(self):
            '''!@brief Gets the most recent delta T between readings'''
            return self.delta_T

        def zero(self):
            '''!@brief Resets the encoder position to zero
            '''
            self.abs_position = 0

    def set_encoder(self, timernumber, pin1, pin2):
        '''!@brief Creates encoder object for motor given timer and pins'''
        self.encoder = L6206.Encoder(timernumber, pin1, pin2)

    def get_encoder(self):
        '''!@brief Gets encoder object attached to this motor'''
        return self.encoder
    
    def update(self):
        '''!@brief polls encoder and updates PID effort'''
        self.encoder.update()
        #print("Checkpoint B")
        if self.PID_enable is True:
            #use utime.ticks_ms and ticks_diff to find deltaT
            current_speed = self.encoder.get_speed()
            error = self.target_speed - current_speed #rad/s
            delta_T = self.encoder.get_delta_T()

            #calc FF effort
            FF_effort = self.K_ff*self.target_speed

            #calc P effort
            P_effort = self.K_p*error

            #calc I effort
            self.error_sum += error*delta_T
            I_effort = self.error_sum*self.K_i
            
            if delta_T == 0:
                raise ValueError("delta zero boooo")

            if delta_T < 0.0001:
                raise ValueError("delta sucks")
            #calc D effort
            D_effort = self.K_d*(error - self.error_was)/delta_T
            self.error_was = error

            #print("Checkpoint E")
           
            control_effort = FF_effort + P_effort + I_effort + D_effort

            if control_effort > self.MAX_CONTRL_EFFORT:
                control_effort = self.MAX_CONTRL_EFFORT
            
            elif control_effort < -self.MAX_CONTRL_EFFORT:
                control_effort = -self.MAX_CONTRL_EFFORT


            #print("Checkpoint F")

            #print(f"target: {self.target_speed}, error: {error}, P: {P_effort}, I: {I_effort}, D:{D_effort}, effort:{control_effort}/{self.MAX_CONTRL_EFFORT}, enc speed: {current_speed}")

            #convert control effort to duty cycle
            duty = int(control_effort/self.MAX_CONTRL_EFFORT*100)
            self.set_duty(duty)

    def set_speed(self, speed):
        '''@brief Sets PWM according to target speed (in rad/s), begins PID control'''
        if self.encoder is None:
            return ValueError("No encoder attached")
        self.PID_enable = True
        self.target_speed = speed #rad/s
      
        #print(f"approximate duty: {int(speed/self.K_V/self.SUPPLY_VOLTAGE*100)}")

        #set duty to approximate value
        #self.set_duty(int(speed/self.K_V/self.SUPPLY_VOLTAGE*100))   #SEE EQ 2 in report

    def reset_PID_error(self):
          #print("reset errors")
          self.error_sum = 0
          self.error_was = 0

if __name__ == '__main__':

    tim_A = Timer(3, freq = 20_000)
    mot_A = L6206(tim_A, Pin.cpu.C6, Pin.cpu.B13, Pin.cpu.B14)

    tim_B = Timer(4, freq = 20_000)
    mot_B = L6206(tim_B, Pin.cpu.B6, Pin.cpu.B11, Pin.cpu.B12)

    mot_A.set_encoder(5, Pin.cpu.A0, Pin.cpu.A1)
    mot_B.set_encoder(3, Pin.cpu.B4, Pin.cpu.B5)

    mot_A.enable()
    mot_B.enable()

    mot_A.set_duty(100)
    mot_B.set_duty(100)

    interval = 1_000_000
    poll_delay_ms = 1

    Button = Pin(Pin.cpu.C13, mode = Pin.IN)  #This pin is pulled high when the button isn't pressed
    button_toggle = 0

    while True:
        utime.sleep_ms(200)
        mot_A.update()
        mot_B.update()