from pyb import Pin, ADC
from src.IR_array import IR_array,IR_sensor
import time

if __name__ == '__main__':
    thresholds = [3840, 3830, 3850, 3840, 3830]
    weights = [-22, -11, 0, 11, 22]

    IR_1 = IR_sensor(Pin.cpu.C2,thresholds[1])
    IR_2 = IR_sensor(Pin.cpu.C3,thresholds[2])
    IR_3 = IR_sensor(Pin.cpu.A4,thresholds[3])
    IR_4 = IR_sensor(Pin.cpu.B0,thresholds[4])
    IR_5 = IR_sensor(Pin.cpu.C1,thresholds[5])

    my_array = IR_array([IR_1, IR_2, IR_3, IR_4, IR_5])

    switch_A = Pin(Pin.cpu.H0, mode = Pin.IN, pull=Pin.PULL_DOWN)
    switch_B = Pin(Pin.cpu.H1, mode = Pin.IN, pull=Pin.PULL_DOWN)

    while True:
        time.sleep_ms(250)
        line_results = my_array.get_vals()
        position = 0

        for i in range(len(line_results)):
            my_array[i] = -(my_array[i] - thresholds[i])
            position += my_array[i] * weights[i]

        print(f"{my_array.get_line()}, vals: {my_array.get_vals()}, calculated position: {position}")
        
