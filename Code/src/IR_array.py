from pyb import Pin, ADC

class IR_sensor:

    def __init__ (self, AN_pin, threshold):
        self.AN_pin = AN_pin
        self.ADC = ADC(AN_pin)
        self.threshold = threshold
    
    def get_line(self):
        value = self.ADC.read()
        if value <= self.threshold:
            return True
        else:
            return False
    
    def get_val(self):
        return self.ADC.read()
            
class IR_array:

    def __init__(self, array):
        self.array = []
        for sensor in array:
            if type(sensor) is IR_sensor:
                self.array.append(sensor)
            else:
                raise TypeError("attempted to add non IR_sensor to IR_array")
        pass

    def get_line(self):
        self.temp = []
        for sensor in self.array:
            self.temp.append(sensor.get_line())
        return self.temp

    
    def get_vals(self):
        self.temp = []
        for sensor in self.array:
            self.temp.append(sensor.get_val())
        return self.temp

    
    

