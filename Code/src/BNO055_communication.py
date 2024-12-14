from pyb import I2C, Pin
import utime
import time
import struct

DEFAULT_ADDRESS = 0x28
BYTE_SIZE = 1
NUM_CAL_CONSTANTS = 11*2
NUM_BYTE_IN_DATA = 2

class BNO055:
    def __init__(self, controller):
        self.i2c = controller
        self.mode = "NDOF"
        self.mag_cal = False
        self.acc_cal = False
        self.gyr_cal = False
        self.sys_cal = False
        self.mag_radius = 0
        self.acc_radius = 0
        self.gyr_z_cal = 0
        self.gyr_y_cal = 0
        self.gyr_x_cal = 0
        self.mag_z_cal = 0
        self.mag_y_cal = 0
        self.mag_x_cal = 0
        self.acc_z_cal = 0
        self.acc_y_cal = 0
        self.acc_x_cal = 0

    def change_mode(self, new_mode):
        # Operating Modes (OPR_MODE) for various sensor configurations:
        # IMU             : xxxx1000b
        # COMPASS         : xxxx1001b
        # M4G             : xxxx1010b
        # NDOF_FMC_OFF    : xxxx1011b
        # NDOF            : xxxx1100b
        modes = {
            "CONFIGMODE": 0x00,
            "ACCONLY": 0x01,
            "MAGONLY": 0x02,
            "GYROONLY": 0x03,
            "ACCMAG": 0x04,
            "ACCGYRO": 0x05,
            "MAGGYRO": 0x06,
            "AMG": 0x07,
            "IMU": 0x08,
            "COMPASS": 0x09,
            "M4G": 0x0A,
            "NDOF_FMC_OFF": 0x0B,
            "NDOF": 0x0C
        }

        # Check if the mode is valid
        if new_mode in modes:
            # Write the mode value to the operation mode register (0x3D)
            self.i2c.mem_write(modes[new_mode], DEFAULT_ADDRESS, 0x3D, timeout=10)
            self.mode = new_mode
        else:
            print("Invalid mode selected.")

    # NOTE: might have to write blocking code to wait while cal status is set appropriately
    def get_cal_status(self):
        cal_status = self.i2c.mem_read(BYTE_SIZE, DEFAULT_ADDRESS, 0x35)[0]
        if (cal_status & 0x03) == 0:
            self.mag_cal = False
        else:
            self.mag_cal = True

        if (cal_status & 0x0C) == 0:
            self.acc_cal = False
        else:
            self.acc_cal = True

        if (cal_status & 0x30) == 0:
            self.gyr_cal = False
        else:
            self.gyr_cal = True
        
        if (cal_status & 0xC0) == 0:
            self.sys_cal = False
        else:
            self.sys_cal = True

        return (self.mag_cal, self.acc_cal, self.gyr_cal, self.sys_cal)

    def get_cal_coef(self):
        buf = bytearray([0 for _ in range(NUM_CAL_CONSTANTS)])
        self.i2c.mem_read(buf, DEFAULT_ADDRESS, 0x55)
        return buf

    def set_cal_coef(self, cal_buffer = None):
        oldMode = self.mode
        self.change_mode("CONFIGMODE")
        if cal_buffer is None:
            cal_buffer = bytearray(
                self.acc_x_cal.to_bytes(2, 'little') +
                self.acc_y_cal.to_bytes(2, 'little') +
                self.acc_z_cal.to_bytes(2, 'little') +
                self.mag_x_cal.to_bytes(2, 'little') +
                self.mag_y_cal.to_bytes(2, 'little') +
                self.mag_z_cal.to_bytes(2, 'little') +
                self.gyr_x_cal.to_bytes(2, 'little') +
                self.gyr_y_cal.to_bytes(2, 'little') +
                self.gyr_z_cal.to_bytes(2, 'little') +
                self.acc_radius.to_bytes(2, 'little') +
                self.mag_radius.to_bytes(2, 'little')
            )
        else:
            (
                self.acc_x_cal, self.acc_y_cal, self.acc_z_cal, 
                self.mag_x_cal, self.mag_y_cal, self.mag_z_cal, 
                self.gyr_x_cal, self.gyr_y_cal, self.gyr_z_cal, 
                self.acc_radius, self.mag_radius
            ) = struct.unpack('<hhhhhhhhhhh', cal_buffer)
        # Write the entire buffer to the calibration registers, starting at address 0x55
        for i in range(NUM_CAL_CONSTANTS):
            value = struct.pack('<h', (cal_buffer[i+1] << 8 | cal_buffer[i]))
            print(value)
            self.i2c.mem_write(value, DEFAULT_ADDRESS, 0x55+i, timeout=1000)
            i = i + 2
            utime.sleep_ms(100)
            print(f"At address {0x55+i} the value should be {cal_buffer[i]}")
            print(f"At address {0x55+i} the value is {struct.unpack('<h', i2c.mem_read(2, DEFAULT_ADDRESS, 0x55+i))}")

        self.change_mode(oldMode)

    def get_EUL_heading(self):
        buf = bytearray([0 for _ in range(NUM_BYTE_IN_DATA)])
        self.i2c.mem_read(buf, DEFAULT_ADDRESS, 0x1A)
        eul_heading = struct.unpack('<h', buf)
        return eul_heading[0]

    def get_EUL_roll(self):
        buf = bytearray([0 for _ in range(NUM_BYTE_IN_DATA)])
        self.i2c.mem_read(buf, DEFAULT_ADDRESS, 0x1C)
        eul_roll = struct.unpack('<h', buf)
        return eul_roll[0]

    def get_EUL_pitch(self):
        buf = bytearray([0 for _ in range(NUM_BYTE_IN_DATA)])
        self.i2c.mem_read(buf, DEFAULT_ADDRESS, 0x1E)
        eul_pitch = struct.unpack('<h', buf)
        return eul_pitch[0]

    def get_GYR_Z(self):
        buf = bytearray([0 for _ in range(NUM_BYTE_IN_DATA)])
        self.i2c.mem_read(buf, DEFAULT_ADDRESS, 0x18)
        gyr_z = struct.unpack('<h', buf)
        return gyr_z[0]/5760*3.1415*2

    def get_GYR_Y(self):
        buf = bytearray([0 for _ in range(NUM_BYTE_IN_DATA)])
        self.i2c.mem_read(buf, DEFAULT_ADDRESS, 0x16)
        gyr_y = struct.unpack('<h', buf)
        return gyr_y[0]/5760*3.1415*2

    def get_GYR_X(self):
        buf = bytearray([0 for _ in range(NUM_BYTE_IN_DATA)])
        self.i2c.mem_read(buf, DEFAULT_ADDRESS, 0x14)
        gyr_x = struct.unpack('<h', buf)
        return gyr_x[0]/5760*3.1415*2
    
    def get_ACC_X(self):
        buf = bytearray([0 for _ in range(NUM_BYTE_IN_DATA)])
        self.i2c.mem_read(buf, DEFAULT_ADDRESS, 0x08)
        acc_x = struct.unpack('<h', buf)
        return acc_x[0]

    def get_ACC_Y(self):
        buf = bytearray([0 for _ in range(NUM_BYTE_IN_DATA)])
        self.i2c.mem_read(buf, DEFAULT_ADDRESS, 0x0A)
        acc_y = struct.unpack('<h', buf)
        return acc_y[0]

    def get_ACC_Z(self):
        buf = bytearray([0 for _ in range(NUM_BYTE_IN_DATA)])
        self.i2c.mem_read(buf, DEFAULT_ADDRESS, 0x0C)
        acc_z = struct.unpack('<h', buf)
        return acc_z[0]
    
    def get_all_data(self):
        data = bytearray()

        data.extend(struct.pack('<h', self.get_EUL_heading()))
        data.extend(struct.pack('<h', self.get_EUL_roll()))
        data.extend(struct.pack('<h', self.get_EUL_pitch()))

        data.extend(struct.pack('<h', self.get_GYR_X()))
        data.extend(struct.pack('<h', self.get_GYR_Y()))
        data.extend(struct.pack('<h', self.get_GYR_Z()))
    
        data.extend(struct.pack('<h', self.get_ACC_X()))
        data.extend(struct.pack('<h', self.get_ACC_Y()))
        data.extend(struct.pack('<h', self.get_ACC_Z()))

        return data

def test02(imu):
    running = True
    while running:
        try:
            data = imu.get_all_data()
            print(struct.unpack('<hhhhhhhhhhh', data))
            utime.sleep(1)
            
        except(KeyboardInterrupt):
            print("Detected a keyboard break. Terminating program. \n")
            running = False

if __name__ == "__main__":
    i2c = I2C(1, I2C.CONTROLLER, baudrate=100000)
    utime.sleep_ms(100)                   
    imu = BNO055(i2c)
    imu.change_mode("IMU")
    utime.sleep(6)
    print("Done Sleeping")

    #while(not (imu.get_cal_status()[1] and imu.get_cal_status()[2] == True)):
    #    utime.sleep_ms(100)

    print("Done with Initial Calibration")

    utime.sleep_ms(100)
    
    #test02(imu)

    


    