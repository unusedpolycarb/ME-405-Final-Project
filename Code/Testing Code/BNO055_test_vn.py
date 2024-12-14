from pyb import I2C, Pin
import utime
import time
import struct

DEFAULT_ADDRESS = 0x28 #40
BYTE_SIZE = 1
NUM_CAL_CONSTANTS = 11*2
NUM_BYTE_IN_DATA = 2

class BNO055:
    def __init__(self, controller):
        self.i2c = controller
        self.mode = "INIT"
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
            i2c.mem_write(modes[new_mode], DEFAULT_ADDRESS, 0x3D, timeout=1000)
            self.mode = new_mode
        else:
            print("Invalid mode selected.")
        utime.sleep_ms(20)

    # NOTE: might have to write blocking code to wait while cal status is set appropriately
    def get_cal_status(self):
        cal_status = i2c.mem_read(BYTE_SIZE, DEFAULT_ADDRESS, 0x35)[0]
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

    def get_cal_coef(self): #works
        buf = bytearray([0 for _ in range(NUM_CAL_CONSTANTS)])
        i2c.mem_read(buf, DEFAULT_ADDRESS, 0x55)
        (
            self.acc_x_cal, self.acc_y_cal, self.acc_z_cal, 
            self.mag_x_cal, self.mag_y_cal, self.mag_z_cal, 
            self.gyr_x_cal, self.gyr_y_cal, self.gyr_z_cal, 
            self.acc_radius, self.mag_radius
        ) = struct.unpack('>hhhhhhhhhhh', buf)

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

        # Write the entire buffer to the calibration registers, starting at address 0x55
        i2c.mem_write(cal_buffer, DEFAULT_ADDRESS, 0x55, timeout=1000)
        self.change_mode(oldMode)

        print(self.get_cal_status())

    def get_EUL_heading(self):
        buf = bytearray([0 for _ in range(NUM_BYTE_IN_DATA)])
        i2c.mem_read(buf, DEFAULT_ADDRESS, 0x1A)
        eul_heading = struct.unpack('>h', buf)
        return eul_heading

    def get_EUL_roll(self):
        buf = bytearray([0 for _ in range(NUM_BYTE_IN_DATA)])
        i2c.mem_read(buf, DEFAULT_ADDRESS, 0x1C)
        eul_roll = struct.unpack('>h', buf)
        return eul_roll

    def get_EUL_pitch(self):
        buf = bytearray([0 for _ in range(NUM_BYTE_IN_DATA)])
        i2c.mem_read(buf, DEFAULT_ADDRESS, 0x1E)
        eul_pitch = struct.unpack('>h', buf)
        return eul_pitch

    def get_GYR_Z(self):
        buf = bytearray([0 for _ in range(NUM_BYTE_IN_DATA)])
        i2c.mem_read(buf, DEFAULT_ADDRESS, 0x18)
        gyr_z = struct.unpack('>h', buf)
        return gyr_z

    def get_GYR_Y(self):
        buf = bytearray([0 for _ in range(NUM_BYTE_IN_DATA)])
        i2c.mem_read(buf, DEFAULT_ADDRESS, 0x16)
        gyr_y = struct.unpack('>h', buf)
        return gyr_y

    def get_GYR_X(self):
        buf = bytearray([0 for _ in range(NUM_BYTE_IN_DATA)])
        i2c.mem_read(buf, DEFAULT_ADDRESS, 0x14)
        gyr_x = struct.unpack('>h', buf)
        return gyr_x
    
    def get_ACC_X(self):
        buf = bytearray([0 for _ in range(NUM_BYTE_IN_DATA)])
        i2c.mem_read(buf, DEFAULT_ADDRESS, 0x08)
        acc_x = struct.unpack('>h', buf)
        return acc_x

    def get_ACC_Y(self):
        buf = bytearray([0 for _ in range(NUM_BYTE_IN_DATA)])
        i2c.mem_read(buf, DEFAULT_ADDRESS, 0x0A)
        acc_y = struct.unpack('>h', buf)
        return acc_y

    def get_ACC_Z(self):
        buf = bytearray([0 for _ in range(NUM_BYTE_IN_DATA)])
        i2c.mem_read(buf, DEFAULT_ADDRESS, 0x0C)
        acc_z = struct.unpack('>h', buf)
        return acc_z
    
def test02():
    running = True
    while running:
        try:
            print(f'EULER Heading: {imu.get_EUL_heading()}')
            print(f'EULER Roll: {imu.get_EUL_roll()}')
            print(f'EULER Pitch: {imu.get_EUL_pitch()}')
            print(f'GYRO X: {imu.get_GYR_X()}')
            print(f'GYRO Y: {imu.get_GYR_Y()}')
            print(f'GYRO Z: {imu.get_GYR_Z()}')
            print(f'ACC X: {imu.get_ACC_X()}')
            print(f'ACC Y: {imu.get_ACC_Y()}')
            print(f'ACC Z: {imu.get_ACC_Z()}')
            time.sleep(1)
            
        except(KeyboardInterrupt):
            print("Detected a keyboard break. Terminating program. \n")
            running = False

if __name__ == "__main__":
    i2c = I2C(1, I2C.CONTROLLER, baudrate=100000)
    utime.sleep_ms(100)                             
    imu = BNO055(i2c)
    #while(not (imu.get_cal_status()[1] and imu.get_cal_status()[2] == True)):
    #    utime.sleep(100)
    #print(imu.get_cal_coef())


    