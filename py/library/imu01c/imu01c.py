from threading import Timer
import time
import math
import atexit
from sys import version_info

IMU01C_X = 0
IMU01C_Y = 1
IMU01C_Z = 2

class imu01c:
    """imu01c Driver

    Communicates with imu01c over i2c

    """
    ### LSM303 Address ###
    LSM303D_ADDRESS = 0x1D # Default address Magnetometer / Accelerometer
    L3GD20H_ADDRESS	= 0x6B # Default address Gyroscope

    # registers

    # reserved 0x00-0x0E
    L3GD20H_REGISTER_WHO_AM_I       = 0x0F	# r	
    # reserved 0x10-0x1F
    L3GD20H_REGISTER_CTRL1          = 0x20	#/ rw
    L3GD20H_REGISTER_CTRL2          = 0x21	# rw
    L3GD20H_REGISTER_CTRL3          = 0x22	# rw
    L3GD20H_REGISTER_CTRL4          = 0x23	# rw
    L3GD20H_REGISTER_CTRL5          = 0x24	# rw
    L3GD20H_REGISTER_REFERENCE      = 0x25	# rw
    L3GD20H_REGISTER_OUT_TEMP       = 0x26	# r
    L3GD20H_REGISTER_STATUS         = 0x27	# r
    L3GD20H_REGISTER_OUT_X_L        = 0x28	# r
    L3GD20H_REGISTER_OUT_X_H        = 0x29	# r
    L3GD20H_REGISTER_OUT_Y_L        = 0x2A	# r
    L3GD20H_REGISTER_OUT_Y_H        = 0x2B	# r
    L3GD20H_REGISTER_OUT_Z_L        = 0x2C	# r
    L3GD20H_REGISTER_OUT_Z_H        = 0x2D	# r
    L3GD20H_REGISTER_FIFO_CTRL      = 0x2E	# rw
    L3GD20H_REGISTER_FIFO_SRC       = 0x2F	# r
    L3GD20H_REGISTER_IG_CFG         = 0x30	# rw
    L3GD20H_REGISTER_IG_SRC         = 0x31	# r
    L3GD20H_REGISTER_IG_THS_XH      = 0x32	# rw
    L3GD20H_REGISTER_IG_THS_XL      = 0x33	# rw
    L3GD20H_REGISTER_IG_THS_YH      = 0x34	# rw
    L3GD20H_REGISTER_IG_THS_YL      = 0x35	# rw
    L3GD20H_REGISTER_IG_THS_ZH      = 0x36	# rw
    L3GD20H_REGISTER_IG_THS_ZL      = 0x37	# rw
    L3GD20H_REGISTER_IG_DURATION    = 0x38	# rw
    L3GD20H_REGISTER_LOW_ODR        = 0x39	# rw

    LSM303D_REGISTER_TEMP_OUT_L     = 0x05
    LSM303D_REGISTER_TEMP_OUT_H     = 0x06
    LSM303D_REGISTER_STATUS_M       = 0x07
    LSM303D_REGISTER_OUT_X_L_M      = 0x08
    LSM303D_REGISTER_OUT_X_H_M      = 0x09
    LSM303D_REGISTER_OUT_Y_L_M      = 0x0A
    LSM303D_REGISTER_OUT_Y_H_M      = 0x0B
    LSM303D_REGISTER_OUT_Z_L_M      = 0x0C
    LSM303D_REGISTER_OUT_Z_H_M      = 0x0D
    # reserved 0x0E
    LSM303D_REGISTER_WHO_AM_I       = 0x0F
    # reserved 0x10-0x11
    LSM303D_REGISTER_INT_CTRL_M     = 0x12
    LSM303D_REGISTER_INT_SRC_M      = 0x13
    LSM303D_REGISTER_INT_THS_L_M    = 0x14
    LSM303D_REGISTER_INT_THS_H_M    = 0x15
    LSM303D_REGISTER_OFFET_X_L_M    = 0x16
    LSM303D_REGISTER_OFFET_X_H_M    = 0x17
    LSM303D_REGISTER_OFFET_Y_L_M    = 0x18
    LSM303D_REGISTER_OFFET_Y_H_M    = 0x19
    LSM303D_REGISTER_OFFET_Z_L_M    = 0x1A
    LSM303D_REGISTER_OFFET_Z_H_M    = 0x1B
    LSM303D_REGISTER_REFERENCE_X    = 0x1C
    LSM303D_REGISTER_REFERENCE_Y    = 0x1D
    LSM303D_REGISTER_REFERENCE_Z    = 0x1E
    LSM303D_REGISTER_CTRL0          = 0x1F 
    LSM303D_REGISTER_CTRL1          = 0x20        
    LSM303D_REGISTER_CTRL2          = 0x21        
    LSM303D_REGISTER_CTRL3          = 0x22        
    LSM303D_REGISTER_CTRL4          = 0x23       
    LSM303D_REGISTER_CTRL5          = 0x24        
    LSM303D_REGISTER_CTRL6          = 0x25        
    LSM303D_REGISTER_CTRL7          = 0x26        
    LSM303D_REGISTER_STATUS_A       = 0x27       
    LSM303D_REGISTER_OUT_X_L_A      = 0x28
    LSM303D_REGISTER_OUT_X_H_A      = 0x29
    LSM303D_REGISTER_OUT_Y_L_A      = 0x2A
    LSM303D_REGISTER_OUT_Y_H_A      = 0x2B
    LSM303D_REGISTER_OUT_Z_L_A      = 0x2C
    LSM303D_REGISTER_OUT_Z_H_A      = 0x2D
    LSM303D_REGISTER_FIFO_CTRL      = 0x2E
    LSM303D_REGISTER_FIFO_SRC       = 0x2F
    LSM303D_REGISTER_IG_CFG1        = 0x30
    LSM303D_REGISTER_IG_SRC1        = 0x31
    LSM303D_REGISTER_IG_THS1        = 0x32
    LSM303D_REGISTER_IG_DUR1        = 0x33
    LSM303D_REGISTER_IG_CFG2        = 0x34
    LSM303D_REGISTER_IG_SRC2        = 0x35
    LSM303D_REGISTER_IG_THS2        = 0x36
    LSM303D_REGISTER_IG_DUR2        = 0x37
    LSM303D_REGISTER_CLICK_CFG      = 0x38
    LSM303D_REGISTER_CLICK_SRC      = 0x39
    LSM303D_REGISTER_CLICK_THS      = 0x3A
    LSM303D_REGISTER_TIME_LIMIT     = 0x3B
    LSM303D_REGISTER_TIME_LATENCY   = 0x3C
    LSM303D_REGISTER_TIME_WINDOW    = 0x3D
    LSM303D_REGISTER_ACT_THS        = 0x3E
    LSM303D_REGISTER_ACT_DUR        = 0x3F

    LSM303D_MAG_SCALE_2             = 0x00
    LSM303D_MAG_SCALE_4             = 0x20
    LSM303D_MAG_SCALE_8             = 0x40
    LSM303D_MAG_SCALE_12            = 0x60

    LSM303D_ACCEL_SCALE             = 2     # +/- 2g
    TEMPERATURE_CELSIUS_OFFSET 		= 14    # offset for temperature calc (guesswork) originally 18
    TILT_HEADING_OFFSET             = 0     # 

    mag = [0,0,0]
    accel = [0,0,0]
    gyro = [0,0,0]
    tiltcomp = [0,0,0]
    heading=0
    headingDegrees=0
    tiltHeading=0
    tiltHeadingDegrees=0
        
    def __init__(self):
        self._is_setup = False
        
        self._i2c_retries = 10
        self._i2c_retry_time = 0.01

        self.bus = None
        
    def setup(self):
        if self._is_setup:
            return True

        if self.bus is None:
            try:
                import RPi.GPIO as GPIO
                from smbus import SMBus
                
                rev = GPIO.RPI_REVISION
                if rev == 2 or rev == 3:
                    self.bus = SMBus(1)
                else:
                    self.bus = SMBus(0)
                    
                whoami = self._i2c_read_byte(self.LSM303D_ADDRESS, self.LSM303D_REGISTER_WHO_AM_I)
                # print("LSM303D WhoamI: "+str(whoami))
                if(whoami == 0x49):
                    self._i2c_write_byte(self.LSM303D_ADDRESS, self.LSM303D_REGISTER_CTRL1, 0x57) # 0x57 = ODR=50hz, all accel axes on ## maybe 0x27 is Low Res?
                    self._i2c_write_byte(self.LSM303D_ADDRESS, self.LSM303D_REGISTER_CTRL2, 0x00) # set full scale +/- 2g
                    self._i2c_write_byte(self.LSM303D_ADDRESS, self.LSM303D_REGISTER_CTRL3, 0x00) # no interrupt
                    self._i2c_write_byte(self.LSM303D_ADDRESS, self.LSM303D_REGISTER_CTRL4, 0x00) # no interrupt
                    self._i2c_write_byte(self.LSM303D_ADDRESS, self.LSM303D_REGISTER_CTRL5, 0xC0) # 0x10 = mag 50Hz output rate and enabled temperature
                    self._i2c_write_byte(self.LSM303D_ADDRESS, self.LSM303D_REGISTER_CTRL6, self.LSM303D_MAG_SCALE_4) # Magnetic Scale +/1 1.3 Guass
                    self._i2c_write_byte(self.LSM303D_ADDRESS, self.LSM303D_REGISTER_CTRL7, 0x00) # 0x00 continuous conversion mode
                else:
                    raise IOError("Failed to find LSM303D device on "+str(self.LSM303D_ADDRESS))
                    
                whoami2 = self._i2c_read_byte(self.L3GD20H_ADDRESS, self.L3GD20H_REGISTER_WHO_AM_I)
                # print("L3GD20H WhoamI: "+str(whoami2))
                if(whoami2 == 0xD7):
                    self._i2c_write_byte(self.L3GD20H_ADDRESS, self.L3GD20H_REGISTER_CTRL1, 0x0F) # 0xF = enable all axis in normal mode
                else:
                    raise IOError("Failed to find L3GD20H device on "+str(self.L3GD20H_ADDRESS))
                    
            except ImportError:
                if version_info[0] < 3:
                    raise ImportError("This library requires python-smbus\nInstall with: sudo apt-get install python-smbus")
                elif version_info[0] == 3:
                    raise ImportError("This library requires python3-smbus\nInstall with: sudo apt-get install python3-smbus")

        atexit.register(self._atexit)

        self._is_setup = True

    def _atexit(self):
        # maybe turn off sensors here
        x = 1; # dummy line to maintain function


    def getMag(self):
        self.setup()
        
        self.mag[IMU01C_X] = self._twos_comp(self._i2c_read_byte(self.LSM303D_ADDRESS, self.LSM303D_REGISTER_OUT_X_H_M) << 8 | 
                          self._i2c_read_byte(self.LSM303D_ADDRESS, self.LSM303D_REGISTER_OUT_X_L_M), 16)
        self.mag[IMU01C_Y] = self._twos_comp(self._i2c_read_byte(self.LSM303D_ADDRESS, self.LSM303D_REGISTER_OUT_Y_H_M) << 8 | 
                          self._i2c_read_byte(self.LSM303D_ADDRESS, self.LSM303D_REGISTER_OUT_Y_L_M), 16)
        self.mag[IMU01C_Z] = self._twos_comp(self._i2c_read_byte(self.LSM303D_ADDRESS, self.LSM303D_REGISTER_OUT_Z_H_M) << 8 | 
                          self._i2c_read_byte(self.LSM303D_ADDRESS, self.LSM303D_REGISTER_OUT_Z_L_M), 16)

        return self.mag
        
    def getAccel(self):
        self.setup()
        
        accel = [0,0,0]
        accel[IMU01C_X] = self._twos_comp(self._i2c_read_byte(self.LSM303D_ADDRESS, self.LSM303D_REGISTER_OUT_X_H_A) << 8 | 
                           self._i2c_read_byte(self.LSM303D_ADDRESS, self.LSM303D_REGISTER_OUT_X_L_A), 16)
        accel[IMU01C_Y] = self._twos_comp(self._i2c_read_byte(self.LSM303D_ADDRESS, self.LSM303D_REGISTER_OUT_Y_H_A) << 8 | 
                           self._i2c_read_byte(self.LSM303D_ADDRESS, self.LSM303D_REGISTER_OUT_Y_L_A), 16)
        accel[IMU01C_Z] = self._twos_comp(self._i2c_read_byte(self.LSM303D_ADDRESS, self.LSM303D_REGISTER_OUT_Z_H_A) << 8 | 
                           self._i2c_read_byte(self.LSM303D_ADDRESS, self.LSM303D_REGISTER_OUT_Z_L_A), 16)

        for i in range(IMU01C_X, IMU01C_Z+1):
            self.accel[i] = accel[i] / math.pow(2, 15) * self.LSM303D_ACCEL_SCALE
            
        return self.accel

    def getGyro(self):
        self.setup()
        
        self.gyro[IMU01C_X] = self._twos_comp(self._i2c_read_byte(self.L3GD20H_ADDRESS, self.L3GD20H_REGISTER_OUT_X_H) << 8 | 
                          self._i2c_read_byte(self.L3GD20H_ADDRESS, self.L3GD20H_REGISTER_OUT_X_L), 16)
        self.gyro[IMU01C_Y] = self._twos_comp(self._i2c_read_byte(self.L3GD20H_ADDRESS, self.L3GD20H_REGISTER_OUT_Y_H) << 8 | 
                          self._i2c_read_byte(self.L3GD20H_ADDRESS, self.L3GD20H_REGISTER_OUT_Y_L), 16)
        self.gyro[IMU01C_Z] = self._twos_comp(self._i2c_read_byte(self.L3GD20H_ADDRESS, self.L3GD20H_REGISTER_OUT_Z_H) << 8 | 
                          self._i2c_read_byte(self.L3GD20H_ADDRESS, self.L3GD20H_REGISTER_OUT_Z_L), 16)
                          
        return self.gyro
                          
    def getL3GD20HTemp(self):
        self.setup()
        
        temp = self._twos_comp(self._i2c_read_byte(self.L3GD20H_ADDRESS, self.L3GD20H_REGISTER_OUT_TEMP), 8)
       
        return (temp / 8.0) + self.TEMPERATURE_CELSIUS_OFFSET
        
    def getLSM303DTemp(self):
        self.setup()
        
        temp = self._twos_comp(((self._i2c_read_byte(self.LSM303D_ADDRESS, self.LSM303D_REGISTER_TEMP_OUT_H) << 8) | self._i2c_read_byte(self.LSM303D_ADDRESS, self.LSM303D_REGISTER_TEMP_OUT_L)) >> 4, 12)
        
        return (temp / 8.0) + self.TEMPERATURE_CELSIUS_OFFSET

    def getHeading(self):
        # original
        self.getMag()
        
        self.heading = math.atan2(self.mag[IMU01C_X], self.mag[IMU01C_Y])

        if self.heading < 0:
            self.heading += 2*math.pi
        if self.heading > 2*math.pi:
            self.heading -= 2*math.pi

        self.headingDegrees = round(math.degrees(self.heading),2)
        
        return self.headingDegrees

    def getTiltHeading(self):
        # This is currently incorrect and needs fixing. Look at the mathematics
        self.getMag()
        
        truncate = [0,0,0]
        
        for i in range(IMU01C_X, IMU01C_Z+1):
            truncate[i] = math.copysign(min(math.fabs(self.accel[i]), 1.0), self.accel[i])
            # truncate[i] = math.copysign(min(math.fabs(self.gyro[i]), 1.0), self.gyro[i])

        try:
            pitch = math.asin(-1*truncate[IMU01C_X])
            roll = math.asin(truncate[IMU01C_Y]/math.cos(pitch)) if abs(math.cos(pitch)) >= abs(truncate[IMU01C_Y]) else 0
            # print("Roll {}".format(roll))
            # set roll to zero if pitch approaches -1 or 1

            self.tiltcomp[IMU01C_X] = self.mag[IMU01C_X] * math.cos(pitch) + self.mag[IMU01C_Z] * math.sin(pitch)
            self.tiltcomp[IMU01C_Y] = self.mag[IMU01C_X] * math.sin(roll) * math.sin(pitch) + \
                               self.mag[IMU01C_Y] * math.cos(roll) - self.mag[IMU01C_Z] * math.sin(roll) * math.cos(pitch)
            self.tiltcomp[IMU01C_Z] = self.mag[IMU01C_X] * math.cos(roll) * math.sin(pitch) + \
                               self.mag[IMU01C_Y] * math.sin(roll) + \
                               self.mag[IMU01C_Z] * math.cos(roll) * math.cos(pitch)
            self.tiltHeading = math.atan2(self.tiltcomp[IMU01C_Y], self.tiltcomp[IMU01C_X])

            # if self.tiltHeading < 0:
            #     self.tiltHeading += 2*math.pi
            # if self.tiltHeading > 2*math.pi:
            #     self.tiltHeading -= 2*math.pi

            self.tiltHeadingDegrees = round(math.degrees(self.tiltHeading),2)

            self.tiltHeadingDegrees += self.TILT_HEADING_OFFSET
            if(self.tiltHeadingDegrees < 0):
                self.tiltHeadingDegrees += 360
            
            if(self.tiltHeadingDegrees >= 360):
                self.tiltHeadingDegrees -= 360

        except Exception:
            print("AccelX {}, AccelY {}".format(self.accel[IMU01C_X], self.accel[IMU01C_Y]))
            print("TruncX {}, TruncY {}".format(truncate[IMU01C_X], truncate[IMU01C_Y]))
            print("Pitch {}, cos(pitch) {}, Bool(cos(pitch)) {}".format(pitch, math.cos(pitch), bool(math.cos(pitch))))

        return self.tiltHeadingDegrees

    def isMagReady(self):
        self.setup()
        
        temp =  self._i2c_read_byte(self.LSM303D_ADDRESS, self.LSM303D_REGISTER_STATUS_M) & 0x08
        
        return temp

    def update(self):
        self.getAccel()
        self.getMag()
        self.getGyro()
        self.getHeading()
        self.getTiltHeading()
        self.getLSM303DTemp()
        self.getL3GD20HTemp()

    def _twos_comp(self, val, bits):
        # Calculate the 2s complement of int:val #
        if(val&(1<<(bits-1)) != 0):
            val = val - (1<<bits)
        return val
        # return val if val < 32768 else val - 65536

    def _i2c_write_byte(self, i2c_address, reg, data):
        if type(data) is int:
            for x in range(self._i2c_retries):
                try:
                    self.bus.write_byte_data(i2c_address, reg, data)
                    return
                except IOError:
                    time.sleep(self._i2c_retry_time)
                    continue

            raise IOError("Failed to write byte")

    def _i2c_read_byte(self, i2c_address, reg):
        for x in range(self._i2c_retries):
            try:
                return self.bus.read_byte_data(i2c_address, reg)
            except IOError:
                time.sleep(self._i2c_retry_time)
                continue

        raise IOError("Failed to read byte")

    

    

    # pan = servo_one
    # tilt = servo_two
    # get_pan = get_servo_one
    # get_tilt = get_servo_two
