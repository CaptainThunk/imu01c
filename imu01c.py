#!/usr/bin.env python

### For pololu imu01c

import smbus
import time
import math
import RPi.GPIO as GPIO
import struct

rev = GPIO.RPI_REVISION
if rev == 2 or rev == 3:
    bus = smbus.SMBus(1)
else:
    bus = smbus.SMBus(0)

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

X = 0
Y = 1
Z = 2

def twos_comp(val, bits):
    # Calculate the 2s complement of int:val #
    if(val&(1<<(bits-1)) != 0):
        val = val - (1<<bits)
    return val
#   return val if val < 32768 else val - 65536

class accelcomp:
    mag = [0,0,0]
    accel = [0,0,0]
    gyro = [0,0,0]
    tiltcomp = [0,0,0]
    heading=0
    headingDegrees=0
    tiltHeading=0
    tiltHeadingDegrees=0

    def __init__(self):
        whoami = bus.read_byte_data(LSM303D_ADDRESS, LSM303D_REGISTER_WHO_AM_I)
        print("LSM303D WhoamI: "+str(whoami))
        if(whoami == 0x49):
            
            bus.write_byte_data(LSM303D_ADDRESS, LSM303D_REGISTER_CTRL1, 0x57) # 0x57 = ODR=50hz, all accel axes on ## maybe 0x27 is Low Res?
                    # bus.write_byte_data(LSM303D_ADDR, CTRL_REG1, 0x47) # 0x57 = ODR=50hz, all accel axes on ## maybe 0x27 is Low Res?
            bus.write_byte_data(LSM303D_ADDRESS, LSM303D_REGISTER_CTRL2, 0x00) # set full scale +/- 2g
            bus.write_byte_data(LSM303D_ADDRESS, LSM303D_REGISTER_CTRL3, 0x00) # no interrupt
            bus.write_byte_data(LSM303D_ADDRESS, LSM303D_REGISTER_CTRL4, 0x00) # no interrupt
            bus.write_byte_data(LSM303D_ADDRESS, LSM303D_REGISTER_CTRL5, 0xC0) # 0x10 = mag 50Hz output rate and enabled temperature
            bus.write_byte_data(LSM303D_ADDRESS, LSM303D_REGISTER_CTRL6, LSM303D_MAG_SCALE_4) # Magnetic Scale +/1 1.3 Guass
            bus.write_byte_data(LSM303D_ADDRESS, LSM303D_REGISTER_CTRL7, 0x00) # 0x00 continuous conversion mode
            #bus.write_byte_data(LSM303D_ADDR, CRA_REG_M, 0b10010000) # sets temp enable and output rate 75hz
            #            bus.write_byte_data(LSM303D_ADDR, CRA_REG_M, 0x98) # sets temp enable and output rate 75hz
             #           bus.write_byte_data(LSM303D_ADDR, CRB_REG_M, MAG_GAIN_4_0) # sets gain to 2.5
            
            # bus.write_byte_data(LSM303D_ADDRESS_ACCEL, LSM303D_REGISTER_ACCEL_CTRL_REG1_A, 0x47)  #initialise the Accelerometer - all axis enabled 50hz
            # bus.write_byte_data(LSM303D_ADDRESS_ACCEL, LSM303D_REGISTER_ACCEL_CTRL_REG4_A, 0x00)  #set continuous update
            
            # bus.write_byte_data(LSM303D_ADDRESS_MAG, LSM303D_REGISTER_MAG_CRA_REG_M, 0x98)        #enable temp sensor at 75hz
            # bus.write_byte_data(LSM303D_ADDRESS_MAG, LSM303D_REGISTER_MAG_MR_REG_M, 0x00)         #initialise the Magnetometer
            
        whoami2 = bus.read_byte_data(L3GD20H_ADDRESS, L3GD20H_REGISTER_WHO_AM_I)
        print("L3GD20H WhoamI: "+str(whoami2))
        if(whoami2 == 0xD7):
            bus.write_byte_data(L3GD20H_ADDRESS, L3GD20H_REGISTER_CTRL1, 0x0F) # 0xF = enable all axis in normal mode
            
            
            
    def getMag(self):
        self.mag[X] = twos_comp(bus.read_byte_data(LSM303D_ADDRESS, LSM303D_REGISTER_OUT_X_H_M) << 8 | 
                          bus.read_byte_data(LSM303D_ADDRESS, LSM303D_REGISTER_OUT_X_L_M), 16)
        self.mag[Y] = twos_comp(bus.read_byte_data(LSM303D_ADDRESS, LSM303D_REGISTER_OUT_Y_H_M) << 8 | 
                          bus.read_byte_data(LSM303D_ADDRESS, LSM303D_REGISTER_OUT_Y_L_M), 16)
        self.mag[Z] = twos_comp(bus.read_byte_data(LSM303D_ADDRESS, LSM303D_REGISTER_OUT_Z_H_M) << 8 | 
                          bus.read_byte_data(LSM303D_ADDRESS, LSM303D_REGISTER_OUT_Z_L_M), 16)

    def getAccel(self):
        accel = [0,0,0]
        accel[X] = twos_comp(bus.read_byte_data(LSM303D_ADDRESS, LSM303D_REGISTER_OUT_X_H_A) << 8 | 
                           bus.read_byte_data(LSM303D_ADDRESS, LSM303D_REGISTER_OUT_X_L_A), 16)
        accel[Y] = twos_comp(bus.read_byte_data(LSM303D_ADDRESS, LSM303D_REGISTER_OUT_Y_H_A) << 8 | 
                           bus.read_byte_data(LSM303D_ADDRESS, LSM303D_REGISTER_OUT_Y_L_A), 16)
        accel[Z] = twos_comp(bus.read_byte_data(LSM303D_ADDRESS, LSM303D_REGISTER_OUT_Z_H_A) << 8 | 
                           bus.read_byte_data(LSM303D_ADDRESS, LSM303D_REGISTER_OUT_Z_L_A), 16)

        for i in range(X, Z+1):
            self.accel[i] = accel[i] / math.pow(2, 15) * LSM303D_ACCEL_SCALE

    def getGyro(self):
        self.gyro[X] = twos_comp(bus.read_byte_data(L3GD20H_ADDRESS, L3GD20H_REGISTER_OUT_X_H) << 8 | 
                          bus.read_byte_data(L3GD20H_ADDRESS, L3GD20H_REGISTER_OUT_X_L), 16)
        self.gyro[Y] = twos_comp(bus.read_byte_data(L3GD20H_ADDRESS, L3GD20H_REGISTER_OUT_Y_H) << 8 | 
                          bus.read_byte_data(L3GD20H_ADDRESS, L3GD20H_REGISTER_OUT_Y_L), 16)
        self.gyro[Z] = twos_comp(bus.read_byte_data(L3GD20H_ADDRESS, L3GD20H_REGISTER_OUT_Z_H) << 8 | 
                          bus.read_byte_data(L3GD20H_ADDRESS, L3GD20H_REGISTER_OUT_Z_L), 16)
                          
    def getL3GD20HTemp(self):
        temp = twos_comp(bus.read_byte_data(L3GD20H_ADDRESS, L3GD20H_REGISTER_OUT_TEMP), 8)
       
        return (temp / 8.0) + TEMPERATURE_CELSIUS_OFFSET
        
    def getLSM303DTemp(self):
        # temp = twos_comp(bus.read_byte_data(LSM303D_ADDRESS_MAG, LSM303D_REGISTER_MAG_TEMP_OUT_H_M) << 8 | 
                # bus.read_byte_data(LSM303D_ADDRESS_MAG, LSM303D_REGISTER_MAG_TEMP_OUT_L_M), 12)
        #temp = (bus.read_byte_data(LSM303D_ADDRESS_MAG, LSM303D_REGISTER_MAG_TEMP_OUT_H_M) >> 8 | bus.read_byte_data(LSM303D_ADDRESS_MAG, LSM303D_REGISTER_MAG_TEMP_OUT_L_M) << 8)    
        # temp = twos_comp(bus.read_byte_data(LSM303D_ADDRESS_MAG, LSM303D_REGISTER_MAG_TEMP_OUT_H_M) << 8 | 
                 # bus.read_byte_data(LSM303D_ADDRESS_MAG, LSM303D_REGISTER_MAG_TEMP_OUT_L_M) >> 8, 12)
        #if temp & (1 << 11):
        #    temp = (temp & ((1 << 12) -1)) - (1 << 12)
            
        
        # print(bus.read_byte_data(LSM303D_ADDRESS_MAG, LSM303D_REGISTER_MAG_TEMP_OUT_H_M))
        # print(bus.read_byte_data(LSM303D_ADDRESS_MAG, LSM303D_REGISTER_MAG_TEMP_OUT_L_M))
        # print("Temp {}".format(temp))
        
        # thi = bus.read_byte_data(LSM303D_ADDRESS_MAG, LSM303D_REGISTER_MAG_TEMP_OUT_H_M)
        # tlo = bus.read_byte_data(LSM303D_ADDRESS_MAG, LSM303D_REGISTER_MAG_TEMP_OUT_L_M)
       # print(thi << 4, tlo << 8)
        temp = twos_comp(((bus.read_byte_data(LSM303D_ADDRESS, LSM303D_REGISTER_TEMP_OUT_H) << 8) | bus.read_byte_data(LSM303D_ADDRESS, LSM303D_REGISTER_TEMP_OUT_L)) >> 4, 12)
        #temp = (temp / 8.0) + TEMPERATURE_CELSIUS_OFFSET
        # temp = ((thi * 16) + (tlo / 16))
        #temp = (thi / 16) + (tlo / 256)
        return (temp / 8.0) + TEMPERATURE_CELSIUS_OFFSET

    def getHeading(self):

        heading = 180 * math.atan2(self.mag[Y], self.mag[X])/math.pi
        if(heading < 0):
            heading += 360

        self.headingDegrees = heading

        mtMagX = self.mag[X] / 450 * 100.0
        mtMagY = self.mag[Y] / 450 * 100.0
        mtMagY = self.mag[Z] / 400 * 100.0

        self.heading = math.atan2(mtMagX, mtMagY)



        # original, it's 90 deg out
#        self.heading = math.atan2(self.mag[X], self.mag[Y])

#        if self.heading < 0:
#            self.heading += 2*math.pi
#        if self.heading > 2*math.pi:
#            self.heading -= 2*math.pi

#        self.headingDegrees = round(math.degrees(self.heading),2)

    def getTiltHeading(self):
        truncate = [0,0,0]
        for i in range(X, Z+1):
            truncate[i] = math.copysign(min(math.fabs(self.accel[i]), 1.0), self.accel[i])
        try:
            pitch = math.asin(-1*truncate[X])
            roll = math.asin(truncate[Y]/math.cos(pitch)) if abs(math.cos(pitch)) >= abs(truncate[Y]) else 0
            # print("Roll {}".format(roll))
            # set roll to zero if pitch approaches -1 or 1

            self.tiltcomp[X] = self.mag[X] * math.cos(pitch) + self.mag[Z] * math.sin(pitch)
            self.tiltcomp[Y] = self.mag[X] * math.sin(roll) * math.sin(pitch) + \
                               self.mag[Y] * math.cos(roll) - self.mag[Z] * math.sin(roll) * math.cos(pitch)
            self.tiltcomp[Z] = self.mag[X] * math.cos(roll) * math.sin(pitch) + \
                               self.mag[Y] * math.sin(roll) + \
                               self.mag[Z] * math.cos(roll) * math.cos(pitch)
            self.tiltHeading = math.atan2(self.tiltcomp[Y], self.tiltcomp[X])

            if self.tiltHeading < 0:
                self.tiltHeading += 2*math.pi
            if self.tiltHeading > 2*math.pi:
                self.heading -= 2*math.pi

            self.tiltHeadingDegrees = round(math.degrees(self.tiltHeading),2)
            # print("AccelX {}, AccelY {}".format(self.accel[X], self.accel[Y]))
            # print("TruncX {}, TruncY {}".format(truncate[X], truncate[Y]))
            # print("Pitch {}, cos(pitch) {}, Bool(cos(pitch)) {}".format(pitch, math.cos(pitch), bool(math.cos(pitch))))
        except Exception:
            print("AccelX {}, AccelY {}".format(self.accel[X], self.accel[Y]))
            print("TruncX {}, TruncY {}".format(truncate[X], truncate[Y]))
            print("Pitch {}, cos(pitch) {}, Bool(cos(pitch)) {}".format(pitch, math.cos(pitch), bool(math.cos(pitch))))


    def isMagReady(self):
        temp =  bus.read_byte_data(LSM303D_ADDRESS, LSM303D_REGISTER_STATUS_M) & 0x08
        #print("Status: ", temp)
        return temp
        # return True

    def update(self):

        self.getAccel()
        self.getMag()
        self.getHeading()
        self.getTiltHeading()

if __name__ == '__main__':

    from time import sleep

    lsm = accelcomp()

    print ('         Accel.                                ||             Compass             ||             Gyro             ||                Heading	     ||      Temp')
    print ('       x       |       y       |       z       ||     x     |     y     |     z     ||     x     |     y     |     z     ||        calc      | tilt corrected  ||')
    while True:

        temp = lsm.getLSM303DTemp()
        temp2 = lsm.getL3GD20HTemp()
        
        while not bool(lsm.isMagReady()):
            print("Not Ready!")
            sleep(.01)
        lsm.getMag()
        lsm.getHeading()
        lsm.getTiltHeading()
        lsm.getAccel()
        lsm.getGyro()
        print('{0:12} |{1:12}|{2:12} || {3:12} |{4:12} |{5:12} || {6:12} |{7:12} |{8:12} || {9:12} deg |  {10:12} deg | {11:12} degrees'.format(
                lsm.accel[X], lsm.accel[Y], lsm.accel[Z], lsm.mag[X], lsm.mag[Y], lsm.mag[Z], lsm.gyro[X], lsm.gyro[Y], lsm.gyro[Z], lsm.headingDegrees, lsm.tiltHeadingDegrees, temp2))
        # print('{0:3} |{1:3} |{2:3}'.format(lsm.accel[X], lsm.accel[Y], lsm.accel[Z]))
        sleep(.5)

