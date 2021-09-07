#!/usr/bin/env python
import imu01c
import time

# Simple example of outputs

print ('        Accel.           ||          Compass         ||           Gyro           ||         Heading         ||      Temperature in Celsius     ')
print ('   x   |    y   |    z   ||    x   |    y   |    z   ||    x   |   y    |    z   ||    calc    |    tilt    ||     LSM303D    |     L3GD20H    ')
while True:

        temp = imu01c.getLSM303DTemp()
        # temp2 = imu01c.getL3GD20HTemp()
        
        while not bool(imu01c.isMagReady()):
            print("Not Ready!")
            time.sleep(.01)
            
        # functions return the values or you can fetch them from the instance
        xyzM = imu01c.getMag()
        xyzA = imu01c.getAccel()
        xyzG = imu01c.getGyro()
        heading = imu01c.getHeading()
        tHeadhing = imu01c.getTiltHeading()
        print('{0:2.4f} | {1:2.4f} | {2:2.4f} || {3:6d} | {4:6d} | {5:6d} || {6:6d} | {7:6d} | {8:6d} || {9:6.2f} deg | {10:6.2f} deg || {11:6.2f} degrees | {12:6.2f} degrees'.format(
                imu01c.accel[imu01c.IMU01C_X], imu01c.accel[imu01c.IMU01C_Y], imu01c.accel[imu01c.IMU01C_Z], 
                xyzM[imu01c.IMU01C_X], xyzM[imu01c.IMU01C_Y], xyzM[imu01c.IMU01C_Z], 
                xyzG[0], xyzG[1], xyzG[2],
                heading, tHeadhing, 
                temp, imu01c.getL3GD20HTemp())
        )
    
        time.sleep(.5)