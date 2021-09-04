/*  
    C++ interface for controlling a Pololu imu01c sensor on a Raspberry Pi.
    Class built upon SMBus and i2C standard APIs.

    Works with sensor imu01c (dicontinued) (https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout/which-lsm303-do-i-have) It's the blue one.
    imu01c Specsheet Reference https://cdn-shop.adafruit.com/datasheets/LSM303DLHC.PDF

    Stand-alone compilation
    -----------------------
    libi2c.so.0.1.0/0.1.1 must exist on the system.
    See build_lsm.sh for building details.

    Copyright (C) 2021 Captain Thunk

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
    MA 02110-1301 USA.
*/

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <cmath>
#include <sys/wait.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include "smbus.h"
#include "imu01c.h"

using namespace std;

namespace imu01cns {

    // Constructor
    imu01c::imu01c()
    {
    }
    // Destructor
    imu01c::~imu01c()
    {
		if (i2c_lsm303d_addr != 0) {
			close(i2c_lsm303d_addr);
		}
		
		if (i2c_l3gd20h_addr != 0) {
			close(i2c_l3gd20h_addr);
		}
    }

    // public functions
    bool imu01c::setup()
    {
		if (_is_setup) return true;
		
		// setup communication with LSM303D
		if (i2c_lsm303d_addr == 0) {
			if ((i2c_lsm303d_addr = open("/dev/i2c-1", O_RDWR)) < 0) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": failed to open the i2c bus." << endl;
				return false;
			}
			if (ioctl(i2c_lsm303d_addr, I2C_SLAVE, LSM303D_ADDRESS) < 0) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": failed to access the i2c slave at 0x1D." << endl;
				return false;
			}
			// Check functionalities
			unsigned long mFuncs;
			if (ioctl(i2c_lsm303d_addr, I2C_FUNCS, &mFuncs) < 0) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ 
					<< ": could not get the adapter functionality matrix (errno " 
					<< strerror(errno) << ")."
					<< endl;
				return false;
			}
			if (!(mFuncs & I2C_FUNC_SMBUS_READ_BYTE)) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__
					<< MISSING_FUNC_1
					<< "SMBus receive byte" << MISSING_FUNC_2 << endl;
				return false;
			}
			if (!(mFuncs & I2C_FUNC_SMBUS_WRITE_BYTE)) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__
						<< MISSING_FUNC_1
						<< "SMBus send byte" << MISSING_FUNC_2 << endl;
				return false;
			}
			if (!(mFuncs & I2C_FUNC_SMBUS_READ_BYTE_DATA)) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__
						<< MISSING_FUNC_1
						<< "SMBus read byte" << MISSING_FUNC_2 << endl;
				return false;
			}
			if (!(mFuncs & I2C_FUNC_SMBUS_READ_WORD_DATA)) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__
						<< MISSING_FUNC_1
						<< "SMBus read word" << MISSING_FUNC_2 << endl;
				return false;
			}
		}
			
		// initialise the Accelerometer - all axis enabled 50hz
		if (!i2c_write_byte(i2c_lsm303d_addr, LSM303D_REGISTER_CTRL1, 0x57))	// 0x57 = ODR=50hz, all accel axes on ## maybe 0x27 is Low Res?
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to initialise accelerometer." << endl;
		
		// set Accelerometer to continuous updating
		if (!i2c_write_byte(i2c_lsm303d_addr, LSM303D_REGISTER_CTRL2, 0x00))	// set full scale +/- 2g
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to set full scale +/- 2g." << endl;
		
		if (!i2c_write_byte(i2c_lsm303d_addr, LSM303D_REGISTER_CTRL3, 0x00))	// no interrupt
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to set no interrupt." << endl;
		if (!i2c_write_byte(i2c_lsm303d_addr, LSM303D_REGISTER_CTRL4, 0x00))	// no interrupt
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to set no interrupt." << endl;
			
		if (!i2c_write_byte(i2c_lsm303d_addr, LSM303D_REGISTER_CTRL5, 0xC0))	// 0x10 = mag 50Hz output rate and enabled temperature
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to set mag 50Hz output rate and enabled temperature." << endl;
			
		// enable temperature sensor at 75hz
		if (!i2c_write_byte(i2c_lsm303d_addr, LSM303D_REGISTER_CTRL6, LSM303D_MAG_SCALE_4))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to set Magnetic Scale +/4 Guass." << endl;
		
		// initialise the Magnetometer
		if (!i2c_write_byte(i2c_lsm303d_addr, LSM303D_REGISTER_CTRL7, 0x00))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to initialise magnetometer." << endl;
		
		// setup communication with L3GD20H
		if (i2c_l3gd20h_addr == 0) {
			if ((i2c_l3gd20h_addr = open("/dev/i2c-1", O_RDWR)) < 0) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": failed to open the i2c bus." << endl;
				return false;
			}
			if (ioctl(i2c_l3gd20h_addr, I2C_SLAVE, L3GD20H_ADDRESS) < 0) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": failed to access the i2c slave at 0x6B." << endl;
				return false;
			}
			// Check functionalities
			unsigned long mFuncs;
			if (ioctl(i2c_l3gd20h_addr, I2C_FUNCS, &mFuncs) < 0) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ 
					<< ": could not get the adapter functionality matrix (errno " 
					<< strerror(errno) << ")."
					<< endl;
				return false;
			}
			if (!(mFuncs & I2C_FUNC_SMBUS_READ_BYTE)) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__
					<< MISSING_FUNC_1
					<< "SMBus receive byte" << MISSING_FUNC_2 << endl;
				return false;
			}
			if (!(mFuncs & I2C_FUNC_SMBUS_WRITE_BYTE)) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__
						<< MISSING_FUNC_1
						<< "SMBus send byte" << MISSING_FUNC_2 << endl;
				return false;
			}
			if (!(mFuncs & I2C_FUNC_SMBUS_READ_BYTE_DATA)) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__
						<< MISSING_FUNC_1
						<< "SMBus read byte" << MISSING_FUNC_2 << endl;
				return false;
			}
			if (!(mFuncs & I2C_FUNC_SMBUS_READ_WORD_DATA)) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__
						<< MISSING_FUNC_1
						<< "SMBus read word" << MISSING_FUNC_2 << endl;
				return false;
			}
		}	

		// initialise the Gyroscope
		if (!i2c_write_byte(i2c_l3gd20h_addr, L3GD20H_REGISTER_CTRL1, 0x0F))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to initialise gyroscope." << endl;
		
		_is_setup = true;
		return true;
    }

    void imu01c::getMag() {
		// self.mag[X] = twos_comp(bus.read_byte_data(LSM303D_ADDRESS_MAG, LSM303D_REGISTER_MAG_OUT_X_H_M) << 8 | 
                          // bus.read_byte_data(LSM303D_ADDRESS_MAG, LSM303D_REGISTER_MAG_OUT_X_L_M), 16)
		
		unsigned char mHi;
		unsigned char mLo;
		
		// x
		if (!i2c_read_byte(i2c_lsm303d_addr, LSM303D_REGISTER_OUT_X_H_M, &mHi))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get magnetometer x hi byte." << endl; 
		if (!i2c_read_byte(i2c_lsm303d_addr, LSM303D_REGISTER_OUT_X_L_M, &mLo))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get magnetometer x lo byte." << endl;
		mag_xyz[X] = twos_comp((mHi << 8 | mLo), 16);
		
		// y
		if (!i2c_read_byte(i2c_lsm303d_addr, LSM303D_REGISTER_OUT_Y_H_M, &mHi))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get magnetometer y hi byte." << endl; 
		if (!i2c_read_byte(i2c_lsm303d_addr, LSM303D_REGISTER_OUT_Y_L_M, &mLo))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get magnetometer y lo byte." << endl;
		mag_xyz[Y] = twos_comp((mHi << 8 | mLo), 16);
		
		// z
		if (!i2c_read_byte(i2c_lsm303d_addr, LSM303D_REGISTER_OUT_Z_H_M, &mHi))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get magnetometer z hi byte." << endl; 
		if (!i2c_read_byte(i2c_lsm303d_addr, LSM303D_REGISTER_OUT_Z_L_M, &mLo))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get magnetometer z lo byte." << endl;
		mag_xyz[Z] = twos_comp((mHi << 8 | mLo), 16);
		
		// printf("X: %d, Y: %d, Z: %d \r\n", mag_xyz[0], mag_xyz[1], mag_xyz[2]); 				  
	}
	
    void imu01c::getAccel() {
		unsigned char aHi;
		unsigned char aLo;
		
		// x
		if (!i2c_read_byte(i2c_lsm303d_addr, LSM303D_REGISTER_OUT_X_H_A, &aHi))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get accelerometer x hi byte." << endl; 
		if (!i2c_read_byte(i2c_lsm303d_addr, LSM303D_REGISTER_OUT_X_L_A, &aLo))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get accelerometer x lo byte." << endl;
		accel_xyz[X] = twos_comp((aHi << 8 | aLo), 16) / pow(2, 15) * LSM303D_ACCEL_SCALE;
		
		// y
		if (!i2c_read_byte(i2c_lsm303d_addr, LSM303D_REGISTER_OUT_Y_H_A, &aHi))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get accelerometer y hi byte." << endl; 
		if (!i2c_read_byte(i2c_lsm303d_addr, LSM303D_REGISTER_OUT_Y_L_A, &aLo))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get accelerometer y lo byte." << endl;
		accel_xyz[Y] = twos_comp((aHi << 8 | aLo), 16) / pow(2, 15) * LSM303D_ACCEL_SCALE;

		// z
		if (!i2c_read_byte(i2c_lsm303d_addr, LSM303D_REGISTER_OUT_Z_H_A, &aHi))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get accelerometer z hi byte." << endl; 
		if (!i2c_read_byte(i2c_lsm303d_addr, LSM303D_REGISTER_OUT_Z_L_A, &aLo))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get accelerometer z lo byte." << endl;
		accel_xyz[Z] = twos_comp((aHi << 8 | aLo), 16) / pow(2, 15) * LSM303D_ACCEL_SCALE;

		// printf("X: %d, Y: %d, Z: %d \r\n", accel_xyz[0], accel_xyz[1], accel_xyz[2]);
		
	}
	
	void imu01c::getGyro() {
		unsigned char gHi;
		unsigned char gLo;
		
		// x
		if (!i2c_read_byte(i2c_l3gd20h_addr, L3GD20H_REGISTER_OUT_X_H, &gHi))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get gyroscope x hi byte." << endl; 
		if (!i2c_read_byte(i2c_l3gd20h_addr, L3GD20H_REGISTER_OUT_X_L, &gLo))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get gyroscope x lo byte." << endl;
		gyro_xyz[X] = twos_comp((gHi << 8 | gLo), 16);
		
		// y
		if (!i2c_read_byte(i2c_l3gd20h_addr, L3GD20H_REGISTER_OUT_Y_H, &gHi))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get gyroscope y hi byte." << endl; 
		if (!i2c_read_byte(i2c_l3gd20h_addr, L3GD20H_REGISTER_OUT_Y_L, &gLo))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get gyroscope y lo byte." << endl;
		gyro_xyz[Y] = twos_comp((gHi << 8 | gLo), 16);
		
		// z
		if (!i2c_read_byte(i2c_l3gd20h_addr, L3GD20H_REGISTER_OUT_Z_H, &gHi))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get gyroscope z hi byte." << endl; 
		if (!i2c_read_byte(i2c_l3gd20h_addr, L3GD20H_REGISTER_OUT_Z_L, &gLo))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get gyroscope z lo byte." << endl;
		gyro_xyz[Z] = twos_comp((gHi << 8 | gLo), 16);
		
		// printf("X: %d, Y: %d, Z: %d \r\n", gyro_xyz[0], gyro_xyz[1], gyro_xyz[2]); 				  
	}
	
	void imu01c::getLSM303DTemperature() {
		unsigned char tHi;
		unsigned char tLo;
		
		if (!i2c_read_byte(i2c_lsm303d_addr, LSM303D_REGISTER_TEMP_OUT_H, &tHi))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get LSM303D thermometer hi byte." << endl; 
		if (!i2c_read_byte(i2c_lsm303d_addr, LSM303D_REGISTER_TEMP_OUT_L, &tLo))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get LSM303D thermometer lo byte." << endl;
		
		LSM303D_Temperature = twos_comp(((tHi << 8) | tLo) >> 4, 12);
		
		LSM303D_TemperatureDegrees = (LSM303D_Temperature / 8.0) + TEMPERATURE_CELSIUS_OFFSET;
	}
	
	void imu01c::getL3GD20HTemperature() {
		unsigned char byte;
		
		if (!i2c_read_byte(i2c_l3gd20h_addr, L3GD20H_REGISTER_OUT_TEMP, &byte))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get L3GD20H thermometer byte." << endl; 
		
		L3GD20H_Temperature = twos_comp(byte, 8);
		
		L3GD20H_TemperatureDegrees = (L3GD20H_Temperature / 8.0) + TEMPERATURE_CELSIUS_OFFSET;
	}
	
	void imu01c::getHeading() { 
/*	
		heading = 180 * atan2(mag_xyz[Y], mag_xyz[X]) / M_PI;
        if(heading < 0) {
            heading += 360;
		}

        headingDegrees = heading;

        double mtMagX = mag_xyz[X] / 450 * 100.0;
        double mtMagY = mag_xyz[Y] / 450 * 100.0;
        // double mtMagZ = mag_xyz[Z] / 400 * 100.0;	// unused

        heading = atan2(mtMagX, mtMagY);
		
		/* original code - it's 90 deg out */
        heading = atan2(mag_xyz[X], mag_xyz[Y]);

        if(heading < 0) {
            heading += 2*M_PI;
		}
        if(heading > 2*M_PI) {
            heading -= 2*M_PI;
		}

		headingDegrees = heading * (180.0 / M_PI); 		// radians to degrees
	}
	
	void imu01c::getTiltHeading() {
		double truncate[3] {0, 0, 0};
		double tiltcomp[3] {0, 0, 0};
		double pitch, roll;
		
		truncate[X] = copysign(min(fabs(accel_xyz[X]), 1.0), accel_xyz[X]);
		truncate[Y] = copysign(min(fabs(accel_xyz[Y]), 1.0), accel_xyz[Y]);
		truncate[Z] = copysign(min(fabs(accel_xyz[Z]), 1.0), accel_xyz[Z]);
		// truncate[X] = copysign(min(fabs(gyro_xyz[X]), 1.0), gyro_xyz[X]);
		// truncate[Y] = copysign(min(fabs(gyro_xyz[Y]), 1.0), gyro_xyz[Y]);
		// truncate[Z] = copysign(min(fabs(gyro_xyz[Z]), 1.0), gyro_xyz[Z]);
		// printf("TruncX: %.2f, TruncY: %.2f, TruncZ: %.2f \r\n",truncate[X], truncate[Y], truncate[Z]);
        try
		{	
            pitch = asin(-1*truncate[X]);
            
			// roll = math.asin(truncate[Y]/math.cos(pitch)) if abs(math.cos(pitch)) >= abs(truncate[Y]) else 0
			
			if(abs(cos(pitch)) >= abs(truncate[Y])) {
				roll = asin(truncate[Y]/cos(pitch));
			} else {
				roll = 0.0;
			}
			// printf("Roll: %.4f \r\n",roll);
            // set roll to zero if pitch approaches -1 or 1

            tiltcomp[X] = mag_xyz[X] * cos(pitch) + mag_xyz[Z] * sin(pitch);
            tiltcomp[Y] = mag_xyz[X] * sin(roll) * sin(pitch) + mag_xyz[Y] * cos(roll) - mag_xyz[Z] * sin(roll) * cos(pitch);
            tiltcomp[Z] = mag_xyz[X] * cos(roll) * sin(pitch) + mag_xyz[Y] * sin(roll) + mag_xyz[Z] * cos(roll) * cos(pitch);
            tiltHeading = atan2(tiltcomp[Y], tiltcomp[X]);
/*
            if(tiltHeading < 0) {
                tiltHeading += 2*M_PI;
			}
            if(tiltHeading > (2*M_PI)) {
				heading -= 2*M_PI;
                // tiltHeading -= 2*M_PI;
			}
/**/
            tiltHeadingDegrees = tiltHeading * (180.0 / M_PI);

			tiltHeadingDegrees += TILT_HEADING_OFFSET;
			if(tiltHeadingDegrees < 0) {
				tiltHeadingDegrees += 360;
			}
			if(tiltHeadingDegrees >= 360) {
				tiltHeadingDegrees -= 360;
			}
			
        } catch(exception& e) {
			printf("AccelX: %.2f, AccelY: %.2f \r\n",accel_xyz[X], accel_xyz[Y]);
			printf("TruncX: %.2f, TruncY: %.2f \r\n",truncate[X], truncate[Y]);
			// printf("Pitch: %.2f, cos(pitch): %.2f, Bool(cos(pitch)): %s \r\n",pitch, cos(pitch, bool(cos(pitch))));
			printf("Pitch: %.2f, cos(pitch): %.2f, Bool(cos(pitch)): %i  \r\n", pitch, cos(pitch), bool(cos(pitch)));
		}
	}
	
	void imu01c::update() {
		setup();
		getMag();
		getAccel();
		getGyro();
		getLSM303DTemperature();
		getL3GD20HTemperature();
		getHeading();
		getTiltHeading();
	}

   
	// Private functions

	int imu01c::twos_comp(int val, int bits)
	{
		// # Calculate the 2s complement of int:val #
		if ((val&(1 << (bits-1))) != 0)
		{
			val = val - (1 << bits);
		}

		return val;
	}
	
	bool imu01c::isMagReady() 
	{
		return true;
	}
	
    // Write one byte to the i2c device
    bool imu01c::i2c_write_byte(int addr, int reg, unsigned char c)
    {
		if (!check_int_range(c, 0, 0xFF)) return false;
		
        for (int i = 0; i < _i2c_retries; i++)
        {
            if (i2c_smbus_write_byte_data(addr, reg, c) >= 0) return true;
            lsleep(_i2c_retry_time);
        }
        cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": failed to write byte to the i2c device." << endl;
        return false;
    }

    // Read one byte from the i2c device
    bool imu01c::i2c_read_byte(int addr, int reg, unsigned char * c)
    {
		int res = -1;
        for (int i = 0; i < _i2c_retries; i++)
		{
			if ((res = i2c_smbus_read_byte_data(addr, reg)) >= 0)
			{
			*c = (res & 0xFF);
			return true;
			}
				lsleep(_i2c_retry_time);
		}
		cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": failed to read byte from the i2c device." << endl;
		return false;
    }

    // Suspend this thread for ms milliseconds.
    int imu01c::lsleep(long int ms)
    {
        int result = 0;
        struct timespec ts_sleep, ts_remaining;
		ts_remaining.tv_sec = (time_t)(ms / 1000);
		ts_remaining.tv_nsec = (long)(ms % 1000) * 1000000;
		
        do
        {
			ts_sleep = ts_remaining;
            result = nanosleep(&ts_sleep, &ts_remaining);
        }
		
        while ((errno == EINTR) && (result == -1));
		
        if (result == -1)
        {
            cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << " nanosleep() failed" << endl;
        }
		
        return result;
    }

	// Check bounds of an int value.
    bool imu01c::check_int_range(int value, int value_min, int value_max)
    {
        return ((value >= value_min) && (value <= value_max));
    }
} //namespace imu01cns
