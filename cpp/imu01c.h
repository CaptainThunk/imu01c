/*  
    C++ interface for controlling a Pololu imu01c sensor on a Raspberry Pi.
    Class built upon SMBus and i2C standard APIs.

    Works with sensor imu01c (dicontinued) (https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout/which-lsm303-do-i-have) It's the green one.
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

#ifndef IMU01C_H
#define IMU01C_H

#include <iostream>
#include <cstdio>
#include <cstdint>

#define MISSING_FUNC_1  "Error: Adapter does not have "
#define MISSING_FUNC_2  " capability.\n"

// LSM303 Address ###
#define LSM303D_ADDRESS 				0x1D // Default address Magnetometer/Accelerometer
#define L3GD20H_ADDRESS					0x6B // Default address Gyroscope

// registers

// L3GD20H Gyroscope / Thermometer

// reserved 0x00-0x0E
#define L3GD20H_WHO_AM_I				0x0F	// r	
// reserved 0x10-0x1F
#define L3GD20H_REGISTER_CTRL1			0x20	// rw
#define L3GD20H_REGISTER_CTRL2			0x21	// rw
#define L3GD20H_REGISTER_CTRL3			0x22	// rw
#define L3GD20H_REGISTER_CTRL4			0x23	// rw
#define L3GD20H_REGISTER_CTRL5			0x24	// rw
#define L3GD20H_REGISTER_REFERENCE		0x25	// rw
#define L3GD20H_REGISTER_OUT_TEMP		0x26	// r
#define L3GD20H_REGISTER_STATUS			0x27	// r
#define L3GD20H_REGISTER_OUT_X_L		0x28	// r
#define L3GD20H_REGISTER_OUT_X_H		0x29	// r
#define L3GD20H_REGISTER_OUT_Y_L		0x2A	// r
#define L3GD20H_REGISTER_OUT_Y_H		0x2B	// r
#define L3GD20H_REGISTER_OUT_Z_L		0x2C	// r
#define L3GD20H_REGISTER_OUT_Z_H		0x2D	// r
#define L3GD20H_REGISTER_FIFO_CTRL		0x2E	// rw
#define L3GD20H_REGISTER_FIFO_SRC		0x2F	// r
#define L3GD20H_REGISTER_IG_CFG			0x30	// rw
#define L3GD20H_REGISTER_IG_SRC			0x31	// r
#define L3GD20H_REGISTER_IG_THS_XH		0x32	// rw
#define L3GD20H_REGISTER_IG_THS_XL		0x33	// rw
#define L3GD20H_REGISTER_IG_THS_YH		0x34	// rw
#define L3GD20H_REGISTER_IG_THS_YL		0x35	// rw
#define L3GD20H_REGISTER_IG_THS_ZH		0x36	// rw
#define L3GD20H_REGISTER_IG_THS_ZL		0x37	// rw
#define L3GD20H_REGISTER_IG_DURATION	0x38	// rw
#define L3GD20H_REGISTER_LOW_ODR		0x39	// rw

// LSM303D Accelerometer / Magnetometer / Thermometer

#define LSM303D_REGISTER_TEMP_OUT_L		0x05
#define LSM303D_REGISTER_TEMP_OUT_H		0x06
#define LSM303D_REGISTER_STATUS_M		0x07
#define LSM303D_REGISTER_OUT_X_L_M		0x08
#define LSM303D_REGISTER_OUT_X_H_M		0x09
#define LSM303D_REGISTER_OUT_Y_L_M		0x0A
#define LSM303D_REGISTER_OUT_Y_H_M		0x0B
#define LSM303D_REGISTER_OUT_Z_L_M		0x0C
#define LSM303D_REGISTER_OUT_Z_H_M		0x0D
// reserved 0x0E
#define LSM303D_REGISTER_WHO_AM_I		0x0F
// reserved 0x10-0x11
#define LSM303D_REGISTER_INT_CTRL_M		0x12
#define LSM303D_REGISTER_INT_SRC_M		0x13
#define LSM303D_REGISTER_INT_THS_L_M	0x14
#define LSM303D_REGISTER_INT_THS_H_M	0x15
#define LSM303D_REGISTER_OFFET_X_L_M	0x16
#define LSM303D_REGISTER_OFFET_X_H_M	0x17
#define LSM303D_REGISTER_OFFET_Y_L_M	0x18
#define LSM303D_REGISTER_OFFET_Y_H_M	0x19
#define LSM303D_REGISTER_OFFET_Z_L_M	0x1A
#define LSM303D_REGISTER_OFFET_Z_H_M	0x1B
#define LSM303D_REGISTER_REFERENCE_X	0x1C
#define LSM303D_REGISTER_REFERENCE_Y	0x1D
#define LSM303D_REGISTER_REFERENCE_Z	0x1E
#define LSM303D_REGISTER_CTRL0			0x1F 
#define LSM303D_REGISTER_CTRL1			0x20        
#define LSM303D_REGISTER_CTRL2			0x21        
#define LSM303D_REGISTER_CTRL3			0x22        
#define LSM303D_REGISTER_CTRL4			0x23       
#define LSM303D_REGISTER_CTRL5			0x24        
#define LSM303D_REGISTER_CTRL6			0x25        
#define LSM303D_REGISTER_CTRL7			0x26        
#define LSM303D_REGISTER_STATUS_A		0x27       
#define LSM303D_REGISTER_OUT_X_L_A		0x28
#define LSM303D_REGISTER_OUT_X_H_A		0x29
#define LSM303D_REGISTER_OUT_Y_L_A		0x2A
#define LSM303D_REGISTER_OUT_Y_H_A		0x2B
#define LSM303D_REGISTER_OUT_Z_L_A		0x2C
#define LSM303D_REGISTER_OUT_Z_H_A		0x2D
#define LSM303D_REGISTER_FIFO_CTRL		0x2E
#define LSM303D_REGISTER_FIFO_SRC		0x2F
#define LSM303D_REGISTER_IG_CFG1		0x30
#define LSM303D_REGISTER_IG_SRC1		0x31
#define LSM303D_REGISTER_IG_THS1		0x32
#define LSM303D_REGISTER_IG_DUR1		0x33
#define LSM303D_REGISTER_IG_CFG2		0x34
#define LSM303D_REGISTER_IG_SRC2		0x35
#define LSM303D_REGISTER_IG_THS2 		0x36
#define LSM303D_REGISTER_IG_DUR2		0x37
#define LSM303D_REGISTER_CLICK_CFG		0x38
#define LSM303D_REGISTER_CLICK_SRC		0x39
#define LSM303D_REGISTER_CLICK_THS		0x3A
#define LSM303D_REGISTER_TIME_LIMIT		0x3B
#define LSM303D_REGISTER_TIME_LATENCY	0x3C
#define LSM303D_REGISTER_TIME_WINDOW	0x3D
#define LSM303D_REGISTER_ACT_THS		0x3E
#define LSM303D_REGISTER_ACT_DUR		0x3F

#define LSM303D_MAG_SCALE_2				0x00	// Magnetic Scale +/2 Guass
#define LSM303D_MAG_SCALE_4				0x20	// Magnetic Scale +/4 Guass
#define LSM303D_MAG_SCALE_8				0x40	// Magnetic Scale +/8 Guass
#define LSM303D_MAG_SCALE_12			0x60	// Magnetic Scale +/12 Guass

#define LSM303D_ACCEL_SCALE 			2 		// +/- 2g
#define TEMPERATURE_CELSIUS_OFFSET 		14		// offset for temperature calc (guesswork) originally 18
#define TILT_HEADING_OFFSET 			0		// offset for Tilt Heading for correction

#define X 0
#define Y 1
#define Z 2

#define max(x, y) (((x) > (y)) ? (x) : (y))
#define min(x, y) (((x) < (y)) ? (x) : (y))
 
namespace imu01cns {

    // Communicates with Pan-Tilt HAT over i2c
    // to control pan, tilt and light functions

    class imu01c
    {

		public:
			imu01c();
			~imu01c();

			bool setup();
			void getMag();
			void getAccel();
			void getGyro();
			void getLSM303DTemperature();
			void getL3GD20HTemperature();
			void getHeading();
			void getTiltHeading();
			void update();
			
			double accel_xyz[3] {0, 0, 0};
			int mag_xyz[3] {0, 0, 0};
			int gyro_xyz[3] {0, 0, 0};
			double heading = 0;
			double headingDegrees = 0;
			double tiltHeading = 0;
			double tiltHeadingDegrees = 0;
			int LSM303D_Temperature = 0;
			int L3GD20H_Temperature = 0;
			double LSM303D_TemperatureDegrees = 0;
			double L3GD20H_TemperatureDegrees = 0;

		private:
			bool i2c_write_byte(int addr, int reg, unsigned char c);
			bool i2c_read_byte(int addr, int reg, unsigned char * c);
			bool check_int_range(int value, int value_min, int value_max);
			int  lsleep(long int ms);
			int twos_comp(int val, int bits);
			bool isMagReady();

			bool _is_setup = false;
			unsigned int _idle_timeout = 500;
			int _i2c_retries = 10;
			unsigned int _i2c_retry_time = 50;
			int i2c_lsm303d_addr = 0, i2c_l3gd20h_addr = 0, _length = 0;
			unsigned char _buffer[60] = {0};
			
			
	}; // class imu01c

} // namespace imu01cns

#endif

