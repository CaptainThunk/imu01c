/*  
    This program checks a Pololu MinIMU-9 v3 sensor device (imu01c) plugged into a Raspberry Pi.
    Works with sensor MinIMU-9 v3 (dicontinued) (https://www.pololu.com/product/2468)
    MinIMU-9 v3 Specsheet Reference https://www.pololu.com/file/0J703/LSM303D.pdf / https://www.pololu.com/file/0J731/L3GD20H.pdf

    Stand-alone compilation
    -----------------------
    libi2c.so.0.1.0/0.1.1 must exist on the system.
    See build_imu.sh for building details.

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

// #include <errno.h>
// #include <string.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <math.h>
// #include <unistd.h>
// #include <iostream>
// #include <exception>
// #include <new>
// #include <time.h>
#include "imu01c.c"

int main()
{
    int err = -1;

	// Now, define an instance of our struct
    struct imu01cClass imu01c;
    imu01c.is_setup = false;
    imu01c.setup = imu01c_setup;
	
	printf("\n[INFO] Checking imu01c Module...\n\n");
		
	if (!imu01c.setup(&imu01c))
	{
		printf("[FATAL] Could not initialize imu01c module. Aborting.\n");
		return err;
	}
	
	imu01c.update(&imu01c);
	
	printf("mX: %d, mY: %d, mZ: %d \r\n", imu01c.mag_xyz[X], imu01c.mag_xyz[Y], imu01c.mag_xyz[Z]);
	printf("aX: %f, aY: %f, aZ: %f \r\n", imu01c.accel_xyz[X], imu01c.accel_xyz[Y], imu01c.accel_xyz[Z]);
	printf("gX: %d, gY: %d, gZ: %d \r\n", imu01c.gyro_xyz[X], imu01c.gyro_xyz[Y], imu01c.gyro_xyz[Z]);
	printf("LSM303D Temp: %d, Temp C: %.1f \r\n", imu01c.LSM303D_Temperature, imu01c.LSM303D_TemperatureDegrees);
	printf("L3GD20H Temp: %d, Temp C: %.1f \r\n", imu01c.L3GD20H_Temperature, imu01c.L3GD20H_TemperatureDegrees);
	printf("Heading: %.4f \r\n", imu01c.headingDegrees);
	printf("Tilt Heading: %.4f \r\n", imu01c.tiltHeadingDegrees);

	// Getting to this point asserts no error occurred
	err = 0;
	printf("\n[SUCCESS] imu01c module is fully operational.\n\n");

    return err;
}
