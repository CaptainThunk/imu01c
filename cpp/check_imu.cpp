/*  
    This program checks a Pololu imu01c sensor device plugged into a Raspberry Pi.
    Works with sensor imu01c (dicontinued) (https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout/which-lsm303-do-i-have) It's the green one.
    imu01c Specsheet Reference https://cdn-shop.adafruit.com/datasheets/LSM303DLHC.PDF

    Stand-alone compilation
    -----------------------
    libi2c.so.0.1.0/0.1.1 must exist on the system.
    See build_cp.sh for building details.

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

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <iostream>
#include <exception>
#include <new>
#include <time.h>
#include "imu01c.h"

#define OK		"OK !\n"
#define NOK		"NOK !\n"
#define ONESECOND	1000
#define FIFTYMILLIS	50

using namespace std;
using namespace imu01cns;

// Suspend this thread for ms milliseconds.
int lsleep(long ms)
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

int main()
{
    int err = -1;

    try
    {
		imu01c * lsm = new imu01c();
		
		printf("\n[INFO] Checking imu01c Module...\n\n");
		
		if (!lsm->setup())
		{
			cerr << "[FATAL] Could not initialize imu01c module. Aborting.\n" << endl;
			goto error;
		}
		
		lsm->update();
		// lsm->getAccel();
		// lsm->getTemperature();
		
		printf("mX: %d, mY: %d, mZ: %d \r\n", lsm->mag_xyz[X], lsm->mag_xyz[Y], lsm->mag_xyz[Z]);
		printf("aX: %f, aY: %f, aZ: %f \r\n", lsm->accel_xyz[X], lsm->accel_xyz[Y], lsm->accel_xyz[Z]);
		printf("gX: %d, gY: %d, gZ: %d \r\n", lsm->gyro_xyz[X], lsm->gyro_xyz[Y], lsm->gyro_xyz[Z]);
		printf("LSM303D Temp: %d, Temp C: %.1f \r\n", lsm->LSM303D_Temperature, lsm->LSM303D_TemperatureDegrees);
		printf("L3GD20H Temp: %d, Temp C: %.1f \r\n", lsm->L3GD20H_Temperature, lsm->L3GD20H_TemperatureDegrees);
		printf("Heading: %.4f \r\n", lsm->headingDegrees);
		printf("Tilt Heading: %.4f \r\n", lsm->tiltHeadingDegrees);
error:
		// Get rid of LSM object and frees memory.
		if (lsm) delete lsm;
		
		
		
	// Getting to this point asserts no error occurred
	err = 0;
	printf("\n[SUCCESS] imu01c module is fully operational.\n\n");


	} catch(exception& e) {
		cerr << "[FATAL] could not create imu01c instance class : "
			<< e.what()
			<< "Aborting.\n" << endl;
			exit(-1);
	}

    return err;
}
