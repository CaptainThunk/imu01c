from .imu01c import imu01c, IMU01C_X, IMU01C_Y, IMU01C_Z

__version__ = '0.0.1'

imu = imu01c()

# functions
# setup = imu.setup

getMag = imu.getMag
getAccel = imu.getAccel
getGyro = imu.getGyro
getLSM303DTemp = imu.getLSM303DTemp
getL3GD20HTemp = imu.getL3GD20HTemp
getHeading = imu.getHeading
getTiltHeading = imu.getTiltHeading

isMagReady = imu.isMagReady

update = imu.update

# vars
mag = imu.mag
accel = imu.accel
gyro = imu.gyro
tiltcomp = imu.tiltcomp
heading = imu.heading
headingDegrees = imu.headingDegrees
tiltHeading = imu.tiltHeading
tiltHeadingDegrees = imu.tiltHeadingDegrees
