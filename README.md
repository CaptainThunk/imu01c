# imu01c

Python script and C, C++ routines to run the Pololu MinIMU-9 v3 sensor (imu01c) for the Rasberry Pi

Accelerometer, Magnetometer, Gyroscope and Thermometer

Works with sensor MinIMU-9 v3 (dicontinued) (https://www.pololu.com/product/2468)
MinIMU-9 v3 Specsheet Reference https://www.pololu.com/file/0J703/LSM303D.pdf / https://www.pololu.com/file/0J731/L3GD20H.pdf

Python requires the i2cdevice library
pip install i2cdevice

C requires the following packages:
sudo apt-get install i2c-tools
sudo apt-get install libi2c-dev

The C routine is very sketchy and creates a pseudo class in a very crude manner.

Enjoy.

September 3rd 2021

Captain Thunk
