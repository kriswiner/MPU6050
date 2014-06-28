MPU-6050
========

Basic MPU-6050 Arduino sketch of sensor function

For a discussion of performance on various microcontroller platforms, uses and limitations of the MPU-6050 see here: https://github.com/kriswiner/MPU-6050/wiki/Affordable-9-DoF-Sensor-Fusion

For a report from the recent Invensense Developers Conference see here:https://github.com/kriswiner/MPU-6050/wiki/2014-Invensense-Developer%27s-Conference

This sketch demonstrates  MPU-6050 basic functionality including initialization, accelerometer and gyro calibration, sleep mode functionality as well as parameterizing the register addresses. Added display functions to allow display to on-breadboard monitor. 
No DMP use. We just want to get out the accelerations, temperature, and gyro readings.
 
Runs on 3.3V 8 MHz Pro Mini and Teensy 3.1.

Added quaternion filter based on Madgwick's open-source sensor fusion algorithms. The MPU-6050 lacks a magnetic vector for absolute orientation estimation as is possible with the MPU-9150 or LSM9DS0. This algorithm allows estimation of quaternions and relative orientation, allowing output of Yaw, Pitch, and Roll which is subject to Yaw drift due to gyro bias drift. Despite the inclusion of a gyro bias drift correction component to the sensor fusion algorithm, Yaw drift is about half a degree per minute or less, which is not too bad. In principle, Yaw should not be possible to estimate with only a single absolute reference (gravity down), yet this algorithm does a good job of estimating relative Yaw with good stability over short time scales.

I have added code compiled with the mbed compiler to run 6-axis sensor fusion using the STM32MF401 ARM processor to run the MPU-6050.  Why would this be necessary or desirable?

The filter runs at a  200 Hz update rate on a 3.3 V 8 MHz Pro Mini AVR (Arduino) microcontroller.

The filter runs at a 3200 Hz update rate on a 3.3 V 96 MHz Teensy 3.1 ARM microcontroller.

The filter runs at a 5500 Hz update rate on a 3.3 V 84 MHz STM32F401 ARM microcontroller.

One doesn't need more that about 1000 Hz sensor fusion filter update rate to get optimal performance from 6- and 9-axis motion sensors, but the power consumption is proportional to the microcontroller clock speed. If that 1000 Hz rate can be achieved at a lower clock speed, then lower power consumption can be achieved. This is a critical consideration for portable (wearable) motion sensing and moion control devices.
