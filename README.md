MPU-6050
========

Basic MPU-6050 Arduino sketch of sensor function

For a discussion of performance on various microcontroller platforms, uses and limitations of the MPU-6050 see ![here.](https://github.com/kriswiner/MPU-6050/wiki/Affordable-9-DoF-Sensor-Fusion)

I have written a report from the June 11-12, 2014 ![Invensense Developers Conference.](https://github.com/kriswiner/MPU-6050/wiki/2014-Invensense-Developer%27s-Conference)

This sketch demonstrates  MPU-6050 basic functionality including initialization, accelerometer and gyro calibration, sleep mode functionality as well as parameterizing the register addresses. Added display functions to allow display to on-breadboard monitor. 
No DMP use. We just want to get out the accelerations, temperature, and gyro readings.
 
Runs on 3.3V 8 MHz Pro Mini and Teensy 3.1.

Added quaternion filter based on Madgwick's open-source sensor fusion algorithms. The MPU-6050 lacks a magnetic vector for absolute orientation estimation as is possible with the MPU-9150 or LSM9DS0. This algorithm allows estimation of quaternions and relative orientation, allowing output of Yaw, Pitch, and Roll which is subject to Yaw drift due to gyro bias drift. Despite the inclusion of a gyro bias drift correction component to the sensor fusion algorithm, Yaw drift is about half a degree per minute or less, which is not too bad. In principle, Yaw should not be possible to estimate with only a single absolute reference (gravity down), yet this algorithm does a good job of estimating relative Yaw with good stability over short time scales.

I have ![added](https://github.com/kriswiner/MPU-6050/tree/master/STM32F401) code compiled with the mbed compiler to run 6-axis sensor fusion using the STM32MF401 ARM processor and the MPU-6050.  Why would this be necessary or desirable?

The filter runs at a  200 Hz update rate on a 3.3 V 8 MHz Pro Mini AVR (Arduino) microcontroller.

The filter runs at a 3200 Hz update rate on a 3.3 V 96 MHz Teensy 3.1 ARM microcontroller.

The filter runs at a 5500 Hz update rate on a 3.3 V 84 MHz STM32F401 ARM microcontroller.

One doesn't need more that about 1000 Hz sensor fusion filter update rate to get optimal performance from 6- and 9-axis motion sensors, but the power consumption is proportional to the microcontroller clock speed. If that 1000 Hz rate can be achieved at a lower clock speed, then lower power consumption can be achieved. This is a critical consideration for portable (wearable) motion sensing and motion control devices.

Let's assume the sensor fusion filter update rate is linearly proportional to the clock speed (a pretty good assumption). Then to get 1000 Hz update rates for the Teensy only requires 96/3.2 = 30 MHz clock speed. In fact, at 24 MHz clock speed the update rate with the Teensy 3.1 is 1365 Hz. For the STM32F401, the clock speed needs to be only 84/5.5 = 16 MHz to reach a sensor fusion filter update rate of 1000 Hz. In practice, the clock speeds are not arbitrary but must meet certain constraints; it might not be possible to run the STM32F401 at such a low speed. So far, I have only run it at 84 and 42 MHz; at 42 MHz I got sensor fusion filter update rates of ~4400 Hz. It appears that the STM32F401 would require less than half the power compared to the Teensy 3.1 to achieve the same level of sensor fusion performance. Meaning that all else being equal (it never is!), a wearable device using the STM32F401 as the microcontroller could last twice as long before battery change or charging is required. This is a major commercial advantage and why the STM32 M4 Cortex family of processors is so interesting.
