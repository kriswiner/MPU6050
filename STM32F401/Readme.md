I have started to reproduce the motion sensor sketches I wrote for the 3.3 V 8 MHz Pro Mini and the Teensy 3.1 
for 6- and 9-axis motion sensors with open-source sensor fusion filters on another ARM family of processors, 
the STM32M4 Cortex family, namely the STM32F401.

The STM32F401 cannot be programmed with an Arduino-like language but the mbed compiler makes the translation from Arduino to
something that can run the STM32F401 pretty easy. The mbed compiler is a middle ground between the ease of programming 
an Arduino and the full-blown tool-chain approach to programming modern ARM devices.

My intent here is to have all of the 6- and 9-axis motion sensors and their sensor fusion algorithms working on the 
STM32F401. I am designing portable motion sensing and control devices and have pretty much decided that AVR processors 
are not up to the task. I am currently targeting the Teensy 3.1 ARM M4 Cortex device since it is essentially a commodity 
available at $17 each from OSHPark.com, open-source, and, I believe, amenable to modular re-configuration by addition of 
small boards to get me the motion sensing, BLE commo, battery charging, and motor control functions I want in a very small
total package. However, I am exploring the use of the STM32F401 family also since it is inexpensive, as capable if not
more so than the Teensy 3.1 ARM processor, and can operate at extremely low power, which is essential for a portable
device.

So I am on a parallel development path; which one of these ARM solutions wins time will tell...
