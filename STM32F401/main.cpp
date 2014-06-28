
/* MPU6050 Basic Example Code
 by: Kris Winer
 date: May 1, 2014
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Demonstrate  MPU-6050 basic functionality including initialization, accelerometer trimming, sleep mode functionality as well as
 parameterizing the register addresses. Added display functions to allow display to on breadboard monitor. 
 No DMP use. We just want to get out the accelerations, temperature, and gyro readings.
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors worked for me. They should be on the breakout
 board.
 
 Hardware setup:
 MPU6050 Breakout --------- Arduino
 3.3V --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 
  Note: The MPU6050 is an I2C sensor and uses the Arduino Wire library. 
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */
 
#include "mbed.h"
#include "MPU6050.h"
#include "N5110.h"

// Using NOKIA 5110 monochrome 84 x 48 pixel display
// pin 9 - Serial clock out (SCLK)
// pin 8 - Serial data out (DIN)
// pin 7 - Data/Command select (D/C)
// pin 5 - LCD chip select (CS)
// pin 6 - LCD reset (RST)
//Adafruit_PCD8544 display = Adafruit_PCD8544(9, 8, 7, 5, 6);

float sum = 0;
uint32_t sumCount = 0;

   MPU6050 mpu6050;
   
   Timer t;

   Serial pc(USBTX, USBRX); // tx, rx

   //        VCC,   SCE,  RST,  D/C,  MOSI,S CLK, LED
   N5110 lcd(PA_8, PB_10, PA_9, PA_6, PA_7, PA_5, PC_7);
        
int main()
{
  pc.baud(9600);  

  //Set up I2C
  i2c.frequency(400000);  // use fast (400 kHz) I2C   
  
  t.start();        
  
  lcd.init();
  lcd.setBrightness(0.05);
  
    
  // Read the WHO_AM_I register, this is a good test of communication
  uint8_t whoami = mpu6050.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
  pc.printf("I AM 0x%x\n\r", whoami); pc.printf("I SHOULD BE 0x68\n\r");
  
  if (whoami == 0x68) // WHO_AM_I should always be 0x68
  {  
    pc.printf("MPU6050 is online...");
    wait(1);
    lcd.clear();
    lcd.printString("MPU6050 OK", 0, 0);

    
    mpu6050.MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values
    pc.printf("x-axis self test: acceleration trim within : "); pc.printf("%f", SelfTest[0]); pc.printf("% of factory value \n\r");
    pc.printf("y-axis self test: acceleration trim within : "); pc.printf("%f", SelfTest[1]); pc.printf("% of factory value \n\r");
    pc.printf("z-axis self test: acceleration trim within : "); pc.printf("%f", SelfTest[2]); pc.printf("% of factory value \n\r");
    pc.printf("x-axis self test: gyration trim within : "); pc.printf("%f", SelfTest[3]); pc.printf("% of factory value \n\r");
    pc.printf("y-axis self test: gyration trim within : "); pc.printf("%f", SelfTest[4]); pc.printf("% of factory value \n\r");
    pc.printf("z-axis self test: gyration trim within : "); pc.printf("%f", SelfTest[5]); pc.printf("% of factory value \n\r");
    wait(1);

    if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) 
    {
    mpu6050.resetMPU6050(); // Reset registers to default in preparation for device calibration
    mpu6050.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
    mpu6050.initMPU6050(); pc.printf("MPU6050 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

    lcd.clear();
    lcd.printString("MPU6050", 0, 0);
    lcd.printString("pass self test", 0, 1);
    lcd.printString("initializing", 0, 2);  
    wait(2);
       }
    else
    {
    pc.printf("Device did not the pass self-test!\n\r");
 
       lcd.clear();
       lcd.printString("MPU6050", 0, 0);
       lcd.printString("no pass", 0, 1);
       lcd.printString("self test", 0, 2);      
      }
    }
    else
    {
    pc.printf("Could not connect to MPU6050: \n\r");
    pc.printf("%#x \n",  whoami);
 
    lcd.clear();
    lcd.printString("MPU6050", 0, 0);
    lcd.printString("no connection", 0, 1);
    lcd.printString("0x", 0, 2);  lcd.setXYAddress(20, 2); lcd.printChar(whoami);
 
    while(1) ; // Loop forever if communication doesn't happen
  }



 while(1) {
  
  // If data ready bit set, all data registers have new data
  if(mpu6050.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt
    mpu6050.readAccelData(accelCount);  // Read the x/y/z adc values
    mpu6050.getAres();
    
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes - accelBias[1];   
    az = (float)accelCount[2]*aRes - accelBias[2];  
   
    mpu6050.readGyroData(gyroCount);  // Read the x/y/z adc values
    mpu6050.getGres();
 
    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes; // - gyroBias[0];  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes; // - gyroBias[1];  
    gz = (float)gyroCount[2]*gRes; // - gyroBias[2];   

    tempCount = mpu6050.readTempData();  // Read the x/y/z adc values
    temperature = (tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
   }  
   
    Now = t.read_us();
    deltat = (float)((Now - lastUpdate)/1000000.0f) ; // set integration time by time elapsed since last filter update
    lastUpdate = Now;
    
    sum += deltat;
    sumCount++;
    
    if(lastUpdate - firstUpdate > 10000000.0f) {
     beta = 0.04;  // decrease filter gain after stabilized
     zeta = 0.015; // increasey bias drift gain after stabilized
    }
    
   // Pass gyro rate as rad/s
    mpu6050.MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f);

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = t.read_ms() - count;
    if (delt_t > 500) { // update LCD once per half-second independent of read rate

    pc.printf("ax = %f", 1000*ax); 
    pc.printf(" ay = %f", 1000*ay); 
    pc.printf(" az = %f  mg\n\r", 1000*az); 

    pc.printf("gx = %f", gx); 
    pc.printf(" gy = %f", gy); 
    pc.printf(" gz = %f  deg/s\n\r", gz); 
    
    pc.printf(" temperature = %f  C\n\r", temperature); 
    
    pc.printf("q0 = %f\n\r", q[0]);
    pc.printf("q1 = %f\n\r", q[1]);
    pc.printf("q2 = %f\n\r", q[2]);
    pc.printf("q3 = %f\n\r", q[3]);      
    
    lcd.clear();
    lcd.printString("MPU6050", 0, 0);
    lcd.printString("x   y   z", 0, 1);
    lcd.setXYAddress(0, 2); lcd.printChar((char)(1000*ax));
    lcd.setXYAddress(20, 2); lcd.printChar((char)(1000*ay));
    lcd.setXYAddress(40, 2); lcd.printChar((char)(1000*az)); lcd.printString("mg", 66, 2);
    
    
  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    roll  *= 180.0f / PI;

//    pc.printf("Yaw, Pitch, Roll: \n\r");
//    pc.printf("%f", yaw);
//    pc.printf(", ");
//    pc.printf("%f", pitch);
//    pc.printf(", ");
//    pc.printf("%f\n\r", roll);
//    pc.printf("average rate = "); pc.printf("%f", (sumCount/sum)); pc.printf(" Hz\n\r");

     pc.printf("Yaw, Pitch, Roll: %f %f %f\n\r", yaw, pitch, roll);
     pc.printf("average rate = %f\n\r", (float) sumCount/sum);
 
    myled= !myled;
    count = t.read_ms(); 
    sum = 0;
    sumCount = 0; 
}
}
 
 }
