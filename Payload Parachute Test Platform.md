<h1>Payload Parachute Test Platform (PPTP)</h1>

<h3>Boards Used:</h3>
Sparkfun GPS Breakout - SAM-M10Q                https://www.sparkfun.com/products/21834 <br>
Sparkfun 9DoF IMU Breakout - ICM-20948          https://www.sparkfun.com/products/15335 <br>
Sparkfun MicroSD Breakout                       https://www.sparkfun.com/products/13743 <br>
Adafruit BMP388 (Altimeter & Temperature)       https://a.co/d/gY7BUB1 <br>
TCA9548A (Multiplexer)                          https://a.co/d/ihOqFEC <br>
Y4183 (Buck Converter) <br>
MS24 (Servo) <br>
Arduino Uno <br>

![image](https://github.com/ZachGregory/CUinSpace-payload-2024/assets/123396117/e02fb189-9f5c-4467-bd2e-2b5b17546c4b)


Materials for learning arduino and the sensors:
You'll want to first download the arduino IDE from here: https://www.arduino.cc/en/software

Start here to learn some basics of arduino. 
For now try to output to the onboard led, start the serial connection and output something to it. Just play around with it and get a feel for the syntax. 
Blink sample code:
https://learn.adafruit.com/adafruit-arduino-lesson-1-blink/loading-the-blink-example

Serial output sample code: https://www.arduino.cc/reference/en/language/functions/communication/serial/print/

Since we are using multiple I2C sensors that may try to use the same I2C address we are using a multiplexer. It deals with assigning different addresses to each sensor and lets us swap between them easily. 
https://electropeak.com/learn/connect-multiple-i2c-devices-to-arduino-using-i2c-multiplexer-tca9548a/
