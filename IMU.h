#pragma once
#include <Arduino.h>

/* ============================================
This library simply wraps the example DMP code of Jeff Rowberg.
We didn't write most of the backend, the wrapped code, nor the comments in this library.
=============================================== */

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class 
// using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.

   For the Galileo Gen1/2 Boards, there is no INT pin support. Therefore
   the INT pin does not need to be connected, but you should work on getting
   the timing of the program right, so that there is no buffer overflow.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

class IMU
{
    private:
        // class default I2C address is 0x68
        // specific I2C addresses may be passed as a parameter here
        // AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
        // AD0 high = 0x69
        MPU6050 _mpu;
        //MPU6050 mpu(0x69); // <-- use for AD0 high

        //led state vars
        bool _blinkState = false;

        // MPU control/status vars
        bool _dmpReady = false;  // set true if DMP init was successful
        uint8_t _mpuIntStatus;   // holds actual interrupt status byte from MPU
        uint8_t _devStatus;      // return status after each device operation (0 = success, !0 = error)
        uint16_t _packetSize;    // expected DMP packet size (default is 42 bytes)
        uint16_t _fifoCount;     // count of all bytes currently in FIFO
        uint8_t _fifoBuffer[64]; // FIFO storage buffer

        // orientation/motion vars
        VectorFloat _gravity;    // [x, y, z]            gravity vector
        Quaternion _q;           // [w, x, y, z]         quaternion container
        float _euler[3];         // [psi, theta, phi]    Euler angle container
        float _ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

        //offset vars
        int _ax;
        int _ay;
        int _az;
        int _gx;
        int _gy;
        int _gz;

        // ================================================================
        // ===               INTERRUPT DETECTION DATA                   ===
        // ================================================================

        // This function is not required when using the Galileo 
        volatile bool _mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
        int _mpuInterruptPin = 38;
    
    public:
        IMU();

        void begin(bool verbose);

        void update();

        void setOffsets(int ax, int ay, int az, int gx, int gy, int gz);

        void dmpDataReady();

        float getYaw();

        float getPitch();

        float getRoll();
}