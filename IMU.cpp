#include <Arduino.h>
#include "IMU.h"

#define OUTPUT_READABLE_YAWPITCHROLL

// Unccomment if you are using an Arduino-Style Board
#define ARDUINO_BOARD

// Uncomment if you are using a Galileo Gen1 / 2 Board
// #define GALILEO_BOARD

#define LED_PIN 13      // (Galileo/Arduino is 13)

IMU::IMU(unsigned int mpuInterruptPin)
{
    //Initialize MPU interrupt pin
    _mpuInterruptPin = mpuInterruptPin;

    //Initialize led vars
    _blinkState = false;

    //Initialize dmp vars
    _dmpReady = false;

    //Initialize offset vars
    _ax = 0;
    _ay = 0;
    _az = 0;
    _gx = 0;
    _gy = 0;
    _gz = 0;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void IMU::begin(void (*externalDmpDataReady)(), bool verbose /*=false*/)
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        // TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    _mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(F("MPU6050 connection "));
    Serial.print(_mpu.testConnection() ? F("successful") : F("failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
    devStatus = _mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    _mpu.setXGyroOffset(_gx);
    _mpu.setYGyroOffset(_gy);
    _mpu.setZGyroOffset(_gz);
    _mpu.setXAccelOffset(_ax);
    _mpu.setYAccelOffset(_ay);
    _mpu.setZAccelOffset(_az); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        _mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(_mpuInterruptPin), *externalDmpDataReady, RISING);
        _mpuIntStatus = _mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        _dmpReady = true;

        // get expected DMP packet size for later comparison
        _packetSize = _mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void IMU::update()
{
    // if programming failed, don't try to do anything
    if (!_dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available

    #ifdef ARDUINO_BOARD
        while (!_mpuInterrupt && _fifoCount < _packetSize) {
        }
    #endif

    #ifdef GALILEO_BOARD
        delay(10);
    #endif

    // reset interrupt flag and get INT_STATUS byte
    _mpuInterrupt = false;
    _mpuIntStatus = _mpu.getIntStatus();

    // get current FIFO count
    _fifoCount = _mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((_mpuIntStatus & 0x10) || _fifoCount == 1024) {
        // reset so we can continue cleanly
        _mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (_mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (_fifoCount < _packetSize) _fifoCount = _mpu.getFIFOCount();

        // read a packet from FIFO
        _mpu.getFIFOBytes(_fifoBuffer, _packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        _fifoCount -= _packetSize;


        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            _mpu.dmpGetQuaternion(&_q, _fifoBuffer);
            _mpu.dmpGetGravity(&_gravity, &_q);
            _mpu.dmpGetYawPitchRoll(_ypr, &_q, &_gravity);
        #endif

        // blink LED to indicate activity
        _blinkState = !_blinkState;
        digitalWrite(LED_PIN, _blinkState);
    }
}

// ================================================================
// ===                  OFFSET INITIALIZATION                   ===
// ================================================================

void IMU::setOffsets(int ax, int ay, int az, int gx, int gy, int gz)
{
    //Set offset vars
    _ax = ax;
    _ay = ay;
    _az = az;
    _gx = gx;
    _gy = gy;
    _gz = gz;
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

void IMU::dmpDataReady()
{
    _mpuInterrupt = true;
}

float IMU::getYaw()
{
    return _ypr[0] * 180 / M_PI;
}

float IMU::getPitch()
{
    return _ypr[1] * 180 / M_PI;
}

float IMU::getRoll()
{
    return _ypr[2] * 180 / M_PI;
}