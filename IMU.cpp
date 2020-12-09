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

    //Initialize updateRaw() vars
    _waitingForData = false;
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
    if (verbose)
        Serial.println(F("Initializing I2C devices..."));
    _mpu.initialize();

    // verify connection
    if (verbose)
    {
        Serial.println(F("Testing device connections..."));
        Serial.println(F("MPU6050 connection "));
        Serial.print(_mpu.testConnection() ? F("successful") : F("failed"));
    }
    else if (!_mpu.testConnection())
    {
        Serial.println(F("MPU6050 connection failed"));
        while (true)
            ;
    }

    // wait for ready
    if (verbose)
    {
        Serial.println(F("\nSend any character to begin DMP programming and demo: "));
        while (Serial.available() && Serial.read()); // empty buffer
        while (!Serial.available());                 // wait for data
        while (Serial.available() && Serial.read()); // empty buffer again
    }
    else
    {
        while (Serial.available() && Serial.read()); // empty buffer
    }
    

    // load and configure the DMP
    if (verbose)
        Serial.println(F("Initializing DMP..."));
    uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
    devStatus = _mpu.dmpInitialize(verbose);

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
        if (verbose)
            Serial.println(F("Enabling DMP..."));
        _mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        if (verbose)
            Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(_mpuInterruptPin), *externalDmpDataReady, RISING);
        _mpuIntStatus = _mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        if (verbose)
            Serial.println(F("DMP ready! Waiting for first interrupt..."));
        _dmpReady = true;

        // get expected DMP packet size for later comparison
        _packetSize = _mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        if (verbose)
        {
            Serial.print(F("DMP Initialization failed (code "));
            Serial.print(devStatus);
            Serial.println(F(")"));
        }
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void IMU::updateRaw()
{
    // if programming failed, don't try to do anything
    if (!_dmpReady) return;

    //Only enter if not already executed this and did not get halted downstream
    if(!_waitingForData)
    {
        // check for MPU interrupt or extra packet(s) available otherwise exit
        #ifdef ARDUINO_BOARD
            if (!_mpuInterrupt && _fifoCount < _packetSize) return;
        #endif

        #ifdef GALILEO_BOARD
            delay(10);
        #endif

        // reset interrupt flag and get INT_STATUS byte
        _mpuInterrupt = false;
        _mpuIntStatus = _mpu.getIntStatus();

        // get current FIFO count
        _fifoCount = _mpu.getFIFOCount();
    }

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((_mpuIntStatus & 0x10) || _fifoCount == 1024) {
        // reset so we can continue cleanly
        _mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    //or if only reentering from previous halt at packet size
    } else if (_mpuIntStatus & 0x02 || _waitingForData) {
        // wait for correct available data length, should be a VERY short wait
        if (_fifoCount < _packetSize)
        {
            _fifoCount = _mpu.getFIFOCount();

            //Indicate that everything up till here has already been run
            _waitingForData = true;
            return;
        }
        else
            //Reset flag
            _waitingForData = false;

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
            
            _yawRaw = _ypr[0] * 180 / M_PI;
            _pitchRaw = _ypr[1] * 180 / M_PI;
            _rollRaw = _ypr[2] * 180 / M_PI;
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

float IMU::getYawRaw() const
{
    return _yawRaw;
}

float IMU::getPitchRaw() const
{
    return _pitchRaw;
}

float IMU::getRollRaw() const
{
    return _rollRaw;
}