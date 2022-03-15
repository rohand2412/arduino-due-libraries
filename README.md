# Arduino Due Libraries

This library is written for [Arduino](https://www.arduino.cc/en/Guide/Introduction) based robotics applications and has been tested inside out on the [Arduino Due](https://docs.arduino.cc/hardware/due). Feel free to make a pull request to add support for other platforms. The classes are structured for convenience and minimal boilerplate code from a user standpoint.

## List of Classes

Below is a list of all the classes that are unique to this repository and completely written from scratch. For more information on the individual classes see the brief descriptions at the top of their files.

+ Button
+ Button_Wrapper
+ Encoder_Wrapper
+ IMU
+ IMU_Wrapper
+ LED
+ Motor_Wrapper
+ RGB_LED
+ Robot
+ Serial_Wrapper*
+ Servo_Wrapper
+ Ultrasonic
+ Ultrasonic_Wrapper
+ Utilities

*There is a [python version](https://gitlab.com/rohand2412/raspberry_pi_libraries) of the Serial_Wrapper class so that a wider range of devices, not limited to C/C++, can communicate amongst each other.

## Getting Started

Your installation instructions may vary depending on the IDE you use to communicate with your microcontroller.

## PlatformIO via VSCode (Recommended)

1. Navigate to the `.platformio` folder in your user directory.
    + **Unix** (includes Linux and macOS, which are Unix-based)**:** `~/.platformio/`
    + **Windows:** `C:\Users\YourUsername\.platformio\`

2. Create a `lib` within the `.platformio` folder if it doesn't already exist.

3. Clone this repository in the `lib` folder.

4. Restart the VSCode IDE and you should be able to include any of the files in this repository.

If VSCode doesn't autocomplete the filenames or detects "include errors", that's fine, it just means that the `.platformio/lib` folder is not in your VSCode include path. PlatformIO will still compile correctly and work as intended.

## Arduino IDE

1. Navigate to the `libraries` folder of your sketchbook. Your sketchbook folder can be found or changed at *File > Preferences > Sketchbook location*.

2. Clone this repository in the `libraries` folder.

3. Restart the Arduino IDE and you should be able to include any of the files in this repository from *Sketch > Include Library*.

For more information on installation, see under *Manual Installation* on the [official Arduino page](http://www.arduino.cc/en/Guide/Libraries).

This repository was developed with PlatformIO so there are some slight discrepancies in how the library will be compiled in the Arduino IDE. For example, the Arduino IDE doesn't allow VLA(variable length array) initializers even when you know the size at compile time. Which is actually a breaking discrepancy. To fix it, on line 57 of Robot.cpp change `_motors->getMotorNum() * Encoder_Wrapper::PINS_PER_SENSOR` to `4`.

## Example Code

[Here](https://gitlab.com/rohand2412/arduino-due-hardware-interfaces) is a collection of PlatformIO projects that are test cases for the classes of this repository and serve as example code as well. For Arduino IDE users, just look at `src/main.cpp` under any of the projects and don't mind the `#include <Arduino.h>`, that just includes all of the Arduino functionality from PlatformIO.

## Resources

Below is a list of repositories whose files were used in this repository. They are extremely informative and it is recommended that you check them out. Their files are only kept here for portability and as a dependency requirement.

+ [Adafruit Unified BNO055 Driver (AHRS/Orientation)](https://github.com/adafruit/Adafruit_BNO055)
    + Adafruit_BNO055.cpp
    + Adafruit_BNO055.h
    + *Under the* `utility` *folder in the repository*
    + imumaths.h
    + matrix.h
    + quaternion.h
    + vector.h
+ [Adafruit Motor Shield V2 Library](https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library)
    + Adafruit_MotorShield.cpp
    + Adafruit_MotorShield.h
    + *Under the* `utility` *folder in the repository*
    + Adafruit_MS_PWMServoDriver.cpp
    + Adafruit_MS_PWMServoDriver.h
+ [Adafruit Unified Sensor Driver](https://github.com/adafruit/Adafruit_Sensor)
    + Adafruit_Sensor.cpp
    + Adafruit_Sensor.h
+ [ArduinoTrace](https://github.com/bblanchon/ArduinoTrace)*
    + ArduinoTrace.h
+ [Encoder Library](https://github.com/PaulStoffregen/Encoder)
    + Encoder.cpp
    + Encoder.h
    + *Under the* `utility` *folder in the repository*
    + direct_pin_read.h
    + interrupt_config.h
    + interrupt_pins.h
+ [Arduino PID Library](https://github.com/br3ttb/Arduino-PID-Library)
    + PID_v1.cpp
    + PID_v1.h
+ [Servo Library for Arduino](https://github.com/arduino-libraries/Servo)
    + *Under the* `src` *folder in the repository*
    + Servo.h
    + *Under the* `src/sam` *folder in the repository***
    + Servo.cpp
    + ServoTimers.h

*This repository does not depend on `ArduinoTrace.h` it is purely included because it is an extremely helpful debugging tool that everyone should use.

**Under `src` there are folders for each family of CPUs. The [Arduino Due](https://docs.arduino.cc/hardware/due) has an Atmel SAM3X8E ARM Cortex-M3 CPU from the Atmel SAM CPU family, and thus the files under `src/sam` were used. The folder with the files you need may vary depending on the CPU of your microcontroller.