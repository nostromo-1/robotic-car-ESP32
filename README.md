# robotic-car-ESP32
Project of a robotic car with an ESP32 as control unit. It is equivalent to the Raspberry Pi version.

This project is about building a hobby robotic car.
It is a 4WD or 2WD car, with a ESP32 as MCU, and controlled by a wiimote.
It needs some external electronics.
Communication with the robot is achieved via bluetooth (the wiimote).

## Features
* 4WD or 2WD robotic car (depending on chasis)
* It can operate under the control of a Wiimote or in autonomous mode
* The Wiimote can be used to:
  * Move forward (A button) or backward (B button)
  * Turn right (RIGHT button) or left (LEFT button)
  * Increase ('+' button) or decrease speed ('-' button). Speed is signalled in the leds of the Wiimote
  * Activate a buzzer sound (DOWN button)
  * Play a police siren (UP button) over a loudspeaker. If you push again the UP button while still playing, it stops playing
* It can detect collisions via the inertial measurement unit. If it crashes, it tries to avoid the obstacle. NOT YET IMPLEMENTED
* It continuously monitors distance to an obstacle in the front side. If an obstacle is near, it will drive around it by turning until no obstacle is found. If it detects a stall (like in undetected obstacles, due to a non straight position with respect to the sonar), it will move a little backwards and turn to avoid it
* It monitors battery voltage and current consumption and shows them in the display, also showing a battery status symbol in the display. If battery is too low, it powers off
* If the scan button is pressed, it starts scanning for wiimotes and connects to one.
* It displays status messages in a display. It also shows its orientation (roll, pitch and yaw angles).
* It features a KARR-type scanner :-)

## Parts
The following parts are needed to build it:
* Car chasis. For example, https://leantec.es/tienda/chasis-robot-4wd-chasis-robot-de-4-ruedas/ or https://leantec.es/tienda/chassis-2wd-smart-car-chassis-arduino-robot/
* 6V DC motors. If 4WD: 4 motors. If 2WD, then 2 motors (in that case, I use motors with a wheel Hall encoder, [DFRobot FIT0450](https://www.dfrobot.com/product-1457.html), in order to make the car run in a straight line using a PID control loop)
* Motor controller: a L298N based circuit board, like https://www.electrohobby.es/driver-dc/209-driver-motor-l298n.html
* Ultrasonic distance sensor HC-SR04
* Display module SSD1306
* An inertial module unit based on LSM9DS1 (controlled via I2C), like https://learn.adafruit.com/adafruit-lsm9ds1-accelerometer-plus-gyro-plus-magnetometer-9-dof-breakout
* An ESP32 development board, like [this](https://www.mouser.es/ProductDetail/Espressif-Systems/ESP32-DevKitC-32E?qs=GedFDFLaBXFpgD0kAZWDrQ%3D%3D). It needs Bluetooth classic, so it must be the ESP32 (and not one of the variants). The WROOM module is enough, with 4M flash and no PSRAM.
* Transistors, capacitors, resistors, and scan button. A 6V buzzer. See the schematics directory.
* For the audio amplifier: a LM386 integrated circuit, an 8 ohm small speaker and some resistors and capacitors. See the schematics. The audio signal is taken from the GPIO using the internal DAC, so an analogue amplifier is enough. Alternatively, you can use another amplifier like one based on [TPA2005D1](https://www.sparkfun.com/products/11044).
* For the battery status monitor: a PCF8591.
* Power supply: two 18650 type batteries in series, protected. I use 2600 mAh Nitecore. The 5V supply for the Pi comes from a switching regulator. I use the [S7V7F5](https://www.pololu.com/product/2119). Alternatively, you can use 6 NiMH AA batteries.

## Software
The robot is programmed in C using the Espressif environment (not Arduino). It is tested on the Espressif v5.1 version. Please refer to the [installation instructions](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/).

After installing the environment and copying the source files, run `idf.py build` followed by `idf.py flash`.

## Operation
Switch on power on the car. The text "Scan..." will appear on the display. Then, press buttons '1' and '2' on the wiimote. The car will find the wiimote and attach to it. 

Now, you can control the car: press 'A' to move forward.


