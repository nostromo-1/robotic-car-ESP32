# robotic-car-ESP32
Project of a robotic car with an ESP32 as control unit.

![](/images/IMG_20241207_122717.jpg)

This project is about building a hobby robotic car.
It is a 4WD or 2WD car, with a ESP32 as MCU, and controlled by a wiimote.
It needs some external electronics.
Communication with the robot is achieved via bluetooth (the wiimote).

## Features
* 4WD or 2WD robotic car (depending on chasis)
* It can operate under the control of a Wiimote or in autonomous mode
* It can play wav files over the loudspeaker (it needs only an analogue audio amplifier, as it uses the internal DAC)
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
* It communicates via wifi, for example it reads current time and date from an NTP server. Initial wifi configuration is via WPS
* It updates itself over the air when starting (firmware [OTA update](https://github.com/nostromo-1/robotic-car-ESP32/blob/master/OTA%20update.md) from github). This means that a fleet of robot cars will be updated automatically upon boot!

## Parts
The following parts are needed to build it (see the schematics folder):
* Car chasis. For example, https://leantec.es/tienda/chasis-robot-4wd-chasis-robot-de-4-ruedas/ or https://leantec.es/tienda/chassis-2wd-smart-car-chassis-arduino-robot/
* 6V DC motors. If 4WD: 4 motors. If 2WD, then 2 motors (in that case, I use motors with a wheel Hall encoder, [DFRobot FIT0450](https://www.dfrobot.com/product-1457.html), in order to make the car run in a straight line using a PID control loop)
* Motor controller: a L298N based circuit board, like https://www.electrohobby.es/driver-dc/209-driver-motor-l298n.html
* Ultrasonic distance sensor HC-SR04
* Display module SSD1306
* An inertial module unit based on LSM9DS1 (controlled via I2C), like https://learn.adafruit.com/adafruit-lsm9ds1-accelerometer-plus-gyro-plus-magnetometer-9-dof-breakout
* An ESP32 development board, like [this](https://www.mouser.es/ProductDetail/Espressif-Systems/ESP32-DevKitC-32E?qs=GedFDFLaBXFpgD0kAZWDrQ%3D%3D). It needs Bluetooth classic, so it must be the ESP32 (and not one of the variants). The WROOM module is enough, with 4M flash and no PSRAM.
* Transistors, capacitors, resistors, a 6V buzzer, and a push button (this will be used to communicate with the car, see Operation)
* For the audio amplifier: a LM386 integrated circuit, an 8 ohm small speaker and some resistors and capacitors. See the schematics. The audio signal is taken from the GPIO using the internal DAC, so an analogue amplifier is enough. Alternatively, you can use another amplifier like one based on [TPA2005D1](https://www.sparkfun.com/products/11044).
* For the battery status monitor: a PCF8591.
* Power supply: two 18650 type batteries in series, protected. I use 2600 mAh Nitecore. The 5V supply for the ESP32 board comes from a switching regulator. I use the [S7V7F5](https://www.pololu.com/product/2119). Alternatively, you can use 6 NiMH AA batteries.

## Software
The robot is programmed in C using the Espressif environment (not Arduino). It is tested on the Espressif v5.3 version. Please refer to the [installation instructions](https://docs.espressif.com/projects/esp-idf/en/v5.3.2/esp32/get-started/index.html).

After installing the environment and copying the source files, run `idf.py build` followed by `idf.py flash`.

## Operation
Switch on power on the car. The startup sequence begins:
* Initialize display and light it, as life sign (the display shows all text inverted, i.e., black on blue)
* Show the project name and firmware version in the first 2 lines of the display
* Initialize power check system (voltage and current monitoring). If voltage is too low (batteries are dying) it will abort start
* Start wifi connection
  * If no credentials are stored in NVRAM (this is your first run) or the push button is pressed, it will run a WPS initialization: it will display `WPS` and `Press WPS button in wifi router` in the display. If you want wifi, then press the WPS button on your router within 30 seconds
  * Establish a wifi connection with the router and store the credentials in NVRAM, so next time you do not need to use WPS
  * If the wifi connection is successful:
    * write the IP address and the router SSID on the display
    * get time and date from an NTP server
    * if the battery is good enough, check if there is a new firmware version in github; if so, download and install it, and reboot
    * start a small web server, used to retrieve configuration files to a host (to examine them)
* Start bluetooth connection with wiimote
  * The text `Scan...` will appear on the display. Then, press buttons '1' and '2' on the wiimote. The car will find the wiimote and attach to it.
  * The wiimote will vibrate as acknowledment of the connection, and its leds will be lit according to the selected car speed
  * A bluetooth symbol will be displayed on the top right corner of the display
* Start IMU calibration sequence
  * The text `CALIB?` will appear on the display. If you want to calibrate the IMU, you can now press the push button until the text disappears. This step is mandatory in the first run of the car; if you do not do it, the car will display `PLEASE CALIB ME` and abort
  * If you chose to calibrate, it starts the sequence:
    * Accelerometer and gyroscope calibration: The text `HORIZ. WAIT...` will appear on the display. Leave the car horizontal and quiet for 10 seconds
    * Magnetometer error deviation estimation: The text `HORIZ. WAIT...` stays on the display. Leave the car horizontal and quiet for 8 more seconds
    * Magnetometer calibration: The text `ROTATE CAR...` will appear on the display, and the buzzer will briefly piep. Slowly turn around the car in all directions, over all 3 axis, for 30 seconds, until the buzzer pieps again
* Other components will be initialized, and the display will leave inversion state
* It checks the battery status. If it is too low, it will read a text over the loudspeaker ("Help help, my battery is low and it is getting dark"). Otherwise, it will say "I am ready for operation"
* It displays `Ready` and enters normal operation, waiting for user interaction via the wiimote
* It displays the distance to an obstacle in front of it in the first row of the display, and the voltage and current consumption in the second row

Now, you can control the car: press 'A' to move forward. Battery status will be permanently monitored: if it is too low, it will abort.

This video shows the startup sequence:

https://github.com/user-attachments/assets/9085112a-0983-4f0e-8d11-f1e8eb150cce



