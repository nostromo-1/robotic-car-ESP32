# OTA update

Upon start, if a connection to a wifi station can be established (initial wifi configuiration is done via WPS, credentials are stored in NVS for later reboots),
and if there is enough battery level, a new firmware will be automatically retrieved from the github repository.

The car will contact github, and check the firmware version available in the repository. If it is a newer version than the active version, 
it will download the new version and reboot. The procedure needs a special [partition table](https://github.com/nostromo-1/robotic-car-ESP32/blob/master/partition%20tables.md) with 2 firmware slots, for the OTA. Only one of them will be active. 
The OTA software will handle this automatically. This avoids bricking the car in case of bad or incomplete firmware downloads.

The procedure is presented in the car display as follows:

https://github.com/user-attachments/assets/969e3dbb-aede-421e-8186-af46ef93e779

