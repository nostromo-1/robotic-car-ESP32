# Partition tables

The used ESP32 has a 4MB flash. This memory can be divided into partitions, which can contain the main program as well as additional data, such as data on NVRAM or a file system.
In this project, a custom partition table is used, and this option is selected in the provided `sdkconfig`file.
The partition table is defined in the file `partitions.csv`, and its syntax is explained in the Espressif [documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/partition-tables.html).

The following table is used:
```
# Name,   Type, SubType, Offset,  Size, Flags
# Note: if you have increased the bootloader size, make sure to update the offsets to avoid overlap
nvs,      data, nvs,     ,        0x4000,
otadata,  data, ota,     ,        0x2000
phy_init, data, phy,     ,        0x1000,
app0,     app, ota_0,    ,        1728K,
app1,     app, ota_1,    ,        1728K,
storage,  data, spiffs,  ,        576K,
```

The sections are as follows:
* nvs: the Non-Volatile Storage (for example, used for the wifi credentials)
* otadata: the OTA data partition which stores information about the currently selected OTA app slot; this is used for downloading firmware from github automatically
* phy_init: for storing PHY initialisation data
* app0: two slots for the firmware, only one is active
* app1: two slots for the firmware, only one is active
* storage: a [spiffs](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/storage/spiffs.html) filesystem (used to store sound files and the IMU calibration results)

The spiffs filesystem is 576KB in size, only about 450KB are in use, but there must be spare capacity for the filesystem to work properly.
The app partitions are 1728KB each, which is the maximum size available in a 4MB flash. The program size is currently 1300KB.



