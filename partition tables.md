# Partition tables

The used ESP32 has a 4MB flash. This memory can be divided into partitions, which can contain the main program as well as additional data, such as data on NVRAM or a file system.
In this project, a custom partition table is used, and this option is selected in the provided `sdkconfig`file.
The partition table is defined in the file `partitions.csv`, and its syntax is explained in the Espressif [documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/partition-tables.html).

The following table is used:

