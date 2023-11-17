## <b>DataLogExtended Application Description for STEVAL-MKSBOX1V1 (SensorTile.box) evaluation board</b>

This firmware package includes Components Device Drivers, Board Support Package and example application for the following STMicroelectronics elements:

 - STEVAL-MKSBOX1V1 (SensorTile.box) evaluation board that contains the following components:
	- MEMS sensor devices: HTS221, LPS22HH, STTS751, LIS2MDL, LIS2DW12, LIS3DHH, LSM6DSOX
	- analog microphone: MP23ABS1

### <b>Example Description</b> 
Main function is to show how to use SensorTile.box to send data using USB to a connected PC or Desktop and display it on specific application Unicleo-GUI V1.19.0,
which is developed by STMicroelectronics and provided separately, not in binary with this package.
After connection has been established:

- the user can view the data from various on-board environment sensors like Temperature, Humidity, and Pressure.
- the user can also view data from various on-board MEMS sensors as well like Accelerometer, Gyroscope, and Magnetometer.
- the user can also visualize this data as graphs using Unicleo-GUI application.

The Unicleo-GUI could be downloaded from:
https://www.st.com/content/st_com/en/products/embedded-software/evaluation-tool-software/unicleo-gui.html

### <b>Dependencies</b>

STM32Cube packages:

  - STM32L4xx drivers from STM32CubeL4 V1.18.0
  
STEVAL-MKSBOX1V1:

  - STEVAL-MKSBOX1V1 V1.3.6

### <b>Hardware and Software environment</b>

  - This example runs on SensorTile.box device.
  - This example has been tested with SensorTile.box.

### <b>How to use it?</b>

This package contains projects for 3 IDEs viz- IAR, Keil µVision 5 and Integrated Development Environment for STM32.
In order to make the  program work, you must do the following:

 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.

For IAR:

 - Open IAR toolchain (this firmware has been successfully tested with Embedded Workbench V9.20.1).
 - Open the IAR project file EWARM\Project.eww.
 - Rebuild all files and load your image into target memory.
 - Run the example.

For Keil µVision 5:

 - Open Keil µVision 5 toolchain (this firmware has been successfully tested with MDK-ARM Professional Version: 5.37.0).
 - Open the µVision project file MDK-ARM\Project.uvprojx.
 - Rebuild all files and load your image into target memory.
 - Run the example.

For Integrated Development Environment for STM32:

 - Open STM32CubeIDE (this firmware has been successfully tested with Version 1.13.2)
 - Set the default workspace proposed by the IDE (please be sure that there are not spaces in the workspace path).
 - Press "File" -> "Import" -> "Existing Projects into Workspace"; press "Browse" in the "Select root directory" and choose the path where the STM32CubeIDE project is located (it should be STM32CubeIDE\). 
 - Rebuild all files and load your image into target memory.
 - Run the example.
 
### <b>Author</b>

SRA Application Team

### <b>License</b>

Copyright (c) 2023 STMicroelectronics.
All rights reserved.

This software is licensed under terms that can be found in the LICENSE file
in the root directory of this software component.
If no LICENSE file comes with this software, it is provided AS-IS.
   
