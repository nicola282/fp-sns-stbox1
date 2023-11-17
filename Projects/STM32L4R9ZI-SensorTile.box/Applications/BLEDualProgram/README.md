## <b>BLEDualProgram Application Description for STEVAL-MKSBOX1V1 (SensorTile.box) evaluation board</b>

This firmware package includes Components Device Drivers, Board Support Package and example application for the following STMicroelectronics elements:

 - STEVAL-MKSBOX1V1 (SensorTile.box) evaluation board that contains the following components:
	- MEMS sensor devices: HTS221, LPS22HH, STTS751, LIS2MDL, LIS2DW12, LIS3DHH, LSM6DSOX
	- analog microphone: MP23ABS1
 
The Example application initializes all the Components and Library creating some Custom Bluetooth services:

  - The first service exposes all the HW characteristics:
     - Random mems sensor fusion
     - Battery status
	 
   - The second Service exposes the console services where we have stdin/stdout and stderr capabilities
   - The last Service is used for configuration purpose
 
The Example application provides one example on how to perform a firmware over the air update using Bluetooth and select the ROM Bank to use.
For increasing the security, for making the connection is necessary to use a pin: 123456 for default that could be changed 
after the connection
The firwmare over the air update allows also to change the Firwmare running on the board

### <b>Dependencies</b>

STM32Cube packages:

  - STM32L4xx drivers from STM32CubeL4 V1.18.0
  
STEVAL-MKSBOX1V1:

  - STEVAL-MKSBOX1V1 V1.3.6

### <b>Hardware and Software environment</b>

- This example runs on STEVAL-MKSBOX1V1 (SensorTile.box) evaluation board and it can be easily tailored to any other supported device and development board.
- This example must be used with the related ST BLE Sensor Android/iOS application (Version 5.0.0 or higher) available on the Google Play or Apple App Store, in order to read the sent information by Bluetooth Low Energy protocol

### <b>How to use it?</b>

This package contains projects for 3 IDEs viz- IAR, Keil µVision 5 and Integrated Development Environment for STM32.
In order to make the  program work, you must do the following:

 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.

For IAR:

 - Open IAR toolchain (this firmware has been successfully tested with Embedded Workbench V9.20.1).
 - Open the IAR project file EWARM\BLEDualProgram.eww
 - Rebuild all files and Flash the binary on STEVAL-MKSBOX1V1

For Keil µVision 5:

 - Open Keil µVision 5 toolchain (this firmware has been successfully tested with MDK-ARM Professional Version: 5.37.0).
 - Open the µVision project file MDK-ARM\Project.uvprojx
 - Rebuild all files and Flash the binary on STEVAL-MKSBOX1V1
 
For Integrated Development Environment for STM32:

 - Open STM32CubeIDE (this firmware has been successfully tested with Version 1.13.2)
 - Set the default workspace proposed by the IDE (please be sure that there are not spaces in the workspace path).
 - Press "File" -> "Import" -> "Existing Projects into Workspace"; press "Browse" in the "Select root directory" and choose the path where the STM32CubeIDE project is located (it should be STM32CubeIDE\).
 - Rebuild all files and and Flash the binary on STEVAL-MKSBOX1V1
 
### <b>Author</b>

SRA Application Team

### <b>License</b>

Copyright (c) 2023 STMicroelectronics.
All rights reserved.

This software is licensed under terms that can be found in the LICENSE file
in the root directory of this software component.
If no LICENSE file comes with this software, it is provided AS-IS.
   
