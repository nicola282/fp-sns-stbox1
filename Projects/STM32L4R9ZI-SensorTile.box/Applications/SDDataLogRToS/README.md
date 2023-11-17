## <b>DataLogExtended Application Description for STEVAL-MKSBOX1V1 (SensorTile.box) evaluation board</b>

This firmware package includes Components Device Drivers, Board Support Package and example application for the following STMicroelectronics elements:

 - STEVAL-MKSBOX1V1 (SensorTile.box) evaluation board that contains the following components:
	- MEMS sensor devices: HTS221, LPS22HH, STTS751, LIS2MDL, LIS2DW12, LIS3DHH, LSM6DSOX
	- analog microphone: MP23ABS1
	
 - FatFs generic FAT file system module provides access the storage devices such as memory card and hard disk.
 - FreeRTOS Real Time Kernel/Scheduler that allows applications to be organized as a collection of independent threads of execution (under MIT open source license)

### <b>Example Description</b> 
 The Example application initializes all the Components and pressing the User button is possible to 
 start/stop the recording off all board's sensors to SD-card.
 The program save 3 different files on SD-card for each log:
 
 - Sens000.csv where it stores the values of Acc/Gyro/Mag/Pressure/Temperature/Humidity
 - Mic000.wav where it stores the wave file for Analog Microphone at 16Khz
 - Rep000.txt where it stores the summary of used FreeRTOS queues and Max time for writing the Audio Buffer to the .wav file:
 
   - Pool Queue:
     - Max Size  = XXX
     - Released  = XXX
     - Allocated = XXX

   - Message Queue:
     - Max Size  = XXX
     - Released  = XXX
     - Allocated = XXX
	 
  Max time for writing XXXXbytes for Audio =XXX mSec

### <b>Dependencies</b>

STM32Cube packages:

  - STM32L4xx drivers from STM32CubeL4 V1.18.0
  
STEVAL-MKSBOX1V1:

  - STEVAL-MKSBOX1V1 V1.3.6

### <b>Hardware and Software environment</b>

  - This example runs on STEVAL-MKSBOX1V1 (SensorTile.box) evaluation board and it can be easily tailored to any other supported device and development board.

### <b>How to use it?</b>

This package contains projects for 3 IDEs viz- IAR, Keil µVision 5 and Integrated Development Environment for STM32.
In order to make the  program work, you must do the following:

 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.

For IAR:

 - Open IAR toolchain (this firmware has been successfully tested with Embedded Workbench V9.20.1).
 - Open the IAR project file EWARM\DataLog.eww
 - Rebuild all files and Flash the binary on STEVAL-MKSBOX1V1

For Keil µVision 5:

 - Open Keil µVision 5 toolchain (this firmware has been successfully tested with MDK-ARM Professional Version: 5.37.0).
 - Open the µVision project file MDK-ARM\Project.uvprojx
 - Rebuild all files and Flash the binary on STEVAL-MKSBOX1V1
		
For Integrated Development Environment for STM32:

 - Open STM32CubeIDE (this firmware has been successfully tested with Version 1.13.2)
 - Set the default workspace proposed by the IDE (please be sure that there are not spaces in the workspace path).
 - Press "File" -> "Import" -> "Existing Projects into Workspace"; press "Browse" in the "Select root directory" and choose the path where the STM32CubeIDE project is located (it should be STM32CubeIDE\). 
 - Rebuild all files and Flash the binary on STEVAL-MKSBOX1V1
   
### <b>Author</b>

SRA Application Team

### <b>License</b>

Copyright (c) 2023 STMicroelectronics.
All rights reserved.

This software is licensed under terms that can be found in the LICENSE file
in the root directory of this software component.
If no LICENSE file comes with this software, it is provided AS-IS.