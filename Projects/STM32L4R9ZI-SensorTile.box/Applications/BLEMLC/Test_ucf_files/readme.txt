/**
  ******************** (C) COPYRIGHT STMicroelectronics ***********************
  * @file    readme.txt
  * @author  Central LAB
  * @version V7.1.0
  * @date    18-Apr-2020
  * @brief   Description of the ucf test files for MLC and FSM
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0055, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0055
  *
  ******************************************************************************
  */
This directory contains  2 .ucf files:

- z_MLC_LSM6DSOX_inclinometer_DT1to8.ucf test program for LSM6DSOX Machine Learning 
  Core that implements one inclinometer that produces output in any one of the 8 MLC 
  possible outputs

- z_FSM_LSM6DSOX_inclinometer_FSM1to16.ucf test program for LSM6DSOX Finite State Machine 
  that implements one  inclinometer that produces output in any one of the 16 FSM possible outputs.
  Starting from a horizontal position it's possible to see all the FSM outputs change their
  value from 0x0 to 0x80 changing the board inclination


For using these files:
1) copy them on your smartphone (Android/iOS)
2) open the "ST BLE Sensor application" 
3) go on "Create a new App" -> "Expert View"
4) select like input FSM or MLC
5) select like input program the corresponding ufc file for the input that you have selected:
  - z_FSM_LSM6DSOX_inclinometer_FSM1to16.ucf for input FSM
  - z_MLC_LSM6DSOX_inclinometer_DT1to8.ucf for input MLC
6) select "Save Config"
7) select like output "stream to usb"
8) save the created application providing one name for it




For Android smartphone there are also two json files that could be used for avoid to make all the
above steps for creating a new application:

- mlc.json: created starting from z_MLC_LSM6DSOX_inclinometer_DT1to8.ucf program
- fsm.json: created starting from  z_FSM_LSM6DSOX_inclinometer_FSM1to16.ucf program

These files must be placed on Documents/SensorTile.box directory for Android smartphones
and they will be visible under "Create a new App" -> "Expert View" on ST BLE sensor application

 /******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
