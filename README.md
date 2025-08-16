# BMP580-Pressure-Sensor-Test
by JS106351

## Description of Project

This is a simple project in which I test a Bosch **BMP580** pressure sensor that I got off of Ebay with an ST Microelectronics STM32 NUCLEO dev board using Bosch's *BMP5_SensorAPI* API library. The code used here is based on one of the example projects that Bosch provides with the library (*read_sensor_data_normal_mode*). The sensor is connected via **I2C** to the MCU.  SPI/I3C has not been tested or verified for use.  Data from the sensor is processed and sent via UART serial to the computer.  A terminal program is used (*i.e. Procomm Plus*) to display the data.  In this case I set the terminal mode to VT100 and baud rate to 115200.  

Some parts of the BMP5 library had to be modified to work for my use case.  the common.c file was altered to work with the STM32 enviroment.  Other parts relating to functionaly of the code had to be changed.  During initilisation of the sensor when it tries to execute *power_up_check(dev)*, the result always ends up **BMP5_E_POWER_UP**.  Bypassing and ignoring this check by commenting out the code did not affect the operation of the sensor, and the sensor continued to work normally.  Some parts of code relating to the COINES platform have been removed or commented out.  There is some code relating to SPI has been modified but not tested and left commented out as I2C was used only.       

## Hardware/Software Info

Project was developed in STM32CubeIDE ver. 1.5.1 using HAL.  The dev board used is a STMicroelectronics **NUCLEO-G431KB** which can be found on DigiKey/Mouser etc. for cheap.
The sensor can be found on Ebay under **I2C/SPI BMP580 Temperature Barometric Pressure Sensor Module For Arduino LGA-10**
The API from Bosch can be found here: https://github.com/boschsensortec/BMP5_SensorAPI
Terminal software used is Procomm Plus Ver. 4.8
This code is provided as is.  


