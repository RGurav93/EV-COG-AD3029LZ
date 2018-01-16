SDHC card interface code for EVCOG 3029 and EVCOG 3029 (expander gear required for both).

* ADuCM3029 compat code (BSP v2.0)
* ADuCM4050 compat code (BSP V2.0)
* this program creates a text file named "log.txt" in the sd card and logs the temperature with time 
stamp on to that file.
* FATFS source code included
* Refer http://elm-chan.org/fsw/ff/00index_e.html for explanation on fatfs api's.


How to use:
							/**********ADuCM3029************/
							
1. Download source code ,unzip it inside the BSP examples folder of ADuCM3029
	(typically: C:\Analog Devices\ADuCM302x\ADuCM302x_EZ_Kit\examples).
2. Import the project into IAR
3. For EVCOG 3029 with expander gear the spi configuration in sdcard_spi_hal.h for sd card slot is as follows : (already modified in the source)
  	SPI_DEV_NUM -> 0, 
	SPI_CS_PORT -> 0, 
	SPI_CS_PIN  -> GPIO Port 2 pin 8  (the cs is set manually due to fatfs library requirements).  
4. Insert a SDHC card in the sd card slot of the expander gear.
5. Build the project and download to EV-COG.
6. Run it for some time and the unplug the SDHC card and now you can view the log.txt created in 
	the card via a pc or mobile.

							/**********ADuCM4050************/
							
1. Download source code ,unzip it inside the BSP examples folder of ADuCM4050
	(typically: C:\Analog Devices\ADuCM4x50\ADuCM4x50_EZ_Kit\examples).
2. Import the project into IAR
3. Remove startup_ADuCM3029.s and system_ADuCM3029.c
4. Add C:\Analog Devices\ADuCM4x50\ADuCM4x50_EZ_Kit\Source\system_ADuCM4050.c and
   C:\Analog Devices\ADuCM4x50\ADuCM4x50_EZ_Kit\Source\IAR\startup_ADuCM4050.s
   do the same for common.c
5. Goto project-> options-> general -> and change the device to ADuCM4050
6. In the project-> options-> c/c++ compiler -> preprocessor replace __ADUCM3029__ with __ADUCM4050__
7. In the project-> options-> assembler -> preprocessor -> additional directories clear all entries and
	paste  the following path $PROJ_DIR$\..\..\Include and  $PROJ_DIR$\..\..\Include\config
8. For EVCOG 4050 with expander gear the spi configuration in sdcard_spi_hal.h for sd card slot is as follows :
 (already modified in the source)
  	SPI_DEV_NUM -> 0, 
	SPI_CS_PORT -> 0, 
	SPI_CS_PIN  -> GPIO Port 2 pin 8  (the cs is set manually due to fatfs library requirements).  
9. Insert a SDHC card in the sd card slot of the expander gear.
10. Build the project and download to EV-COG 4050.
11. Run it for some time and the unplug the SDHC card and now you can view the log.txt created in 
	the card via a pc or mobile.
	
Notes:

* Modify sdcard_spi_hal.c if porting to another system. 
All sdcard use cases just uses the functions inside this file.

