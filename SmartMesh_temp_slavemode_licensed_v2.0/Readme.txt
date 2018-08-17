            Analog Devices, Inc. ADuCM302x Application Example

Project Name: SmartMesh_RF_cog_temp_example

Description:  Demonstrates how to obtain temperature data or random values(based on a macro defined in the SmartMesh_RF_cog_temp_example.c") using EV-COG-SMARTMESH1Z. 
              In this example obtains temperature data from the temperature sensor present in the EV-COG-ADuCM3029 or a random value is generated and then transmitted to the E-manager via EV-COG-SMARTMESH1Z. 
 

Overview:
=========
    This example connects the EV-COG-SMARTMESH1Z to a network setup by the E-manager connected to the PC and sends either temperature data or dummy data(based on user
    selection), it waits for an arbitaray time interval before it sends the next set of data.Basic APIs are necessary for configuring the EV-COG-SMARTMESH1Z and joining
    the network and sending data packets. 


User Configuration Macros:
==========================
#define Sensor_data_enable        1   for temperature data
#define Sensor_data_enable        0   for dummy data

Hardware Setup:
===============
   ADuCM3029:- Change the UART headers to redirect MCU UART to ADI_RF Uart lines. 
   
   EV-Cog-SmartMesh1z should be connected to the  ADuCM3029 via the hirose connector.
   Switch 'SW1' must be in the default position( default position is away from the notch) 
   
External connections:
=====================
    None. 

Software Setup:
===============
   Compatible with BSP v2.0 of ADuCM3029.

   Use IAR v7.7 or higher.

   In order to send random data the user can change Sensor_data_enble to 0 and 1 for sending temperature values.

   User has to comment out the code for "Disable Rx buffer interrupts for PIO mode if the FIFO is not enabled" in adi_uart.c (this set is a must for this software version)

    
How to build and run:
=====================

    Prepare hardware as explained in the Hardware Setup section.
							
    Download source code ,unzip it inside the BSP examples folder of ADuCM3029
	(typically: C:\Analog Devices\ADuCM302x\ADuCM302x_EZ_Kit\examples).

    The SmartMesh C Library can be downloaded at https://github.com/dustcloud/sm_clib. Download the C-Library and place it in the folder where one has unziped the source code.

    Import the project into IAR

    Build the project and download the code to EV-COG-AD3029.

    Before running the project see to it E-manager is connected to the PC. 


Expected Result:
=====================
    Upon successful completion the program should output:
    
    Please refer the output verification methods as mentioned by WIKI page for EV-Cog-SMARTMESH1Z.
     
    Link:- https://wiki.analog.com/resources/eval/user-guides/ev-cog-ad3029lz/example_project/temp_sensor_smartmesh
        
References:
===========
    ADuCM302x Hardware Reference Manual
    Refer https://wiki.analog.com/resources/eval/user-guides/ev-cog-smartmesh1z.
    Refer https://wiki.analog.com/resources/eval/user-guides/ev-cog-ad3029lz/example_project/temp_sensor_smartmesh.   
