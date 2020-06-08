Evaluation Boards/Products Supported
------------------------------------ 
EVAL-AD7124-8SZ

Overview
--------
These code files provide the firmware application and device libraries to interface various 
Sigma Delta ADCs to capture the temperature data and dispatch over either UART or Bluetooth
link. This code was developed and tested on EV-COG-AD3029LZ controller board: 
https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/EV-COG-AD3029.html


Hardware Setup
--------------
Required: EV-COG-AD3029LZ Controller Board, EV-COG-BLEINTP1Z Bluetooth Board, EV-GEAR-EXPANDER1Z Expander Board and 
EVAL-AD7124-8SZ Board.
USB cable to connet ADuCM3029 COG board to PC.
Refere below link for complete hardware setup: 
https://wiki.analog.com/resources/tools-software/product-support-software/sigma-delta_adc_temperature-ble_demo


How to Get Started
------------------
-Open CCES (Cross Core Embedded Studio) IDE and open existing "sd-adc_cces_temperature-ble" and "tempsensors" projects into it. 
-Build the "sd-adc_cces_temperature-ble" project to generate binary (.hex) file, copied into Debug/Release folder of project directory.
-Setup the hardware as specified in the project page link from "Hardware Setup" section above.
-Connect ADuCM3029 COG EVAL board to PC via USB cable and manually copy the .hex file into CMSIS-DAP drive mounted on PC.
-Press the Reset button.
-Open the Serial Monitor tool and/or IoT Node Smart IOS App to capture the sensor data sent from ADuCM3029 over UART/BLE link.


Copyright (c) 2020 Analog Devices, Inc.  All rights reserved.
