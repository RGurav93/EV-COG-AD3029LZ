/*!
 *****************************************************************************
  @file:  btns_leds.h

  @brief: Interface for AD3029 COG board buttons and LEDs

  @details:
 -----------------------------------------------------------------------------
 Copyright (c) 2018, 2020 Analog Devices, Inc.  All rights reserved.

 This software is proprietary to Analog Devices, Inc. and its licensors.
 By using this software you agree to the terms of the associated
 Analog Devices Software License Agreement.

*****************************************************************************/

#ifndef BTNS_LEDS_H_
#define BTNS_LEDS_H_

#ifdef __EVCOG__

#define PB1_PORT_NUM        ADI_GPIO_PORT1
#define PB1_PIN_NUM         ADI_GPIO_PIN_0


#define PB2_PORT_NUM        ADI_GPIO_PORT0
#define PB2_PIN_NUM         ADI_GPIO_PIN_9


#define LED1_PORT_NUM       ADI_GPIO_PORT2
#define LED1_PIN_NUM        ADI_GPIO_PIN_2


#define LED2_PORT_NUM       ADI_GPIO_PORT2
#define LED2_PIN_NUM        ADI_GPIO_PIN_10


#define PB1_LABEL           "BTN1"
#define PB2_LABEL           "BTN2"

#define LED1_LABEL          "LED1"
#define LED2_LABEL          "LED2"

#else
#error the test is not ported to this processor
#endif

#endif /* BTNS_LEDS_H_ */
