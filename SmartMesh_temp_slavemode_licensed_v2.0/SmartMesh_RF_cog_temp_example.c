/*! *****************************************************************************
 * @file    SmartMesh_RF_cog.c
 * @brief   Application software for handling data transfer from MCU cog to module(DC9018B-B04,DC9018A-B and DC9021A-05) and vice-versa.
 * @details The APIs in this file can only be called from MCU cog. 
 *
 * @note    This application only facilitates continuos transfer of data from the module to manager of the network.
 -----------------------------------------------------------------------------
Copyright (c) 2017 Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  - Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  - Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  - Modified versions of the software must be conspicuously marked as such.
  - This software is licensed solely and exclusively for use with processors
    manufactured by or for Analog Devices, Inc.
  - This software may not be combined or merged with other code in any manner
    that would cause the software to become subject to terms and conditions
    which differ from those listed here.
  - Neither the name of Analog Devices, Inc. nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.
  - The use of this software may or may not infringe the patent rights of one
    or more patent holders.  This license does not release you from the
    requirement that you obtain separate licenses from these patent holders
    to use this software.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-
INFRINGEMENT, TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF
CLAIMS OF INTELLECTUAL PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

********************************************************************************/

/*=========================== Includes ========================================*/
#include <stdint.h>
#include <stdio.h>
#include "dn_ipmt.h"
#include "dn_uart.h"
#include <drivers/i2c/adi_i2c.h>
#include <drivers/uart/adi_uart.h>
#include "SmartMesh_RF_cog_temp_example.h"

/*=========================== Defines =========================================*/

/* mote state */
#define MOTE_STATE_IDLE           0x01
#define MOTE_STATE_SEARCHING      0x02
#define MOTE_STATE_NEGOCIATING    0x03
#define MOTE_STATE_CONNECTED      0x04
#define MOTE_STATE_OPERATIONAL    0x05

/* Events */
#define JoinStarted               256
#define Operational               32
#define SvcChange                 128

/* service types */
#define SERVICE_TYPE_BW           0x00

/* timings */
#define DATA_PERIOD_S             10                                            /* number of second between data packets */

/* api_ports */
#define SRC_PORT                  0xf0b8
#define DST_PORT                  0xf0b9

/* dummy data */
#define dummy_data                0x00

/*Delay global variable */
#define delay_count_one_sec       26000000
#define delay_count_two_sec       52000000
#define delay_count_ten_sec       260000000

/* Receive Correct */
#define RC_OK                     0x00 

/* Sensor data acquisition */
#define Sensor_data_enable        1

/*=========================== Typedef =========================================*/

typedef void (*timer_callback)(void);
typedef void (*reply_callback)(void);

/*=========================== Variables =======================================*/

typedef struct {
   /* event */
   timer_callback   eventCb;
   /* reply */
   reply_callback   replyCb;
   /* app */
   uint8_t              secUntilTx;
   uint8_t              direction;
   /* api */
   uint8_t              socketId;                          /* ID of the mote's UDP socket */
   uint8_t              replyBuf[MAX_FRAME_LENGTH];        /* holds notifications from ipmt */
   uint8_t              notifBuf[MAX_FRAME_LENGTH];        /* notifications buffer internal to ipmt */
} app_vars_t;

app_vars_t app_vars;

typedef enum {
  Boot_Status,                                            /* Flag_Check value on booting */
  Mote_Status,                                            /* Flag_Check value for getting mote status */
  Open_Socket,                                            /* Flag_Check value for opening a socket for communication */
  Bind_Socket,                                            /* Flag_Check value for binding the socket with a source port */
  Join,                                                   /* Flag_Check value for joining a network */
  SendTo                                                  /* Flag_Check value for sending data to a IPv6 address */
} State_Check;

/*Flag_Check initialization */
State_Check Flag_Check = Boot_Status;

/* Buffer variables */
uint8_t buffer_uart[buffer_size];
int head,tail,next,max_len;

/* Delay Variable */
uint32_t delay_count=0;

/* Transmit clear Flag */
bool transmit_clear=false;

/* control flags */
bool empty,full;

/* I2C variables */
extern uint8_t devMem[ADI_I2C_MEMORY_SIZE];
extern ADI_I2C_HANDLE i2cDevice;
extern uint8_t prologueData[5];
extern ADI_I2C_TRANSACTION xfr;
extern ADI_I2C_RESULT result;
extern uint32_t hwError_i2c;
extern uint8_t rxData[DATASIZE_i2c];

/* Notification Packet for python Tk  */ 
uint8_t sensor_Value_temp[27] ={0x00,0x01,0x05,0x00,0xff,0x01,0x05,0x00,0x00,0x00,0x00,0x3d,0x22,0xdd,0x59,0x00,0x0c,0xea,0x46,0x00,0x00,0x75,0x30,0x01,0x10};
bool  switch_var=false;
/*=========================== Prototypes ======================================*/

/* event */
void scheduleEvent(timer_callback cb);
void cancelEvent(void);
void setCallback(reply_callback cb);

/* ipmt */
void dn_ipmt_notif_cb(uint8_t cmdId, uint8_t subCmdId);
void dn_ipmt_reply_cb(uint8_t cmdId);

/* app */
uint8_t* read_sensor_data(void);
void Smartmesh_RF_cog_receive(void);

/* api */
void api_response_timeout(void);
void api_getMoteStatus(void);
void api_getMoteStatus_reply(void);
void api_openSocket(void);
void api_openSocket_reply(void);
void api_bindSocket(void);
void api_bindSocket_reply(void);
void api_join(void);
void api_join_reply(void);
void api_sendTo(void);
void api_sendTo_reply(void);

/*=========================== Buffer handle ===================================*/
/**
 * @brief    Data written to the cicular buffer is handled here.
 *
 * @param       void
 *
 * @return  void
 *
 * Retrieves the data written to the circular buffer by the interrupt handler and processes the data.
 *
 * @sa      Smartmesh_RF_cog_receive().
 *
 * @note    Buffer handle is called whenever data is written to the circular buffer.
 */

void buffer_handle(void)
{
  while(head!=tail)
  {
   next = tail+1;
  
   if(next>=max_len)
      next = 0;
  
   Smartmesh_RF_cog_receive();
   tail = next;
  }
}

/*=========================== Privates ========================================*/

/**
 * @brief    Schedules events to be executed.
 *
 * @param[in]   cb      Pointer to the function to be scheduled.
 *
 * @return  void.
 *
 * This function currently executes the event that was scheduled.
 *
 * @sa     app_vars.eventCb().
 *
 * @note    This function is provided for implementing time delays using RTC interrupts,currently the application is developed using software delays.
 */

void scheduleEvent(timer_callback cb)
{
   /* function to call */
   app_vars.eventCb = cb;
   app_vars.eventCb();
}

/**
 * @brief    Clears function call.
 *
 * @param       void.
 *
 * @return  void.
 *
 * Clears the function call pointer to NULL.
 *
 * @note    NA.
 */

void cancelEvent(void)
{
   /* clear function to call */
   app_vars.eventCb = NULL;
}

/**
 * @brief    Sets the reply callback function.
 *
 * @param [in]  cb      Pointer to the function to be called upon receiving reply.             
 *
 * @return  void.
 *
 * Function to be called on receving reply from the API is set here.
 *
 * @note    NA.
 */

void setCallback(reply_callback cb)
{
  /* set callback */
  app_vars.replyCb = cb;
}

/*=========================== Impt ============================================*/

/**
 * @brief    Notifications are handled here.
 *
 * @param [in] cmdId      command ID.
 * @param [in] subCmdId   sub command ID.
 *
 * @return  void.
 *
 * All the notifications from the module are handled here,there are 5 types of notifications namely:-
 *  1)Time information.
 *  2)Events.
 *  3)Packet received.
 *  4)transmit done.
 *  5)Received advertisement.
 *
 * 
 * @sa          dn_ipmt_cancelTx()
 *
 *
 * @note    This handles only Event notifications, provisions are given to handle other notifications as well.
 *          For more information on the various fields present in the notifications received refer SmartMesh_IP_Mote_Serial_API_Guide.pdf .
 */

void dn_ipmt_notif_cb(uint8_t cmdId, uint8_t subCmdId)                          /* notification call-back */
{
  dn_ipmt_events_nt* dn_ipmt_events_notif;

  switch (cmdId)
   {
     case CMDID_EVENTS:                                                         /* event notifications */
         /* parse notification */
         dn_ipmt_events_notif = (dn_ipmt_events_nt*)app_vars.notifBuf;
         
         switch (dn_ipmt_events_notif->state)
            {
               case MOTE_STATE_IDLE:
                    Flag_Check=Mote_Status;
                    delay_count=0;
                    transmit_clear=true;
                    dn_ipmt_cancelTx();                                         /* Incase of reset event */
                    break;
               default:
                    /* nothing done here */
                    break;    
            }
         
         switch (dn_ipmt_events_notif->events)
            {
               case JoinStarted:
                    delay_count=0;
                    break;
               case Operational:
                    delay_count=0;
                    break;
               case SvcChange:
                    transmit_clear=true;
                    delay_count=0;
                    Flag_Check=SendTo;
                    break;
               default:
                    /* nothing done here */
                    break;
            }
          
     case CMDID_TIMEINDICATION:                                                 /* Time Indication notifications */
          /* Time Information notifications are handled here */
          break;
     case CMDID_RECEIVE:                                                        /* Packet Received notifications */
          /* Packet Received notifications are handled here */
          break;
     case CMDID_MACRX:                                                          /* Mac Receive notifications */
          /* MacRx notifications are handled here */
          break;
     case CMDID_TXDONE:                                                         /* Tramission done notifications */
          /* Transmit Done notifications are handled here */
          break;
     case CMDID_ADVRECEIVED:                                                    /* Advertisement notifications */
          /* Received advertisement notifications are handled here */
          break;
     default:
          /* nothing to do */
          break;

   }
}

/**
 * @brief    Execute reply call back from API.
 *
 * @param [in] cmdId      command ID.
 *
 * @return  void.
 *
 * Executes reply call back function upon receving reply from API.
 *
 * @note    NA.
 */

void dn_ipmt_reply_cb(uint8_t cmdId) 
{
   app_vars.replyCb();                                                          /* API callback  function */
}

/*=========================== User Application ================================*/
/**
 * @brief    User application sits here.
 *
 * @param       void.
 *
 * @return  uint8_t*    Pointer to an array for transmission is returned .
 *
 * User has the liberty to edit this function to do application specific tasks and send data to be transmitted.
 *
 *
 * @sa           adi_i2c_ReadWrite().
 *
 *
 * @note    If Sensor_data_enable =1 ,then the function transmits temperature data.
 *          Else the function just transmits a dummy data of 0x08.
 */

uint8_t* read_sensor_data(void)
{
   #if(Sensor_data_enable==1)
    /* read temperature registers */
    prologueData[0]     = 0;  /* address of 1st temperature register */
    xfr.pPrologue       = &prologueData[0];
    xfr.nPrologueSize   = 1;
    xfr.pData           = rxData;
    xfr.nDataSize       = 2;
    xfr.bReadNotWrite   = true;
    xfr.bRepeatStart    = true;

    /* blocking read */
    result = adi_i2c_ReadWrite(i2cDevice, &xfr, &hwError_i2c);
    if (result)
       {
         printf("I2c error");
       }

    /* translate temperature reading to human-readable form */
    uint16_t temp;
    float ctemp;
    uint32_t ctemp1;
        
    temp = (rxData[0] << 8) | (rxData[1]);
    ctemp = (temp >> 3) / 16.0;
    ctemp1=(int)(ctemp*100);
    sensor_Value_temp[25]     = (ctemp1>>8)  & 0xff;
    sensor_Value_temp[26]     = (ctemp1>>0)  & 0xff;
    return sensor_Value_temp;
             
   #else
    switch_var = !switch_var;
    if(switch_var==true)
     return ((uint8_t*)dummy_data+1);
    else
     return ((uint8_t*)dummy_data+0);
    #endif
}

/*=========================== APIs ============================================*/

/**
 * @brief    Function executed when time our occurs.
 *
 * @param void.
 *
 * @return  void.
 *
 * Executes getMoteStatus function when time out occurs.
 *
 * @sa      dn_ipmt_cancelTx().
 * @sa      scheduleEvent().
 * @sa      api_getMoteStatus().
 *
 * @note   This function is only used when the application is written using RTC delays,
 *         the current application does not use this function .
 */

void api_response_timeout(void) {
   /* issue cancel command */
   dn_ipmt_cancelTx();
   
   /* Reset Flag values */
   Flag_Check=Mote_Status;

   /* schedule status event */
   scheduleEvent(&api_getMoteStatus);
}

/*=========================== getMoteStatus ===================================*/
/**
 * @brief    Requests for the status of the Module.
 *
 * @param   void.
 *
 * @return  void.
 *
 * Status of the module is obtained once this function executes.
 *
 * @sa      setCallback().
 * @sa      dn_ipmt_getParameter_moteStatus().
 *
 * @note    NA.
 */
void api_getMoteStatus(void) 
{

   /* API callback */
   setCallback(api_getMoteStatus_reply);
   
   transmit_clear=false;
   delay_count=0;

   /* issue function */
   dn_ipmt_getParameter_moteStatus(
      (dn_ipmt_getParameter_moteStatus_rpt*)(app_vars.replyBuf)
   );
}

/**
 * @brief   Reply  for api_getMoteStatus is processed here.
 *
 * @param   void.
 *
 * @return  void.
 *
 * Reply to the getMoteStatus is processed here.
 *
 * @sa      cancelEvent().
 * @sa      scheduleEvent().
 * @sa      api_getMoteStatus().
 *
 * @note    NA.
 */
void api_getMoteStatus_reply(void)
{
   dn_ipmt_getParameter_moteStatus_rpt* reply;
   
   /* cancel timeout */
   cancelEvent();

   /* parse reply */
   reply = (dn_ipmt_getParameter_moteStatus_rpt*)app_vars.replyBuf;
   
   /* choose next step */
   switch (reply->state) 
   {
      case MOTE_STATE_IDLE:
           transmit_clear=true;
           delay_count=0;
           Flag_Check=Open_Socket;
           break;
      case MOTE_STATE_OPERATIONAL:
           transmit_clear=true;
           delay_count=0;
           Flag_Check=SendTo;
           break;
      default:
           transmit_clear=true;
           delay_count=0;
           Flag_Check=Mote_Status;
           break;
   }
}

/*=========================== openSocket ======================================*/
/**
 * @brief   Requests for Opening a Socket for data transfer.
 *
 * @param   void.
 *
 * @return  void.
 *
 * Functions opens up a socket for data transfer,the protocol used here is UDP.
 *
 * @sa      setCallback().
 * @sa      dn_ipmt_openSocket().
 *
 * @note    NA.
 */
void api_openSocket(void) 
{
   
   /* API callback */
   setCallback(api_openSocket_reply);
   
   transmit_clear=false;
   delay_count=0;
   
   /* issue function */
   dn_ipmt_openSocket(
      0,                                              /* protocol */
      (dn_ipmt_openSocket_rpt*)(app_vars.replyBuf)    /* reply */
   );

}

/**
 * @brief   Reply  for api_openSocket is processed here.
 *
 * @param   void.
 *
 * @return  void.
 *
 * Reply contains the socket ID .
 *
 * @sa      cancelEvent().
 * @sa       scheduleEvent().
 * @sa       api_openSocket().
 *
 * @note    NA.
 */
void api_openSocket_reply(void)
{
   dn_ipmt_openSocket_rpt* reply;
   
   /* cancel timeout */
   cancelEvent();
   
   /* parse reply */
   reply = (dn_ipmt_openSocket_rpt*)app_vars.replyBuf;
   
      /* choose next step */
   switch (reply->RC) 
   {
      case RC_OK:
           transmit_clear=true;
           delay_count=0;
           Flag_Check=Bind_Socket;
           break;
      default:
           transmit_clear=true;
           delay_count=0;
           Flag_Check=Open_Socket;
           break;
   }
   
   /* store the socketID */
   app_vars.socketId = reply->socketId;
}

/*=========================== bindSocket ======================================*/
/**
 * @brief    Requests for the binding the socket opened to a port.
 *
 * @param   void.
 *
 * @return  void.
 *
 * Binding the socket ID with the source port.
 *
 * @sa      setCallback().
 * @sa      dn_ipmt_bindSocket().
 *
 * @note    NA.
 */
void api_bindSocket(void) 
{
   
   /* API callback */
   setCallback(api_bindSocket_reply);
   
   transmit_clear=false;
   delay_count=0;
   
   /* issue function */
   dn_ipmt_bindSocket(
      app_vars.socketId,                              /* socketId */
      SRC_PORT,                                       /* port */
      (dn_ipmt_bindSocket_rpt*)(app_vars.replyBuf)    /* reply */
   );
}
/**
 * @brief    Reply  for api_bindSocket is processed here.
 *
 * @param   void.
 *
 * @return  void.
 *
 * Reply just confirms that the socket ID was bound to the specified port.
 *
 * @sa      cancelEvent().
 * @sa       scheduleEvent().
 * @sa       api_bindSocket().
 *
 * @note    NA.
 */
void api_bindSocket_reply(void) 
{
   dn_ipmt_bindSocket_rpt* reply;
  
   /* cancel timeout */
   cancelEvent();
   
   /* parse reply */
   reply = (dn_ipmt_bindSocket_rpt*)app_vars.replyBuf;
   
   /* choose next step */
   switch (reply->RC) 
   {
      case RC_OK:
           transmit_clear=true;
           delay_count=0;
           Flag_Check=Join;
           break;
      default:
           transmit_clear=true;
           delay_count=0;
           Flag_Check=Bind_Socket;
           break;
   }
}

/*=========================== join ============================================*/
/**
 * @brief    Network Join command is issued here.
 *
 * @param   void.
 *
 * @return  void.
 *
 * The module is requested to join the network.
 *
 * @sa      setCallback().
 * @sa      dn_ipmt_join().
 *
 * @note    Currently default paramenters of the SmartMesh IP network is used.
 */

void api_join(void)
{

   /* API callback */
   setCallback(api_join_reply);
   
   transmit_clear=false;
   delay_count = 0;
  
   /* issue function */
   dn_ipmt_join(
      (dn_ipmt_join_rpt*)(app_vars.replyBuf)     /* reply */
   );
 
}

/**
 * @brief    Reply to the api_join is processed here.
 *
 * @param   void.
 *
 * @return  void.
 *
 * Reply to the join request is handled.
 *
 * @sa      cancelEvent().
 * @sa       scheduleEvent().
 * @sa       api_join().
 *
 * @note    NA.
 */
void api_join_reply(void)
{
   dn_ipmt_join_rpt* reply;
  
   /* cancel timeout */
   cancelEvent();
   
   /* parse reply */
   reply = (dn_ipmt_join_rpt*)app_vars.replyBuf;
     
   /* choose next step */
   switch (reply->RC) 
   {
      case RC_OK:
           delay_count=0;
           break;
      default:
           transmit_clear=true;
           delay_count=0;
           Flag_Check=Join;
           break;
   }
}
/*=========================== sendTo ==========================================*/
/**
 * @brief    This function handles the transmission of data packets to a ipv6 address.
 *
 * @param   void.
 *
 * @return  void.
 *
 * api_sendTo function handles the transmission of data packets to a ipv6 address.
 * here the address is of the manager.
 *
 * @sa      setCallback().
 * @sa      api_sendTo().
 * @sa      memcpy().
 *
 * @note    Data is sent to the manager.Destination and source ports are mentioned in the api_ports section 
 *          of the #defines and the packet ID submitted does not require transmission done acknowledgement.
 */

void api_sendTo(void)
{
  #if(Sensor_data_enable==1)
   uint8_t  payload[27];
  #else
   uint8_t  payload[1];
  #endif
   uint8_t  dest_addr[16];
  
   /* API callback */
   setCallback(api_sendTo_reply);
   
   transmit_clear=false;
   delay_count = 0;
   
   /* create payload */
   #if(Sensor_data_enable==1)
   memcpy(payload,read_sensor_data(),27);
   #else
   memcpy(payload,read_sensor_data(),1);
   #endif
   
   /* Manager Address */
   memcpy(dest_addr,ipv6Addr_manager,16);

   /* issue function */
   dn_ipmt_sendTo(
      app_vars.socketId,                                   /* socketId */
      dest_addr,                                           /* destIP */
      DST_PORT,                                            /* destPort */
      SERVICE_TYPE_BW,                                     /* serviceType */
      0,                                                   /* priority */
      0xffff,                                              /* packetId */
      payload,                                             /* payload */
      sizeof(payload),                                     /* payloadLen */
      (dn_ipmt_sendTo_rpt*)(app_vars.replyBuf)             /* reply */
   );

   

}

/**
 * @brief    Reply to sendTo request is handled here .
 *
 * @param   void.
 *
 * @return  void.
 *
 * sendTo reply processed here.
 *
 * @sa      cancelEvent().
 * @sa      scheduleEvent().
 * @sa      api_sendTo().
 *
 * @note    NA.
 */
void api_sendTo_reply(void)
{
   dn_ipmt_sendTo_rpt* reply;
   
   /* cancel timeout */
   cancelEvent();
   
   /* parse reply */
   reply = (dn_ipmt_sendTo_rpt*)app_vars.replyBuf;

   /* choose next step */
   switch (reply->RC) 
   {
      case RC_OK:
           transmit_clear=true;
           delay_count=0;
           for(int i=0;i<10000;i++)
             for(int j=0;j<10000;j++);                                          /* Random delay */
           break;
      default:
           transmit_clear=true;
           delay_count=0;
           while(delay_count != delay_count_one_sec)                             /* Wait due to lack of resources needed to transmit packets*/
             delay_count++;
           delay_count=0;
           break;
   }
}

/*=========================== main ============================================*/
int main(void)
{
   /* reset local variables */
   memset(&app_vars, 0, sizeof(app_vars));
   
   /* dn_ipmt_init initialises all the variables necessary for the application to function
   This function results in the call of dn_uart_init which initalises the UART of the MCU */ 
   
   /* initialize the ipmt module */
   dn_ipmt_init(
      dn_ipmt_notif_cb,                /* Notification Call Back */
      app_vars.notifBuf,               /* Notification Buffer */
      sizeof(app_vars.notifBuf),       /* Notification Buffer Length */
      dn_ipmt_reply_cb                 /* Reply call back */
   );

   
while(1)
    {
       if(head != tail)
           buffer_handle();                                                     /* here the buffer is checked for incoming data and processed */
       else
       {
        if(transmit_clear == true)
         {
         switch(Flag_Check)
          {
            case Mote_Status: 
                 scheduleEvent(&api_getMoteStatus);                           /* Function call for Mote Status is done here */  
                 break;

            case Open_Socket:
                 scheduleEvent(&api_openSocket);                              /* Function call for opening a socket is done here */
                 break;

            case Bind_Socket:
                 scheduleEvent(&api_bindSocket);                              /* Function call for binding the socket is done here */
                 break;

            case Join:
                 scheduleEvent(&api_join);                                    /* Function call for joining the network is done here */
                 break;

            case SendTo:
                 scheduleEvent(&api_sendTo);                                  /* Function call for Data packet tranmission is done here */
                 break;
                   
         default:
                 break;
          }
        }
        else if(transmit_clear == false)                                                                      /* Wait here till reply is received */
         {  
            delay_count++;
            if(delay_count==delay_count_ten_sec)                                       
             {
               delay_count=0;
               scheduleEvent(&api_response_timeout);                                  /*Time out occurs here */
             }
         }
        else
         {
           /* Booting state or unexpected outcome */
         }
       }
    }
return 0;
}