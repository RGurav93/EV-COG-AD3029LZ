/*!
 *****************************************************************************
  @file:  uart_support.c

  @brief: This file transmits the data over ADuCM3029 UART link

  @details:
 -----------------------------------------------------------------------------
 Copyright (c) 2020 Analog Devices, Inc.  All rights reserved.

 This software is proprietary to Analog Devices, Inc. and its licensors.
 By using this software you agree to the terms of the associated
 Analog Devices Software License Agreement.

*****************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <adi_processor.h>
#include <drivers/dma/adi_dma.h>

#include "adi_support.h"
#include "uart_support.h"

/******************************************************************************/
/********************** Variables and User defined data types *****************/
/******************************************************************************/

/* Create a structure for DAM data transfer buffer */
typedef struct {
	const char
	*p_start_address; /* Address of buffer passed down to the UART driver */
	uint32_t size;        		 /* Size of buffer in bytes */
	uint32_t index;        		 /* Buffer index */
	bool 	 in_use;             /* Buffer in use flag */
} dma_buffer_t;

/* Create a structure for DMA UART data transfer channel */
typedef struct {
	dma_buffer_t normal_buffer;		/* Normal operating buffer */
} uart_dma_channel_t;

/* Create DMA UART Tx channel structure */
uart_dma_channel_t dma_uart_tx_chn = {
	.normal_buffer = {
		.p_start_address = NULL,
		.size = 0,
		.index = 0,
		.in_use = false
	}
};

/* Debug string array */
char debug_string[150];

/* Override DMA UART Tx channel weak interrupt */
#ifdef __cplusplus
extern "C"
{
#endif
extern void DMA_UART_TX_Int_Handler(void);
#ifdef __cplusplus
}
#endif

static void uart_Write(const char *pBuffer, uint32_t const nBufSize);
static void uart_wait_for_data_transfer(void);

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/******************************************************************************/
/********************** Functions Definitions *********************************/
/******************************************************************************/

/*!
 * @brief	Initialize the UART port
 *
 * @return  void
 *
 * @details	This initializes the UART port to transmit data over UART link
 */
void uart_init(void)
{
	/* Configure Line Control register */
	pADI_UART0->LCR = ((UART0_CFG_WORD_LENGTH << BITP_UART_LCR_WLS)  |
			   (UART0_CFG_STOP_BIT << BITP_UART_LCR_STOP) |
			   (UART0_CFG_ENABLE_PARITY << BITP_UART_LCR_PEN));

	/* Configure Div-C in Baudrate Divider register */
	pADI_UART0->DIV = UART0_CFG_DIVC;

	/* Configure Div-M and Div-N in Fractional Baudrate register */
	pADI_UART0->FBR = ((UART0_CFG_DIVN << BITP_UART_FBR_DIVN) |
			   (UART0_CFG_DIVM << BITP_UART_FBR_DIVM) |
			   (BITM_UART_FBR_FBEN));

	/* Configure Second Line Control register */
	pADI_UART0->LCR2 = UART0_CFG_OSR;

	/* Configure FIFO Control register */
	pADI_UART0->FCR  = ((UART0_CFG_ENABLE_FIFO << BITP_UART_FCR_FIFOEN)|
			    (UART0_CFG_TRIG_LEVEL << BITP_UART_FCR_RFTRIG));

	/* Configure Half Duplex Control register */
	pADI_UART0->RSC  = ((UART0_CFG_SOUT_POLARITY << BITP_UART_RSC_OENP)  |
			    (UART0_CFG_DEASSERTION << BITP_UART_RSC_OENSP) |
			    (UART0_CFG_DISABLE_RX << BITP_UART_RSC_DISRX) |
			    (UART0_CFG_HOLD_TX << BITP_UART_RSC_DISTX));

	/* Configure Interrupt Enable register */
	pADI_UART0->IEN  = ((UART0_CFG_ENABLE_MODEM_STATUS_INTERRUPT <<
			     BITP_UART_IEN_EDSSI) |
			    (UART0_CFG_ENABLE_RX_STATUS_INTERRUPT << BITP_UART_IEN_ELSI));

	/* Enable UART Interrupt. */
	NVIC_ClearPendingIRQ(UART_EVT_IRQn);
	NVIC_EnableIRQ(UART_EVT_IRQn);

	/* Enable the interrupt for the DMA UART0 Tx channel */
	NVIC_EnableIRQ(DMA0_CH8_DONE_IRQn);
}


/*!
 * @brief	Write data to UART
 *
 * @param	[in] Pointer to output data buffer
 *
 * @param	[in] Buffer size in bytes
 *
 * @return  UART data write error status
 *
 * @details	This writes to data into UART DMA Tx channel to send over UART link
 */
static void uart_Write(const char *p_buffer, uint32_t const buff_size)
{
	/* Return if DMA channel already in use */
	if (dma_uart_tx_chn.normal_buffer.in_use == true) {
		return;
	}

	/* Set the start address of the data buffer to be transmitted */
	dma_uart_tx_chn.normal_buffer.p_start_address =  p_buffer;

	/* Set the buffer size to the size of the data buffer */
	dma_uart_tx_chn.normal_buffer.size = buff_size;

	/* Initialize the buffer index to zero because we will start shifting out
	   the Tx data from the first position of the buffer
	*/
	dma_uart_tx_chn.normal_buffer.index = 0U;

	/* Mark buffer in use */
	dma_uart_tx_chn.normal_buffer.in_use = true;

	/* Enable clear source address decrement for TX channel DMA */
	pADI_DMA0->SRCADDR_CLR = 1u << (uint32_t)UART0_TX_CHANn;

	/* Enable Tx channel DMA */
	pADI_DMA0->EN_SET = 1u << UART0_TX_CHANn;

	/* Enable UART peripheral to generate DMA requests */
	pADI_DMA0->RMSK_CLR = 1u << UART0_TX_CHANn;

	/* Set the primary control data structure as the current DMA descriptor */
	pADI_DMA0->ALT_CLR = 1u << UART0_TX_CHANn;

	/* Fill in the DMA RAM descriptors */
	pPrimaryCCD[UART0_TX_CHANn].DMASRCEND = ((uint32_t)
						dma_uart_tx_chn.normal_buffer.p_start_address +
						(uint32_t)(dma_uart_tx_chn.normal_buffer.size - 1u));

	pPrimaryCCD[UART0_TX_CHANn].DMADSTEND = (uint32_t)&pADI_UART0->TX;

	pPrimaryCCD[UART0_TX_CHANn].DMACDC    = ((uint32_t)ADI_DMA_INCR_NONE <<
						DMA_BITP_CTL_DST_INC)     |
						((uint32_t)ADI_DMA_INCR_1_BYTE << DMA_BITP_CTL_SRC_INC)  |
						(ADI_DMA_WIDTH_1_BYTE << DMA_BITP_CTL_SRC_SIZE)          |
						(0u << DMA_BITP_CTL_R_POWER)                             |
						((dma_uart_tx_chn.normal_buffer.size - 1u) << DMA_BITP_CTL_N_MINUS_1) |
						(DMA_ENUM_CTL_CYCLE_CTL_BASIC << DMA_BITP_CTL_CYCLE_CTL);

	/* Enable UART DMA request interrupt for the Tx channel */
	pADI_UART0->IEN |= (BITM_UART_IEN_EDMAT);

	/* Wait for data to shift out of UART Tx buffer */
	uart_wait_for_data_transfer();
}


/*!
 * @brief	Wait for data to send out of Tx buffer (blocking call)
 *
 * @return  void
 *
 * @details	This makes a blocking call to send data out of DMA Tx buffer
 * 			The DMA interrupt request is generated when data is shifted
 * 			out of DMA Tx channel and buffer in_use flag is cleared from it
 */
static void uart_wait_for_data_transfer(void)
{
	while (dma_uart_tx_chn.normal_buffer.in_use == true) {
		// Wait for Tx buffer to get empty
	}
}


/*!
 * @brief	Interrupt handler for DMA UART Tx channel
 *
 * @return  void
 *
 * @details	This is a interrupt handler routine for DMA UART Tx channel
 * 			The interrupt is triggered when UART data is shifted out of Tx buffer
 */
#ifdef __cplusplus
extern "C"
{
#endif
void DMA_UART_TX_Int_Handler(void)
{
	/* Now that this transaction has completed, this buffer is no longer in use. */
	dma_uart_tx_chn.normal_buffer.in_use = false;

	while (((pADI_UART0->LSR & BITM_UART_LSR_TEMT) != BITM_UART_LSR_TEMT)
	       || (pADI_UART0->TFC != 0u)) {
		/* Waiting until TFC becomes zero */
	}

	/* Disable Tx buffer interrupts. */
	pADI_UART0->IEN &= (uint16_t)~(BITM_UART_IEN_ETBEI | BITM_UART_IEN_EDMAT);
}
#ifdef __cplusplus
}
#endif


/*!
 * @brief	Wrapper to dispatch debug messages over UART link
 *
 * @param  [in] Pointer to data buffer
 *
 * @details	This is a wrapper function that transmits debug messages
 * 			over UART link using uart_write() method
 */
void uart_msg_dispatch(char *pdata)
{
	uart_Write(pdata, strlen(pdata));
}
