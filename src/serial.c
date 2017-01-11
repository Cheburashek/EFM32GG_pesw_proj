/*
 * serial.c
 *
 *  Created on: 9 sty 2017
 *      Author: Cheburashek
 */

#include <stdlib.h>
#include "serial.h"
#include "em_dma.h"
#include "dmactrl.h"
#include "em_device.h"
#include "em_cmu.h"
#include "em_gpio.h"


/** LEUART Rx/Tx Port/Pin Location */
#define LEUART_LOCATION    0
#define LEUART_TXPORT      gpioPortD            /* LEUART transmission port */
#define LEUART_TXPIN       4                    /* LEUART transmission pin */
#define LEUART_RXPORT      gpioPortD            /* LEUART reception port */
#define LEUART_RXPIN       5                    /* LEUART reception pin */

#define SERIAL_SYNC_BYTE	0xAA
#define PACKET_BASE_SIZE	3		// prefix + opcode + len

#define TX_BUFFER_SIZE		32
#define RX_BUFF_SIZE		64


static uint8_t pTxBuffer[TX_BUFFER_SIZE];
volatile static uint8_t *pTxToSend;
volatile static uint16_t dataToSend;
static packetRxCompleteCb_t packetRxCompleteCb;

static void setupDma(void);
static void setupLeuart(void);
static void rxDataParser ( uint8_t data );

void serial_Init ( void )
{
	pTxToSend = pTxBuffer;
	/* Initialize LEUART */
	setupLeuart();

	/* Setup DMA */
//	setupDma();
}


/**************************************************************************//**
 * @brief  Setup DMA
 *
 * @details
 *   This function initializes DMA controller.
 *   It configures the DMA channel to be used for LEUART0 transmit
 *   and receive. The primary descriptor for channel0 is configured for
 *   a single data byte transfer. For continous data reception and transmission
 *   using LEUART DMA loopmode is enabled for channel0.
 *   In the end DMA transfer cycle is configured to basicMode where
 *   the channel source address, destination address, and
 *   the transfercount per dma cycle have been specified.
 *
 *****************************************************************************/
static void setupDma(void)
{
  /* DMA configuration structs */
  DMA_Init_TypeDef       dmaInit;
  DMA_CfgChannel_TypeDef channelCfg;
  DMA_CfgDescr_TypeDef   descrCfg;
  DMA_CfgLoop_TypeDef    loopCfg;

  /* Initializing the DMA */
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);

  /* Setting up channel */
  channelCfg.highPri   = false; /* Can't use with peripherals */
  channelCfg.enableInt = false; /* Interrupt not needed in loop transfer mode */

  /* Configure channel 0 */
  /*Setting up DMA transfer trigger request*/
  channelCfg.select = DMAREQ_LEUART0_RXDATAV;
  channelCfg.cb     = NULL;
  DMA_CfgChannel(0, &channelCfg);

  /* Setting up channel descriptor */
  /* Destination is LEUART_Tx register and doesn't move */
  descrCfg.dstInc = dmaDataIncNone;

  /* Source is LEUART_RX register and transfers 8 bits each time */
  descrCfg.srcInc = dmaDataIncNone;
  descrCfg.size   = dmaDataSize1;

  /* We have time to arbitrate again for each sample */
  descrCfg.arbRate = dmaArbitrate1;
  descrCfg.hprot   = 0;

  /* Configure primary descriptor  */
  DMA_CfgDescr(0, true, &descrCfg);

  /* Configure loop transfer mode */
  loopCfg.enable = true;
  loopCfg.nMinus1 = 0;  /* Single transfer per DMA cycle*/
  DMA_CfgLoop(0, &loopCfg);

  /* Activate basic dma cycle using channel0 */
  DMA_ActivateBasic(0,
		    true,
		    false,
		    (void *)&LEUART0->TXDATA,
		    (void *)&LEUART0->RXDATA,
		    0);
}

static void setupLeuart(void)
{
	/* Enable peripheral clocks */
	CMU_ClockEnable(cmuClock_HFPER, true);
	/* Configure GPIO pins */
	CMU_ClockEnable(cmuClock_GPIO, true);
	/* To avoid false start, configure output as high */
	GPIO_PinModeSet(LEUART_TXPORT, LEUART_TXPIN, gpioModePushPull, 1);
	GPIO_PinModeSet(LEUART_RXPORT, LEUART_RXPIN, gpioModeInput, 0);

	LEUART_Init_TypeDef init = LEUART_INIT_DEFAULT;

	/* Enable CORE LE clock in order to access LE modules */
	CMU_ClockEnable(cmuClock_CORELE, true);

	/* Select LFXO for LEUARTs (and wait for it to stabilize) */
	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
	CMU_ClockEnable(cmuClock_LEUART0, true);

	/* Do not prescale clock */
	CMU_ClockDivSet(cmuClock_LEUART0, cmuClkDiv_1);

	/* Configure LEUART */
	init.enable = leuartDisable;

	LEUART_Init(LEUART0, &init);

	/* Enable pins at default location */
	LEUART0->ROUTE = LEUART_ROUTE_RXPEN | LEUART_ROUTE_TXPEN | LEUART_LOCATION;

	/* Set RXDMAWU to wake up the DMA controller in EM2 */
	//LEUART_RxDmaInEM2Enable(LEUART0, true);

	LEUART_IntEnable ( LEUART0, LEUART_IEN_TXC | LEUART_IEN_RXDATAV );	// Enable tx complete and rx data int
	NVIC_ClearPendingIRQ ( LEUART0_IRQn );
	NVIC_EnableIRQ ( LEUART0_IRQn );

  /* Finally enable it */
//	LEUART_Enable(LEUART0, leuartEnable);
}

bool serial_SendPacket ( uint8_t *pData, uint16_t len, uint8_t opCode )
{
	bool retVal = false;
	uint8_t *pTemp = NULL;
	pTemp = malloc ( len + PACKET_BASE_SIZE );

	if ( pTemp )
	{
		pTemp[0] = SERIAL_SYNC_BYTE;
		pTemp[1] = opCode;
		pTemp[2] = len;

		memcpy ( pTemp + PACKET_BASE_SIZE, pData, len );
		retVal = serial_SendData ( pTemp, len + PACKET_BASE_SIZE );
		free ( pTemp );
	}
	return retVal;
}

bool serial_SendData ( uint8_t *pData, uint16_t len )
{
	if ( pData && len && (len <= (TX_BUFFER_SIZE-dataToSend)) )
	{
		memcpy ( pTxToSend, pData, len );

		dataToSend += len;
		if ( pTxBuffer == pTxToSend )	// When first data to send from buffer
		{
			SERIAL_ENABLE_TX();
			LEUART_Tx ( LEUART0, *pTxToSend );
			pTxToSend++;
			dataToSend--;
		}
		return true;
	}
	return false;
}

void LEUART0_IRQHandler ( void )
{
	uint32_t flags;
	flags = LEUART_IntGet ( LEUART0 );

	if ( LEUART_IF_TXC == (flags & LEUART_IF_TXC) )
	{
		if ( dataToSend )
		{
			LEUART_Tx ( LEUART0, *pTxToSend );
			pTxToSend++;
			dataToSend--;
		}
		else
		{
			pTxToSend = pTxBuffer;
			SERIAL_DISABLE_TX();
		}

		LEUART_IntClear ( LEUART0, LEUART_IF_TXC );
	}
	if ( LEUART_IF_RXDATAV == (flags & LEUART_IF_RXDATAV) )
	{
		rxDataParser ( LEUART_RxDataGet(LEUART0) );
		LEUART_IntClear ( LEUART0, LEUART_IF_RXDATAV );
	}

}

static void rxDataParser ( uint8_t data )
{
	static uint8_t rxBuff[RX_BUFF_SIZE];
	static uint16_t rxActLen = 0;
	static uint16_t actPacketLen = 0;
	static uint8_t actOpCode = 0;

	if ( (SERIAL_SYNC_BYTE == data) && (0 == rxActLen) )
	{
		rxActLen++;
	}
	else if ( 1 == rxActLen )
	{
		actOpCode = data;
		rxActLen++;
	}
	else if ( 2 == rxActLen )
	{
		actPacketLen = data;
		rxActLen++;
	}
	else if ( 3 <= rxActLen )
	{
		rxBuff[rxActLen-PACKET_BASE_SIZE] = data;
		rxActLen++;
		if ( actPacketLen == (rxActLen-PACKET_BASE_SIZE) )	// When end of packet
		{

			if ( packetRxCompleteCb )
			{
				packetRxCompleteCb ( actOpCode, rxBuff, actPacketLen );
			}
			rxActLen = 0;
		}
	}

	if ( rxActLen > RX_BUFF_SIZE )
	{
		rxActLen = 0;	// Big problem
	}

}

void serial_RxPacketCompleteCbSet ( packetRxCompleteCb_t cb )
{
	packetRxCompleteCb = cb;
}

void serial_SetMode ( LEUART_Enable_TypeDef mode )
{
	LEUART_Enable ( LEUART0, mode );
}
