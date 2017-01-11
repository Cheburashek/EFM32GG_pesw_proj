/*
 * serial.h
 *
 *  Created on: 9 sty 2017
 *      Author: Cheburashek
 */

#ifndef SRC_SERIAL_H_
#define SRC_SERIAL_H_

#include "em_leuart.h"

#define SERIAL_ADCDATA_UINT16_OPCODE	0xAB


#define SERIAL_DISABLE_TX() LEUART0->CMD |= LEUART_CMD_TXDIS	// Disable only TX
#define SERIAL_ENABLE_TX() LEUART0->CMD |= LEUART_CMD_TXEN		// Enable TX
#define SERIAL_DISABLE_RX() LEUART0->CMD |= LEUART_CMD_RXDIS	// Disable only TX
#define SERIAL_ENABLE_RX() LEUART0->CMD |= LEUART_CMD_RXEN		// Enable TX

typedef void (*packetRxCompleteCb_t) ( uint8_t opCode, uint8_t *pData , uint8_t len );


void serial_Init ( void );
void serial_SetMode ( LEUART_Enable_TypeDef mode );
bool serial_SendData ( uint8_t *pData, uint16_t len );
bool serial_SendPacket ( uint8_t *pData, uint16_t len, uint8_t opCode );
void serial_RxPacketCompleteCbSet ( packetRxCompleteCb_t cb );

#endif /* SRC_SERIAL_H_ */
