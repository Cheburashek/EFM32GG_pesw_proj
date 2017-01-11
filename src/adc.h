/*
 * adc.h
 *
 *  Created on: 9 sty 2017
 *      Author: Cheburashek
 */

#ifndef SRC_ADC_H_
#define SRC_ADC_H_

#include "em_adc.h"

#define PRS_ADC_CHANNEL	adcPRSSELCh0

void adc_Init ( void );
bool adc_IsConvInProgress ( void );
uint32_t adc_GetVal ( void );
uint32_t adc_GetVal_mV ( void );
void adc_StartSingle ( void );


#endif /* SRC_ADC_H_ */
