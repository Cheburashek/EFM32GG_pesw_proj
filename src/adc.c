/*
 * adc.c
 *
 *  Created on: 9 sty 2017
 *      Author: Cheburashek
 */

#include "adc.h"
#include "display.h"
#include "em_adc.h"
#include "em_cmu.h"
#include "bsp.h"

volatile uint32_t val;

void adc_Init ( void )
{
	CMU_ClockEnable( cmuClock_ADC0, true );

	ADC_Init_TypeDef initStr = {
			.ovsRateSel = _ADC_CTRL_OVSRSEL_X16,
			.lpfMode 	= _ADC_CTRL_LPFMODE_RCFILT,
			.warmUpMode = _ADC_CTRL_WARMUPMODE_NORMAL,
			.tailgate 	= false
	};

	ADC_Init ( ADC0, &initStr );

	ADC_InitSingle_TypeDef singleInitStr = {
			.prsSel 	= adcPRSSELCh0,
			.acqTime 	= adcAcqTime4,
			.reference 	= adcRef2V5,	// TODO?
			.resolution = adcRes12Bit,
			.input 		= adcSingleInputCh0,
			.diff 		= false,
			.prsEnable	= true,
			.leftAdjust	= false,
			.rep 		= false,
	};

	ADC_InitSingle ( ADC0, &singleInitStr );

    /* Manually set some calibration values */
    ADC0->CAL = (0x7C << _ADC_CAL_SINGLEOFFSET_SHIFT) | (0x1F << _ADC_CAL_SINGLEGAIN_SHIFT);

	ADC_IntEnable ( ADC0, ADC_IEN_SINGLE );
	NVIC_ClearPendingIRQ ( ADC0_IRQn );
	NVIC_EnableIRQ(ADC0_IRQn);

	ADC_Start ( ADC0, adcStartSingle );
}


void ADC0_IRQHandler(void)
{
	uint32_t flags;

	BSP_LedToggle (0);

	/* Clear interrupt flags */
	flags = ADC_IntGet(ADC0);
	ADC_IntClear(ADC0, flags);

	val = ADC_DataSingleGet ( ADC0 );
	display_AdcSet ( val );
	ADC_Start ( ADC0, adcStartSingle );
}

