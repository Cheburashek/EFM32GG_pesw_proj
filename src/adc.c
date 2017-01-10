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


#define ADC_RESOLUTION 		adcRes12Bit
#define ADC_REFERENCE_V		adcRef2V5

static bool dataReadyFlag = false;


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
			.prsSel 	= PRS_ADC_CHANNEL,
			.acqTime 	= adcAcqTime4,
			.reference 	= ADC_REFERENCE_V,	// TODO?
			.resolution = ADC_RESOLUTION,
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

}

bool adc_GetDataReadyFlag ( void )
{
	return dataReadyFlag;
}

void adc_ClearDataReadyFlag ( void )
{
	dataReadyFlag = false;
}

uint32_t adc_GetVal ( void )
{
	return ADC_DataSingleGet(ADC0);
}

uint32_t adc_GetVal_mV ( void )
{
	uint16_t res = 0;
	uint16_t ref = 0;

	switch ( ADC_RESOLUTION )
	{
	case adcRes12Bit:
		res = 0x0FFF;
		break;

	case adcRes8Bit:
		res = 0x00FF;
		break;

	case adcRes6Bit:
		res = 0x003F;
		break;

	default:
		break;
	}

	switch ( ADC_REFERENCE_V )
	{
	case adcRef1V25:
		ref = 1250;
		break;

	case adcRef2V5:
		ref = 2500;
		break;

	default:
		break;
	}

	return (ADC_DataSingleGet(ADC0)*ref) / res;
}

void adc_StartSingle ( void )
{
	ADC_Start ( ADC0, adcStartSingle );
}

void ADC0_IRQHandler(void)
{
	uint32_t flags;

	BSP_LedToggle (0);

	/* Clear interrupt flags */
	flags = ADC_IntGet(ADC0);
	ADC_IntClear(ADC0, flags);

	dataReadyFlag = true;
}

