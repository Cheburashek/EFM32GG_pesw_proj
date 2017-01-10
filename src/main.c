/**************************************************************************//**
 * @file
 * @brief LCD controller demo for EFM32GG_STK3700 development kit
 * @version 5.0.0
 ******************************************************************************
 * @section License
 * <b>Copyright 2015 Silicon Labs, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

//E:\Studia\Pesw\EFM32GG_pesw_proj\GNU ARM v4.9.3 - Debug\proj_Pesw_2.hex

#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_prs.h"
#include "em_gpio.h"
#include "bsp.h"
#include "bsp_trace.h"
#include "gpiointerrupt.h"

#include "display.h"
#include "adc.h"
#include "serial.h"


#define PB0_PORT                gpioPortB
#define PB0_PIN                 9
#define PB1_PORT                gpioPortB
#define PB1_PIN                 10
#define ADC_VALUES_TAB_SIZE 	5

volatile uint32_t msTicks; /* counts 1ms timeTicks */
static uint32_t valTab[ADC_VALUES_TAB_SIZE];
static uint8_t currentMeasId = 0;

/* Locatl prototypes */
static void Delay(uint32_t dlyTicks);
static void gpioCallback ( uint8_t pin );
static void gpioInit ( void );
static uint32_t getMeanVal ( void );

/**************************************************************************//**
 * @brief SysTick_Handler
 *   Interrupt Service Routine for system tick counter
 * @note
 *   No wrap around protection
 *****************************************************************************/
void SysTick_Handler(void)
{
  msTicks++;       /* increment counter necessary in Delay()*/
}


/**************************************************************************//**
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 *****************************************************************************/
void Delay(uint32_t dlyTicks)
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) ;
}


static void setupPRS(void)
{
  CMU_ClockEnable(cmuClock_PRS, true);

  /* Use PRS location 0 and output PRS channel 0 on GPIO PORTB. */
  PRS->ROUTE = PRS_ROUTE_CH1PEN;

  /* PRS channel 0 configuration. */
  PRS_SourceSignalSet(
		  PRS_ADC_CHANNEL,
		  PRS_CH_CTRL_SOURCESEL_GPIOH,		// TODO?
		  PRS_CH_CTRL_SIGSEL_GPIOPIN10,
		  prsEdgeNeg
		  );
}

static void gpioInit ( void )
{
	/* Enable GPIO clock. */
	CMU_ClockEnable(cmuClock_GPIO, true);

	/* Configure PB9 as input and enable interrupt. (PB0) */
//	GPIO_PinModeSet(PB0_PORT, PB0_PIN, gpioModeInput, 0);
//	GPIO_IntConfig(PB0_PORT, PB0_PIN, false, true, true);

	/* Configure PB9 as input and enable interrupt. (PB0) */
	GPIO_PinModeSet(PB1_PORT, PB1_PIN, gpioModeInput, 0);
	GPIO_IntConfig(PB1_PORT, PB1_PIN, false, true, true);


	GPIOINT_Init();
//	GPIOINT_CallbackRegister(PB0_PIN, gpioCallback);
	GPIOINT_CallbackRegister(PB1_PIN, gpioCallback);

}

static void gpioCallback ( uint8_t pin )	// TODO: only temporary solution while problem with prs exists
{
	if (pin == PB0_PIN)
	{

	}
	else if (pin == PB1_PIN)
	{
		adc_StartSingle();	// TODO: do dokumentacji - u¿ycie PSR nie ma sensu, bo ADC i tak nie dzia³a w EM2, a prs z zasady ma dzia³aæ bez wybudzania procka.
	}
}

static uint32_t getMeanVal ( void )
{
    uint32_t meanVal = 0;

	for ( uint8_t i = 0; i < ADC_VALUES_TAB_SIZE; i++ )
	{
		meanVal += valTab[i];
	}
	return (meanVal / ADC_VALUES_TAB_SIZE);
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();

  /* If first word of user data page is non-zero, enable eA Profiler trace */
  BSP_TraceProfilerSetup();
  BSP_LedsInit();
  BSP_LedClear(0);
  BSP_LedClear(1);

  /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) while (1) ;

  gpioInit();

//  setupPRS();

  serial_Init();
  display_Init();
  adc_Init ();
//  BSP_LedSet(1);

  display_AdcValSet(1234);

  /* Infinite loop with test pattern. */
  while (1)
  {
	  EMU_EnterEM2(false);	// EM2 for LCD usage

	  while ( !adc_GetDataReadyFlag() ){EMU_EnterEM1();}	// Wait for data ready flag

	  valTab[currentMeasId] = adc_GetVal_mV();

	  if ( currentMeasId == (ADC_VALUES_TAB_SIZE-1) ) // When mean value shall be calculated and shown on LCD
	  {
		  display_AdcValSet ( getMeanVal() );	// Show mean value on LCD
	  }
	  currentMeasId++;

	  if ( ADC_VALUES_TAB_SIZE == currentMeasId )
	  {
		  currentMeasId = 0;
	  }

	  adc_ClearDataReadyFlag();


  }
}


