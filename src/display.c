/*
 * display.c
 *
 *  Created on: 9 sty 2017
 *      Author: Cheburashek
 */


#include "display.h"
#include "segmentlcd.h"
#include "em_lcd.h"
#include "bsp.h"

void display_Init ( void )
{
	SegmentLCD_Init(false);
}

void display_AdcSet ( uint32_t data )
{
	SegmentLCD_Number ( (uint32_t)data );
}
