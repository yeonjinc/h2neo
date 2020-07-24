#include <msp430.h> 

void ADC12_0_Init(void);

/*
 * main.c
 */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
	
	return 0;
}


/*
 * opticalSensor.c
 *
 * 		- Initialize ADC12_0 (P6.0)
 * 		-
 *
 * 	Not sure if this is the cleanest way to do things but it works for now :-)
 *
 *
 *  Created on: Jul 14, 2020
 *      Author: Jenny Cho
 */


/******************************************************************************
 *
 * ADC12_0_Init()
 *
 * Initialize P6.0 as ADC12. Uses 0 ~ (internal) 1.5V as input range and internal
 * clock for timing.
 * There may be some ignored code (between ADC12CTL0 REFCTL0 registers)
 *
 *****************************************************************************/
void ADC12_0_Init() {
	/*
	 * ADC12SHT0_x   -- sample-and-hold time. Defines num ADC12CLK cycles in the sampling period
	 * 						 0b100 --> 64 cycles
	 * ADC12ON       -- ADC12_A on
	 * ADC12MSC      -- set multiple sample and conversion (valid for sequence or repeated modes)
	 */
	REFCTL0 |= REFON;				// enable
	REFCTL0 |= REFVSEL_0;			// 1.5V
//	REFCTL0 |= REFVSEL_1;			// 2.0V
//	REFCTL0 |= REFVSEL_2;			// 2.5V
	REFCTL0 &= ~REFMSTR;  			// Reset REFMSTR to hand over control to ADC12_A ref control registers
	ADC12CTL0 = ADC12ON + ADC12SHT0_8 + ADC12MSC;

	/******************* UNUSED *******************
	 * ADC12REFON    -- reference generator ON
	 * ADC12REF2_5V  -- 0b->1.5V, 1b->2.5V (ADC12REFON must be set)
	 */
//	ADC12CTL0 |= ADC12REFON;
//	ADC12CTL0 &= ~ADC12REF2_5V;  	// 0b, ref voltage = 1.5V
//	ADC12CTL0 |= ADC12REF2_5V;  	// 1b, ref voltage = 2.5V

	/*
	 * ADC12SHP      -- sample-and-hold-pulse-mode select (1->SAMPCON sourced from sampling timer)
	 * ADC12CONSEQ_x -- 2->repeat single channel
	 */
	ADC12CTL1 = ADC12SHP+ADC12CONSEQ_2;       	// Use sampling timer, set mode
	ADC12CTL1 |= ADC12SSEL_0;					// ADC12OSC (MODCLK)
	ADC12CTL2 |= ADC12PDIV;						// Predivide by 4 (0b == prediv by 1)

	ADC12IE = 0x01;                       	    // Enable ADC12IFG.0

	ADC12MCTL0 = ADC12SREF_1;            	    // Vr+ = Vref+ and Vr- = AVss
	volatile unsigned int i;
	for (i=0; i<0x30; i++);              	    // Delay for reference start-up

	ADC12CTL0 |= ADC12ENC;               	    // Enable conversions
	/*
	 * Enable A/D channel A0 (P6.0)
	 * If no ports connect to desired A##, then use ADC12MCTLx register (x == ##)
	 */
	P6SEL |= 0x01;
}
