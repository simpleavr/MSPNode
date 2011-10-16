//******************************************************************************
/*
    Vibra Pad test code for LaunchPad
    Generates "slow" interrupts in the order of seconds
    Blink an LED on/off to test
    Author: Paolo Di Prodi
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */
//******************************************************************************

#include "msp430g2231.h"

volatile unsigned int T0;
void Setup(void);                       // Function protos
unsigned int Cal_VLO(void);

/**
* Reads ADC 'chan' once using AVCC as the reference.
**/
int analogRead(unsigned int pin) {
  ADC10CTL1 = ADC10SSEL_3 + pin;// Select SMCLK and pin
  ADC10CTL0 |= ENC + ADC10SC; // enable and start conversion
  while (1) {
    if ((ADC10CTL1 ^ ADC10BUSY) & ((ADC10CTL0 & ADC10IFG)==ADC10IFG)) { // ensure conversion is complete
      ADC10CTL0 &= ~(ADC10IFG +ENC); // disable conversion and clear flag
      break;
    }
  }
  return ADC10MEM;
}

void main(void)
{
  WDTCTL = WDTPW + WDTHOLD;             // Stop watchdog timer
  Setup();                              // Call init routine

  for (;;)
  {
    LPM3;
     
    //debug LED blink
    P1OUT ^= BIT6;              
    //wake up the RFM12b an do something here

  }//end for
}//end main


void Setup(void)
{
  unsigned int counts;
  signed long temp = 8000000;
  ADC10CTL0 = ADC10ON + ADC10SHT_0 + SREF_0; // ACD10 on, 4 clock cycles per sample, Use Vcc/Vss references
  ADC10AE0 |= BIT0 + BIT1 ;                   // P1.0 or P1.1 ADC option select 
 
  // Ports
  P1DIR |= BIT6;             // P1.6 to output
  P1OUT |= BIT6;             // P1.6 starting with LED on
  //ADC10AE = BIT4;                       // Disable A4 CMOS buffer

// Setup DCO and VLO
  BCSCTL1 = CALBC1_1MHZ;                // Use 1Mhz cal data for DCO
  DCOCTL = CALDCO_1MHZ;                 // Use 1Mhz cal data for DCO
  BCSCTL3 = LFXT1S_2;                   // Use VLO for ACLK

  counts = Cal_VLO();                   // counts is # of VLO clocks in 8MHz

// setup TA for 8 second interrupt
  TACTL = TASSEL_1 + ID_3 + TACLR;      // TA = ACLK/8
// setup TA for 1 second interrupt
  //TACTL = TASSEL_1 + TACLR;      // without any division is one second period good to start with
  TACCTL0 = CCIE;                       // Enable CCR0 interrupt
  //TACCTL1 = CCIE;                       // Enable CCR1 interrupt
// Divide 8Mhz by counts to get # of VLO counts in 1 second period
  T0 = 0;
  do {
    temp -= counts;
    T0++;
  } while (temp > 0);

  TACCR0 = T0;                           // TACCR0 period for 8 second
  TACCR1 = TACCR0 >> 1;                 // Put quarter-period into TACCR1 for use
                                        // when sampling for instance to make it more responsive
// Start timer
  TACTL |= MC_1;                        // Up mode
// Enable interrupt
  _EINT();
}

unsigned int Cal_VLO (void)
{
  unsigned int First_Cap, counts;

  BCSCTL1 |= DIVA_3;                    // Divide ACLK by 8
  TACCTL0 = CM_1 + CCIS_1 + CAP;        // Capture on ACLK
  TACTL = TASSEL_2 + MC_2 + TACLR;      // Start TA, MCLK(DCO), Continuous
  while ((TACCTL0 & CCIFG) == 0);       // Wait until capture

  TACCR0 = 0;                           // Ignore first capture
  TACCTL0 &= ~CCIFG;                    // Clear CCIFG

  while ((TACCTL0 & CCIFG) == 0);       // Wait for next capture

  First_Cap = TACCR0;                   // Save first capture
  TACCTL0 &= ~CCIFG;                    // Clear CCIFG

  while ((TACCTL0 & CCIFG) ==0);        // Wait for next capture

  counts = (TACCR0 - First_Cap);        // # of VLO clocks in 8Mhz
  BCSCTL1 &= ~DIVA_3;                   // Clear ACLK/8 settings
  return counts;
}

// Timer A0 interrupt service routine
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A0 (void)
{
  LPM3_EXIT;
}

