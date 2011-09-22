#include <signal.h>
#include <stdint.h>
#include <msp430x20x2.h>
//#include <msp430x21x1.h>

/* 
2011.09.12

 . add h/w spi via "#define USE_HW_SPI"
 . merge g2553 h/w spi code, not tested

2011.09.02 noon

 finally got this built under CCS
 . only tested w/ g2231 on launchpad at 1Mhz
 . moved some variable around
 . modify CCS build options
   . in build property .. compiler .. basic options, 
     have optimization level set to 4
	 speed size trade offs use 0 (opt for size)
   . in build propery .. compiler .. language option,
     check "Enable support for GCC extensions (--gcc)"
   . in build property .. linker .. basic options, 
     change "Set C system stack size (--stack_size)" from 60 to 50
   . i had tried it on 1Mhz only

2011.09.02

 this is example code to excercise basic jeeNodeJr library.

 . to use jeeNodeJr and accompanying uart functions,
   include rfm.h, uart.h in your source file
 . the current lib organization is thru header file inclusions and
   will only work on simple one source file projects. the intention
   is to minimize code space needed.
 . the project can only be build (at the moment) under mspgcc, this
   includes ubuntu based and window cygwin based
 . codes are TI ccs ready (syntax is good) but it always comes out
   too big for 2k devices, even if i take out the uart, so i will
   have to try it some other time (me not good at windows/CCS).

 uart.h

 . based on http://www.msp430launchpad.com/2010/08/half-duplex-software-uart-on-launchpad.html
 . based on http://blog.hodgepig.org/2010/09/09/573
 . code "stolen" from www.msp430launchpad.com
 . massaged at bit for
   . resolve issues when receiving consecutive characters
   . adjust "half-bit" timing so works on higher clocks

 rfm.h

 . based on 2009-02-09 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
 . see header for pin assignment, etc.
 . implement basic jeeNode functions
 . * receive buffer is 32 instead of 64 (in original jeeNode)
   g2231 has only 128 bytes of RAM
 . work on MCU clocks of 1, 4, 8, 12 Mhz (16Mhz not working)
 . u can see what's implemented and what's not
 . target is to at least also get the "easy" functions available


 tester program usage

 . flash same firmware to two ti launchpads (or launchpad + breadboarded g2211 setup)
 . pick one up as host, connect to pc, find out com port (com2x:)
 . connect w/ putty or termite or something at 9600N81
   may requires unplug / replug a few times in windows (linux looks better)
 . launchpad will not send anything to pc via uart until 1st receving something
 . press '=' to issue resets (to rfm12)
 . any other keys got loaded into buffer
 . press 'enter' to send buffer to slave device
 . if u do "+pingback<Enter>", u are asking the slave (w/ the '+')
   to send back the message every 2 ticks
 . u should see "pickback" showing up / received around 2 ticks
 * if things appears not to work, try issuing '=' to reset device
 * many times problem is with connections / wirings
 * change your rf12_init() for correct module, i.e. 433,866,915,etc

 different MCU clock tested
 . set FCPU to 1000000, 4000000, 8000000, or 12000000
 . ** requires calibrated values in flash **
 . i setup a 1Mhz FCPU slave and
 . have host run at 1,4,8,12Mhz to talk to this 1Mhz slave successfully
 . tried 16Mhz not working, even uart not working, may be my chip is not calibrated well


*/



//______________________________________________________________________
#define ___use_cal_clk(x)	\
BCSCTL1 = CALBC1_##x##MHZ;	\
DCOCTL  = CALDCO_##x##MHZ;

#define __use_cal_clk(x)	___use_cal_clk(x)

#define MHZ     1			// can be 1,4,8,12
#define FCPU    MHZ*1000000
#define USEC    MHZ

//______________________________________________________________________
#ifdef MSP430		// tells us we are using mspgcc
static void __inline__ __delay_cycles(register uint16_t n) {
//static void __inline__ brief_pause(register uint16_t n) {
    __asm__ __volatile__ (
                "1: \n"
                " dec      %[n] \n"
                " jne      1b \n"
        : [n] "+r"(n));

}
#else
#endif

#define led_off()       P1OUT &= ~BIT0
#define led_on()        P1OUT |= BIT0
#define led_flip()      P1OUT ^= BIT0


#define HOST_UART

#ifdef HOST_UART

	#include "uart.h"

#else

	#define uart_init()                     //na
	#define uart_getc(a)            0
	#define uart_putc(a)            //na
	#define uart_puts(a)            //na
	#define uart_puthex8(a)         //na
	#define uart_puthex16(a)        //na
	#define RXD                             0
	#define bitCount                0
	#define uart_recv_int()         asm("nop")

#endif

#define USE_HW_SPI
#include "rfm.h"

static volatile uint8_t ticks = 0;

//______________________________________________________________________
int main(void) {

        WDTCTL = WDTPW + WDTHOLD; // Stop WDT

#if MHZ == 4
        BCSCTL1 = CALBC1_8MHZ | DIVA_1;
        DCOCTL  = CALDCO_8MHZ;
        BCSCTL2 |= DIVM_1 | DIVS_1;
#else
		__use_cal_clk(MHZ);
#endif

        _BIC_SR(GIE);

        P2SEL = 0x00;
        P1DIR = BIT0;

        led_on();

		uart_init();
        rf12_initialize(2, RF12_915MHZ, 33);

        led_off();

        //_______________ watchdog 12khz VLO instead of 32khz crystal
        //                wdt used for event timing, may be easy transmits in the future
        BCSCTL3 |= LFXT1S_2;
        //WDTCTL = WDT_ADLY_1000;       // more like 2.6 sec
        WDTCTL = WDT_ADLY_250;          // like 250ms / 13 * 32, or around 2/3 of a second
        //WDTCTL = WDT_ADLY_16;
        //WDTCTL = WDT_ADLY_1_9;
        IE1 |= WDTIE;

        _BIS_SR(GIE);

        uint8_t needToSend = 0;
        uint8_t autoSend = 0;

        static uint8_t payload_len = 0;
        static char payload[12] = "";


		while (1) {
				//__delay_cycles(50*USEC);
                //if (rf12_recvDone()) {
                if (rf12_recvDone() && rf12_crc == 0) {
                        rf12_data[rf12_len] = '\0';
                        uart_puts((const char*) rf12_data);
                        uart_putc('\r'); uart_putc('\n');
                        if (rf12_data[0] == '+') {
                                // we were asked to send back data at intervals
                                payload_len = rf12_len;
                                uint8_t i;
                                for (i=1;i<rf12_len;i++)
                                        payload[i] = rf12_data[i];
                                payload[0] = ' ';                // take away 1st char of '+', we are just replying
                                autoSend = 2;                   // next send should be in 2 ticks
                        }//if
                        else {
                                autoSend = 0;
                        }//else
					//rf12_recvStart();
                }//if
                if (needToSend && rf12_canSend()) {
                        needToSend = 0;
                        rf12_sendStart(0, payload, payload_len);
                        if (!autoSend) payload_len = 0;
                }//if
                uint8_t c = 0;
                if (uart_getc(&c)) {
                        switch (c) {
                                case '=':  // reset
                                        rf12_initialize(2, RF12_915MHZ, 33);
                                        break;
                                case ' ':  // status
                                        uart_puthex16(rf12_xfer(0x0000));
										uart_putc('\r'); uart_putc('\n');
                                        break;
                                default:
                                        if (c > 0x20) {
                                                payload[payload_len++] = c;
                                                uart_putc(c);
                                        }//if
                                        break;
                        }//switch
                        if (c == 0x0d || payload_len >= 12) {
                                uart_putc('\r'); uart_putc('\n');
                                needToSend = 1;
                        }//if
                }//if

                if (autoSend && ticks >= autoSend) {
                        ticks = 0;
                        needToSend = 1;
                }//if

        }//while
}

//______________________________________________________________________
#ifdef MSP430
interrupt(WDT_VECTOR) WDT_ISR(void)
#else
#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
#endif
{
        ticks++;
}

//______________________________________________________________________
#ifdef MSP430
interrupt(PORT1_VECTOR) PORT1_ISR(void)
#else
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void)
#endif
{
        _BIC_SR(GIE);
        led_on();
        if (!bitCount && (P1IFG & IRQ)) {
                rf12_interrupt();
        }//if
        else {
                if (P1IFG & RXD) uart_recv_int();
        }//else
        P1IFG = 0x00;
        led_off();
        _BIS_SR(GIE);
}

