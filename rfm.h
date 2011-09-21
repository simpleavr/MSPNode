// RFM12B driver implementation
// 2009-02-09 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// 2011-09-19 <simpleavr@gmail.com> http://opensource.org/licenses/mit-license.php

/*
2011.09.19
. jeelabsandy advice typo on h/w spi for G2553 devices
  . should use both UCB0TXBUF, UCB0RXBUF register for spi
  . changed accordingly

   rfm12b layout, this is the smd version, rectangular box is the crystal

               SDO + o--+     o + nSEL
              nIRQ + o  |     o + SCK
     FSK/DATA/nFFS + x  |     o + SDI
    DCLK/CFIL/FFIT + x  |     x + nINT/VDI
               CLK + x--+     o + GND 
              nRES + x        o + VDD
               GND + o        o + ANT 

*/


#ifdef USE_HW_SPI

	// hardware spi mode
	// sdi, sdo refers to msp430 device's pins, NOT rfm12b's
	#define SCK     BIT5		// to SCK
	#define SDI     BIT7		// to SDO
	#define SDO     BIT6		// to SDI
	#define SEL     BIT4		// to nSEL
	#define IRQ		BIT3		// to nIRQ

#else

	// bit-bang mode
	// sdi, sdo refers to rfm12b's sdi, sdo pins
	/*
	#define SCK 	BIT7		// to SCK
	#define SDI 	BIT3		// to SDO
	#define SDO 	BIT6		// to SDI
	#define SEL 	BIT5		// to nSel
	#define IRQ		BIT4		// to nIRQ
	*/
	#define SCK     BIT5		// to SCK
	#define SDI     BIT7		// to SDO
	#define SDO     BIT6		// to SDI
	#define SEL     BIT4		// to nSEL
	#define IRQ		BIT3		// to nIRQ

#endif


uint8_t rf12_config(uint8_t show);
uint8_t rf12_recvDone(void);
uint8_t rf12_canSend(void);

#ifdef __cplusplus
void rf12_initialize(uint8_t id, uint8_t band, uint8_t group=0xd4);
#else
void rf12_initialize(uint8_t id, uint8_t band, uint8_t group);
#endif

void rf12_sendStart_det(uint8_t hdr);
void rf12_sendStart(uint8_t hdr, const void* ptr, uint8_t len);
void rf12_sendWait (uint8_t mode);
void rf12_onOff(uint8_t value);
void rf12_sleep(char n);
char rf12_lowbat(void);
void rf12_easyInit(uint8_t secs);
char rf12_easyPoll(void);
char rf12_easySend(const void* data, uint8_t size);
void rf12_encrypt(const uint8_t*);

uint16_t rf12_control(uint16_t cmd);


// version 1 did not include the group code in the crc
// version 2 does include the group code in the crc
#define RF12_VERSION    2

#define rf12_grp        rf12_buf[0]
#define rf12_hdr        rf12_buf[1]
#define rf12_len        rf12_buf[2]
#define rf12_data       (rf12_buf + 3)

#define RF12_HDR_CTL    0x80
#define RF12_HDR_DST    0x40
#define RF12_HDR_ACK    0x20
#define RF12_HDR_MASK   0x1F

//#define RF12_MAXDATA    66
// limit to 32 bytes packet
#define RF12_MAXDATA    33

#define RF12_433MHZ     1
#define RF12_868MHZ     2
#define RF12_915MHZ     3

// EEPROM address range used by the rf12_config() code
#define RF12_EEPROM_ADDR ((uint8_t*) 0x20)
#define RF12_EEPROM_SIZE 32
#define RF12_EEPROM_EKEY (RF12_EEPROM_ADDR + RF12_EEPROM_SIZE)
#define RF12_EEPROM_ELEN 16

// shorthand to simplify sending out the proper ACK when requested
#define RF12_WANTS_ACK ((rf12_hdr & RF12_HDR_ACK) && !(rf12_hdr & RF12_HDR_CTL))
#define RF12_ACK_REPLY (rf12_hdr & RF12_HDR_DST ? RF12_HDR_CTL : \
            RF12_HDR_CTL | RF12_HDR_DST | (rf12_hdr & RF12_HDR_MASK))
            
// options fro RF12_sleep()
#define RF12_SLEEP 0
#define RF12_WAKEUP -1
// RF12 command codes
#define RF_RECEIVER_ON  0x82DD
#define RF_XMITTER_ON   0x823D
#define RF_IDLE_MODE    0x820D
#define RF_SLEEP_MODE   0x8205
#define RF_WAKEUP_MODE  0x8207
#define RF_TXREG_WRITE  0xB800
#define RF_RX_FIFO_READ 0xB000
#define RF_WAKEUP_TIMER 0xE000

// RF12 status bits
#define RF_LBD_BIT      0x0400
#define RF_RSSI_BIT     0x0100

// bits in the node id configuration byte
#define NODE_BAND       0xC0        // frequency band
#define NODE_ACKANY     0x20        // ack on broadcast packets if set
#define NODE_ID         0x1F        // id of this node, as A..Z or 1..31

// transceiver states, these determine what to do with each interrupt
enum {
    TXCRC1, TXCRC2, TXTAIL, TXDONE, TXIDLE,
    TXRECV,
    TXPRE1, TXPRE2, TXPRE3, TXSYN1, TXSYN2,
};

static uint8_t nodeid;              // address of this node
static uint8_t group;               // network group
static volatile uint8_t rxfill;     // number of data bytes in rf12_buf
static volatile int8_t rxstate;     // current transceiver state

#define RETRIES     8               // stop retrying after 8 times
#define RETRY_MS    1000            // resend packet every second until ack'ed

// maximum transmit / receive buffer: 3 header + data + 2 crc bytes
#define RF_MAX   (RF12_MAXDATA + 5)

#ifdef EASY_FUNCS
static uint8_t ezInterval;          // number of seconds between transmits
static uint8_t ezSendBuf[RF12_MAXDATA]; // data to send
static char ezSendLen;              // number of bytes to send
static uint8_t ezPending;           // remaining number of retries
static long ezNextSend[2];          // when was last retry [0] or data [1] sent
#endif

static volatile uint16_t rf12_crc;         // running crc value
static volatile uint8_t rf12_buf[RF_MAX];  // recv/xmit buf, including hdr & crc bytes
//long rf12_seq;                      // seq number of encrypted packet (or -1)

//static uint32_t seqNum;             // encrypted send sequence number
//static uint32_t cryptKey[4];        // encryption key to use
//void (*crypter)(uint8_t);           // does en-/decryption (null if disabled)

//__________________________________________________________
uint16_t _crc16_update(uint16_t crc, uint8_t a) {
	int i;
	crc ^= a;
	for (i=0;i<8;++i) {
		if (crc&0x0001)
			crc = (crc>>1) ^ 0xA001;
		else
			crc = (crc>>1);
	}//for
	return crc;
}


#ifdef USE_HW_SPI


//______________________________________________________________________
uint16_t rf12_xfer(uint16_t c) {
	uint16_t res;

	P1OUT &= ~(SEL);

#ifdef UCB0RXBUF
    UCB0TXBUF = c>>8;
    while ((UCB0STAT & UCBUSY) == 0x01);
    res = UCB0TXBUF<<8;
    while ((UCB0STAT & UCBUSY) == 0x01);
    UCB0RXBUF = c;
    while ((UCB0STAT & UCBUSY) == 0x01);
    res |= UCB0RXBUF;
#else
    USISRL = c>>8;
    USICTL1 &= ~USIIFG;
    USICNT = 8;

    while ((USICTL1 & USIIFG) != 0x01);
    res = USISRL<<8;

    USISRL = c;
    USICTL1 &= ~USIIFG;
    USICNT = 8;

    while ((USICTL1 & USIIFG) != 0x01);
    res |= USISRL;
#endif

	P1OUT |= (SEL);
    return res;
}

//______________________________________________________________________
// assuming 1Mhz DCO
void rf12_port_init(void) {
    P1DIR |= SCK | SDO | SEL;
    P1DIR &= ~SDI;

    // enable SDI, SDO, SCLK, master mode, MSB, output enabled, hold in reset
    USICTL0 = USIPE7 | USIPE6 | USIPE5 | USIMST | USIOE | USISWRST;

    // SMCLK / 128
    //USICKCTL = USIDIV_7 + USISSEL_2;
    USICKCTL = USIDIV_4 + USISSEL_2;

    // clock phase
    USICTL1 |= USICKPH;

    // release from reset
    USICTL0 &= ~USISWRST;
}
#else
//______________________________________________________________________
uint16_t rf12_xfer(uint16_t cmd) {
	uint8_t  i;
	uint16_t res=0;

	P1OUT &= ~(SCK);
	P1OUT &= ~(SEL);
	for (i=0;i<16;i++) {
		if (cmd & 0x8000)
			P1OUT |= (SDO);
		else
			P1OUT &= ~(SDO);
		// looks like we don't need delays
		//__delay_cycles(5*USEC);
		P1OUT |= (SCK);
		res <<= 1;
		if (P1IN & (SDI)) res |= 0x0001;
		P1OUT &= ~(SCK);
		cmd <<= 1;
	}//for
	P1OUT |= (SEL);
	//P1OUT &= ~(SDO);		// DEB not really needed?
	return (res);
}

//______________________________________________________________________
void rf12_port_init(void) {
	P1OUT |= (SEL);
	P1DIR |= (SEL|SDO|SCK);
}
#endif


//______________________________________________________________________
void rf12_initialize(uint8_t id, uint8_t band, uint8_t g) {
    nodeid = id;
    group = g;

	rf12_port_init();
	//__delay_cycles(5000*USEC);

    rf12_xfer(0x0000); // intitial SPI transfer added to avoid power-up problem
    rf12_xfer(RF_SLEEP_MODE); // DC (disable clk pin), enable lbd

    // wait until RFM12B is out of power-up reset, this takes several *seconds*
    rf12_xfer(RF_TXREG_WRITE); // in case we're still in OOK mode
    while (!(P1IN & IRQ)) rf12_xfer(0x0000);
        
    rf12_xfer(0x80C7 | (band << 4)); // EL (ena TX), EF (ena RX FIFO), 12.0pF 

    rf12_xfer(0xA640); // center frequency
    //rf12_xfer(0xC606); // approx 49.2 Kbps, i.e. 10000/29/(1+6) Kbps
    rf12_xfer(0xC647); // 4.8kbps
    rf12_xfer(0x94A2); // VDI,FAST,134kHz,0dBm,-91dBm 
    rf12_xfer(0xC2AC); // AL,!ml,DIG,DQD4 
    if (group != 0) {
        rf12_xfer(0xCA83); // FIFO8,2-SYNC,!ff,DR 
        rf12_xfer(0xCE00 | group); // SYNC=2DXX； 
    }//if
	else {
        rf12_xfer(0xCA8B); // FIFO8,1-SYNC,!ff,DR 
        rf12_xfer(0xCE2D); // SYNC=2D； 
    }//else
    rf12_xfer(0xC483); // @PWR,NO RSTRIC,!st,!fi,OE,EN 
    rf12_xfer(0x9850); // !mp,90kHz,MAX OUT 
    rf12_xfer(0xCC77); // OB1，OB0, LPX,！ddy，DDIT，BW0 
    rf12_xfer(0xE000); // NOT USE 
    rf12_xfer(0xC800); // NOT USE 
    rf12_xfer(0xC049); // 1.66MHz,3.1V 

    rxstate = TXIDLE;
    P1IES |= IRQ;
    P1IFG &= ~IRQ;
    P1IE  |= IRQ;
}

//______________________________________________________________________
uint8_t rf12_byte(uint8_t byte) {
	while (P1IN & IRQ);
	return (rf12_xfer(0xB800 + byte) & 0xff);
}

//______________________________________________________________________
// access to the RFM12B internal registers with interrupts disabled
uint16_t rf12_control(uint16_t cmd) {
    P1IE &= ~IRQ;
	while (!(P1IN & IRQ));
    uint16_t r = rf12_xfer(cmd);
    P1IE |= IRQ;
    //P1IFG &= ~IRQ;
    return r;
}

//______________________________________________________________________
static void rf12_recvStart () {
    rxfill = rf12_len = 0;
    rf12_crc = ~0;
#if RF12_VERSION >= 2
    if (group != 0)
        rf12_crc = _crc16_update(~0, group);
#endif
    rxstate = TXRECV;    
    rf12_xfer(RF_RECEIVER_ON);
}

//______________________________________________________________________
uint8_t rf12_recvDone () {
	//nop(); nop();
    if (rxstate == TXRECV && (rxfill >= rf12_len + 5 || rxfill >= RF_MAX)) {
        rxstate = TXIDLE;
        if (rf12_len > RF12_MAXDATA)
            rf12_crc = 1; // force bad crc if packet length is invalid
        if (!(rf12_hdr & RF12_HDR_DST) || (nodeid & NODE_ID) == 31 ||
                (rf12_hdr & RF12_HDR_MASK) == (nodeid & NODE_ID)) {
			/*
            if (rf12_crc == 0 && crypter != 0)
                crypter(0);
            else
               rf12_seq = -1;
			*/
            return 1; // it's a broadcast packet or it's addressed to this node
        }
    }
    if (rxstate == TXIDLE)
        rf12_recvStart();
    return 0;
}

//______________________________________________________________________
uint8_t rf12_canSend () {
    // no need to test with interrupts disabled: state TXRECV is only reached
    // outside of ISR and we don't care if rxfill jumps from 0 to 1 here
    if (rxstate == TXRECV && rxfill == 0 &&
            (rf12_xfer(0x0000) & RF_RSSI_BIT) == 0) {
        rf12_xfer(RF_IDLE_MODE); // stop receiver
        //XXX just in case, don't know whether these RF12 reads are needed!
        // rf12_xfer(0x0000); // status register
        // rf12_xfer(RF_RX_FIFO_READ); // fifo read
        rxstate = TXIDLE;
        rf12_grp = group;
        return 1;
    }//if
    return 0;
}

//______________________________________________________________________
void rf12_sendStart_det(uint8_t hdr) {
    rf12_hdr = hdr & RF12_HDR_DST ? hdr :
                (hdr & ~RF12_HDR_MASK) + (nodeid & NODE_ID);
    //if (crypter != 0) crypter(1);
    
    rf12_crc = ~0;
#if RF12_VERSION >= 2
    rf12_crc = _crc16_update(rf12_crc, rf12_grp);
#endif
    rxstate = TXPRE1;
    rf12_xfer(RF_XMITTER_ON); // bytes will be fed via interrupts
}

//______________________________________________________________________
void rf12_memcpy(uint8_t *dp, uint8_t *sp, uint8_t sz) {
	uint8_t i;
	for (i=0;i<sz;i++) *dp++ = *sp++;
}

//______________________________________________________________________
int8_t rf12_memcmp(uint8_t *dp, uint8_t *sp, uint8_t sz) {
	uint8_t i;
	for (i=0;i<sz;i++) {
		if (dp[i] > sp[i]) return 1;
		if (dp[i] < sp[i]) return -1;
	}//for
	return 0;
}

//______________________________________________________________________
void rf12_sendStart(uint8_t hdr, const void* ptr, uint8_t len) {
    rf12_len = len;
    rf12_memcpy((uint8_t*) rf12_data, (uint8_t*) ptr, len);
    rf12_sendStart_det(hdr);
}

//interrupt(PORT1_VECTOR) PORT1_ISR(void)
// *** this pin interrupt must be setup and called from main program
//______________________________________________________________________
void rf12_interrupt(void) {
    // a transfer of 2x 16 bits @ 2 MHz over SPI takes 2x 8 us inside this ISR
	//uint8_t res = rf12_xfer(0x0000) >> 8;
    
    if (rxstate == TXRECV) {
		//rf12_xfer(0x0000);
        uint8_t in = rf12_xfer(RF_RX_FIFO_READ);

        if (rxfill == 0 && group != 0)
            rf12_buf[rxfill++] = group;
            
        rf12_buf[rxfill++] = in;

        rf12_crc = _crc16_update(rf12_crc, in);

        if ((rxfill >= (rf12_len + 5)) || (rxfill >= RF_MAX))
            rf12_xfer(RF_IDLE_MODE);
    }//if
	else {
        uint8_t out;

        if (rxstate < 0) {
            uint8_t pos = 3 + rf12_len + rxstate++;
            out = rf12_buf[pos];
            rf12_crc = _crc16_update(rf12_crc, out);
        }//if
		else {
            switch (rxstate++) {
                case TXSYN1: out = 0x2D; break;
                case TXSYN2: out = rf12_grp; rxstate = - (2 + rf12_len); break;
                case TXCRC1: out = rf12_crc; break;
                case TXCRC2: out = rf12_crc >> 8; break;
                case TXDONE: rf12_xfer(RF_IDLE_MODE); // fall through
                default:     out = 0xAA;
            }//switch
		}//else
            
        rf12_xfer(RF_TXREG_WRITE + out);
    }//else
}

// easy transmissions
#ifdef EASY_FUNCS
//______________________________________________________________________
void rf12_easyInit (uint8_t secs) {
    ezInterval = secs;
}

//______________________________________________________________________
char rf12_easyPoll () {
    if (rf12_recvDone() && rf12_crc == 0) {
        uint8_t myAddr = nodeid & RF12_HDR_MASK;
        if (rf12_hdr == (RF12_HDR_CTL | RF12_HDR_DST | myAddr)) {
            ezPending = 0;
            ezNextSend[0] = 0; // flags succesful packet send
            if (rf12_len > 0)
                return 1;
        }//if
    }//if
    if (ezPending > 0) {
        // new data sends should not happen less than ezInterval seconds apart
        // ... whereas retries should not happen less than RETRY_MS apart
        uint8_t newData = ezPending == RETRIES;
        //long now = millis();
        long now = 1234;
        if (now >= ezNextSend[newData] && rf12_canSend()) {
            ezNextSend[0] = now + RETRY_MS;
            // must send new data packets at least ezInterval seconds apart
            // ezInterval == 0 is a special case:
            //      for the 868 MHz band: enforce 1% max bandwidth constraint
            //      for other bands: use 100 msec, i.e. max 10 packets/second
            if (newData)
                ezNextSend[1] = now +
                    (ezInterval > 0 ? 1000L * ezInterval
                                    : (nodeid >> 6) == RF12_915MHZ ?
                                            13 * (ezSendLen + 10) : 100);
            rf12_sendStart(RF12_HDR_ACK, ezSendBuf, ezSendLen);
            --ezPending;
        }//if
    }//if
    return ezPending ? -1 : 0;
}

//______________________________________________________________________
char rf12_easySend (const void* data, uint8_t size) {
    if (data != 0 && size != 0) {
        if (ezNextSend[0] == 0 && size == ezSendLen &&
			rf12_memcmp(ezSendBuf, (uint8_t*) data, size) == 0)
            return 0;
        rf12_memcpy((uint8_t*) ezSendBuf, (uint8_t*) data, size);
        ezSendLen = size;
    }//if
    ezPending = RETRIES;
    return 1;
}

#endif
