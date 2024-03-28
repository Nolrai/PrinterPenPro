#include <msp430.h>

#define GYRO                    0x01 // Gyro device flag
#define TMC5160                 0x02 // Motor Controller flag

#define TMC5160_REG_STATUS      0x01 // STATUS register address
#define TMC5160_WRITE_MASK   0x80 // Read command mask

#define GYRO_READ_MASK      0x80 // Read command mask
#define GYRO_WHO_AM_I           0x0F // Register name from datasheet
#define GYRO_WHO_AM_I_EXPECTED  0x69 // Who am i result.

void spi_init() {

    //G2553
    P1DIR = 0x1D | BIT6;
    P1SEL = 0x16;
    P1SEL2 = 0x16;

    //USI Control and Config
    //

    // Configure UCA0 for SPI
    //Clock Polarity: The inactive state is high
    //MSB First, 8-bit, Master, 3-pin mode, Synchronous
    UCA0CTL0 |= UCCKPL + UCMSB + UCMST + UCSYNC;
    UCA0CTL1 |= UCSSEL_2;                     // SMCLK
    UCA0BR0 |= 0x20;                          // /2
    UCA0BR1 = 0;                              //
    UCA0MCTL = 0;                             // No modulation must be cleared for SPI
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine** // Release UCA0 from reset
    IE2 |= UCA0RXIE;                          // Enable USCI0 RX interrupt
}

// This just handles using the UCA0 for 2 byte command. It doesn't know which chip its talking too, or what any of the addresses mean,
// or even which direction we care about.
void spi_chunk(unsigned char address, unsigned char* data) {

    while (!(IFG2 & UCA0TXIFG)); //Do we want to sleep instead of busy waiting?
    UCA0TXBUF = address;                        // Load address into TX buffer
    while (!(IFG2 & UCA0TXIFG)); //Do we want to sleep instead of busy waiting?

    UCA0TXBUF = *data;                          // Load data into TX buffer
    while (!(IFG2 & UCA0RXIFG)); //Do we want to sleep instead of busy waiting?

    *data = UCA0RXBUF; // read data from the RX buffer
}

inline
void select_gyro () {
    P1OUT &= ~BIT0; // Set Gyro CS low
    P1OUT |= BIT3; // Make sure Motor CS is high
}

inline
void select_motor () {
    P1OUT |= BIT0; // Make sure Gyro CS is high
    P1OUT &= ~BIT3; // Set Motor CS low
}

inline
void deselect_io () {
    // Set both CS pins high
    P1OUT |= BIT0 | BIT3;
}

unsigned char gyro_who_am_i() {
    unsigned char ret = 0;

    select_gyro ();
    spi_chunk(GYRO_WHO_AM_I | GYRO_READ_MASK, &ret);
    deselect_io ();

    return ret;
}

const unsigned char FIFO_CTRL = 0x06;

typedef struct {
    unsigned waterline:12; // An interrupt is raised when this many points are in the FIFO.
    unsigned reserved0:2;
    unsigned pedo_flags:2;
    unsigned acc_decimation:3;
    unsigned gyro_decimation:3;
    unsigned reserved1:2;
    unsigned third_set_decimation:3;
    unsigned fourth_set_decimation:3;
    unsigned high_byte_only:1;
    unsigned reserved2:1;
    unsigned fifo_mode:3;
    unsigned fifo_odr:4;
    unsigned reserved3:1;
} fifo_ctrl_block;

typedef enum {
    DISCARD_ALL = 0,
    KEEP_ALL = 1,
    KEEP_HALF = 2,
    KEEP_THIRD = 3,
    KEEP_FORTH = 4,
    KEEP_EIGHT = 5,
    KEEP_SIXTEENTH = 6,
    KEEP_THIRTY_SECOND = 7
} decimation;

typedef enum {
    BOTH_BYTES = 0,
    HIGH_ONLY = 1,
} high_byte_only;

typedef enum {
  BYPASS_MODE = 0,
  STOP_WHEN_FULL = 1,
  STOP_AFTER_TRIGGER = 3, //N.B. this will always fill the buffer before stopping!
  START_ON_TRIGGER = 4,
  DISCARD_OLD = 5,
} fifo_mode;

typedef enum {
    ODR_DISABLED = 0,
    HZ_12_5 = 1,
    HZ_26,
    HZ_52,
    HZ_104,
    HZ_208,
    HZ_416,
    HZ_833,
    KHZ_1_33,
    KHZ_3_33,
    KHZ_6_66
} ODR; //Samples per second. 0 means don't collect data.

static const fifo_ctrl_block fifo_default_values = {
   .waterline = 0x0FFF,
   .acc_decimation = KEEP_ALL, // save all data points
   .gyro_decimation = KEEP_ALL, // save all data points
   .third_set_decimation = DISCARD_ALL, // discard all data points
   .fourth_set_decimation = DISCARD_ALL, // discard all data points
   .high_byte_only = BOTH_BYTES, //save high and low byte
   .fifo_mode = BYPASS_MODE, // Don't collect data yet.
   .fifo_odr = HZ_416
};

void spi_gyro_multibyte(unsigned char address, unsigned char* data, unsigned int length) {
    select_gyro ();

    // first 8 bits
    while (!(IFG2 & UCA0TXIFG)); //Do we want to sleep instead of busy waiting?
    UCA0TXBUF = address;         // Load address into TX buffer

    volatile unsigned char discard; //To force the reading of the RX buffer even when we aren't using it.
    unsigned int i;
    for (i=0; i < length; i++) {

        while (!(IFG2 & UCA0TXIFG)); //Do we want to sleep instead of busy waiting?
        if (0 == (address & GYRO_READ_MASK)) {
            UCA0TXBUF = data[i];           // Load data into TX buffer
        } else {
            UCA0TXBUF = 0; // Load dummy value into TX buffer to trigger clock pulses.
        }

        while (!(IFG2 & UCA0RXIFG)); //Do we want to sleep instead of busy waiting?
        if (0 == (address & GYRO_READ_MASK)) {
            discard = UCA0RXBUF; // read data from the RX buffer
        } else {
            data[i] = UCA0RXBUF; // read data from the RX buffer
        }
    }

    deselect_io();
}

void set_fifo_settings (fifo_mode* settings) {
    spi_gyro_multibyte(FIFO_CTRL, (unsigned char*)settings, sizeof(fifo_ctrl_block));
}

void read_fifo_settings (fifo_mode* settings) {
    spi_gyro_multibyte(FIFO_CTRL | GYRO_READ_MASK, (unsigned char*)settings, sizeof(fifo_ctrl_block));
}


void main(void) {
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

    // Don't waste current on unused pins
    P2DIR = 0xFF;
    P2OUT = 0x00;
    P3DIR = 0xFF;
    P3OUT = 0x00;

    if (CALBC1_16MHZ==0xFF) {                 // If calibration constant erased
            while(1);                               // do not load, trap CPU!!
    }
    DCOCTL = 0;                               // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_16MHZ;                    // Set DCO
    DCOCTL = CALDCO_16MHZ;

    spi_init(); // Initialize SPI

    P1OUT &= ~BIT6; //Set red LED off
    volatile char Result = 0;
    while (1) {
        Result = gyro_who_am_i();
        if (Result < 0xFF) {
            P1OUT |= BIT6; // if we got the good result turn on LED D2 (red).
        }
    }
}
