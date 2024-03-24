#include <msp430.h>

#define GYRO                    0x01 // Gyro device flag
#define TMC5160                 0x02 // Motor Controller flag

#define TMC5160_REG_STATUS      0x01 // STATUS register address
#define TMC5160_SPI_READ_MASK   0x80 // Read command mask

#define GYRO_SPI_READ_MASK      0x80 // Read command mask
#define GYRO_WHO_AM_I           0x0F // Reg name from datasheet
#define GYRO_WHO_AM_I_EXPECTED  0x96 // Fixed Who am i result.


void SPI_init() {

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
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    IE2 |= UCA0RXIE;                          // Enable USCI0 RX interrupt
    UCA0CTL1 &= ~UCSWRST;                  // Release UCA0 from reset
}

// This just handles using the UCA0 for 2 byte command. It doesn't know which chip its talking too, or what any of the addresses mean,
// or even which direction we care about.
unsigned char spi_chunk(unsigned char address, unsigned char* data) {

    while (!(IFG2 & UCA0TXIFG)); //Do we want to sleep instead of busy waiting?
    UCA0TXBUF = address;                        // Load address into TX buffer
    while (!(IFG2 & UCA0TXIFG)); //Do we want to sleep instead of busy waiting?

    UCA0TXBUF = *data;                          // Load data into TX buffer
    while (!(IFG2 & UCA0RXIFG)); //Do we want to sleep instead of busy waiting?

    *data = UCA0RXBUF; // read data from the RX buffer
    return 0;
}

void select_gyro () {
    P1OUT &= ~BIT0; // Set Gyro CS low
    P1OUT |= BIT3; // Set Motor CS high
}

void select_motor () {
    P1OUT |= BIT0; // Set Gyro CS high
    P1OUT &= ~BIT3; // Set Motor CS low
}

void deselect_io () {
    P1OUT |= BIT0 | BIT3;
}


unsigned char gyro_who_am_i() {
    unsigned char ret = 0;

    select_gyro ();
    spi_chunk(GYRO_WHO_AM_I | GYRO_SPI_READ_MASK, &ret);
//    deselect_io ();

    return ret;
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

    SPI_init(); // Initialize SPI

    P1OUT &= ~BIT6; //Set red LED off
    while (1) {

        if (0 != Result) {
            P1OUT ^= BIT6; // if we got the good result toggle LED D2 (red).
        }

    }
}
