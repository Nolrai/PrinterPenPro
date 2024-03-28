#include <msp430.h>

#define GYRO                    0x01 // Gyro device flag
#define TMC5160                 0x02 // Motor Controller flag
#define GYRO_READ               0x03
#define TMC5160_READ    `       0x04

#define TMC5160_REG_STATUS      0x01 // STATUS register address
#define TMC5160_SPI_READ_MASK   0x80 // Read command mask

#define GYRO_SPI_READ_MASK      0x80 // Read command mask
#define GYRO_WHO_AM_I           0x0F // Reg name from datasheet


void SPI_init() {

    //G2553
    P1DIR = 0x3D;
    P1SEL = 0x16;
    P1SEL2 = 0x16;
    //deselect both chips
    P1OUT |= BIT0 + BIT3;

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

void adc_Init(){
   //write all zeros to reset ENC
    ADC10CTL0 = 0x00;
    ADC10CTL0 = 0x1010;
    ADC10CTL1 = 0xA000;
    ADC10AE0 = 0x0000;//sets input to temp sensor
}

unsigned char SPI_transfer(unsigned char add, unsigned char data, unsigned int DevID) {

    // if called before transfer complete add data to buffer or?

    if(DevID == GYRO_READ){
        P1OUT &= ~BIT0; // Set CS low

        while (!(IFG2 & UCA0TXIFG));
        UCA0TXBUF = 0x8F;                     // Load data into TX buffer
        while (!(IFG2 & UCA0TXIFG));

        UCA0TXBUF = 0x55;
        while (UCA0STAT & UCBUSY);
        P1OUT |= BIT0; // Set CS high
    }
    return UCA0RXBUF;
}

const SUCCESS = 0;

unsigned long SPI_transferLong(short int addLong,long int dataLong){
    P1OUT &= ~BIT3; // Set CS low
    //if(Dev == TMC5160_READ){
        //0x00 - 0x0F are setup registers
        while (!(IFG2 & UCA0TXIFG));
        UCA0TXBUF = 0x80;                     // Load data into TX buffer Motor cobtroller gives a status word after first 8 bits are sent.
        while (!(IFG2 & UCA0TXIFG));
        UCA0TXBUF = 0x00;                     // Load data into TX buffer. this word should clock MISO data
        while (!(IFG2 & UCA0TXIFG));
        UCA0TXBUF = 0x00;                     // will be write data
        while (!(IFG2 & UCA0TXIFG));
        UCA0TXBUF = 0x00;                     // 32bits
        while (!(IFG2 & UCA0TXIFG));
        UCA0TXBUF = 0x00;                     // last of clock  pulses
        while (UCA0STAT & UCBUSY);
        P1OUT |= BIT3; // Set CS high

    //}
    return SUCCESS;
}

//adc init
//enc =0
//Internal ref off, use vcc
//CONSEQx = 00b



void main(void) {
    volatile unsigned char Result = 0;
    volatile unsigned char MtrRb = 0;
    volatile unsigned int TempRb = 0;
//    int p = 6000;
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

    if (CALBC1_16MHZ==0xFF) {                 // If calibration constant erased
            while(1);                               // do not load, trap CPU!!
    }
    DCOCTL = 0;                               // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_16MHZ;                    // Set DCO
    DCOCTL = CALDCO_16MHZ;



    SPI_init(); // Initialize SPI
    adc_Init();

    while (1) {
        Result = SPI_transfer(GYRO_WHO_AM_I, GYRO, GYRO_READ);
        if (0x69 != Result) continue;
        MtrRb =  SPI_transferLong(0,0);
        if (SUCCESS != MtrRb ) continue;
        //Start conversion
        ADC10CTL0 |= ENC + ADC10SC;//enable and start
        __delay_cycles(1000000);
        while(ADC10CTL1 & ADC10BUSY);
        TempRb = ADC10MEM;

    }
}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
    //INterrupt on int flag from gyro
    //read gyro values
    //start adc conversion

}
