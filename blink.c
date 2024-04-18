#include <msp430.h>				
#include <string.h>
#include <stdlib.h>

#define GYRO                    0x01 // Gyro device flag
#define TMC5160                 0x02 // Motor Controller flag

#define TMC5160_REG_STATUS      0x01 // STATUS register address
#define TMC5160_WRITE_MASK   0x80 // Read command mask

#define GYRO_READ_MASK      0x80 // Read command mask
#define GYRO_WHO_AM_I           0x0F // Register name from datasheet
#define GYRO_WHO_AM_I_EXPECTED  0x69 // Who am i result.


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

inline
void select_gyro () {
    P1OUT |= BIT3; // Deselect Motor by setting Motor CS HIGH
    P1OUT &= ~BIT0; // Select gyro by setting Gyro CS LOW
}

inline
void select_motor () {
    P1OUT |= BIT0; // Deselect Gyro by setting Motor CS HIGH
    P1OUT &= ~BIT3; // Select Motor by setting Gyro CS LOW
}

inline
void deselect_io () {
    P1OUT |= BIT0 | BIT3; //Set both chip selects HIGH (deselected)
}

void gyro_spi (unsigned char address, int size, const unsigned char* mosi, unsigned char* miso) {

    select_gyro ();

    while (!(IFG2 & UCA0TXIFG));
    UCA0TXBUF = address;                     // Load data into TX buffer

    unsigned int i;
    for(i = 0; i < size; i++) {

        while (!(IFG2 & UCA0TXIFG));
        UCA0TXBUF = mosi[i];
        while (UCA0STAT & UCBUSY);
        miso[i] = UCA0RXBUF;
    }

    deselect_io();
}

typedef struct {
    long whichByte;
    unsigned actual:8;
    unsigned expected:8;
} diff_location;

const diff_location NO_ERROR = {0,0,0};
const diff_location OTHER_ERROR = {-1,0,0};

diff_location gyro_write_and_confirm (unsigned char address, volatile int size, const unsigned char* mosi) {
    unsigned char* readback = malloc(size);
    diff_location ret = NO_ERROR;

    if(0 == size) {
        exit(-1);
    }

    gyro_spi (address & ~GYRO_READ_MASK, size, mosi, readback); // write data to gyro
    _delay_cycles (1000); // wait for gyro to settle
    gyro_spi (address | GYRO_READ_MASK, size, mosi, readback); //read data back from gyro

    unsigned int i;
    for (i = size - 1; i != 0; i--) {
        if (readback[i] != mosi[i]) {
            ret.whichByte |= 1L << i;
            ret.actual = readback[i];
            ret.expected = mosi[i];
        }
    }

    free(readback);

    if (0 == size) {
        exit(-1);
    }
    return ret;
}
  // 00000000000000011110100000000000b
const unsigned char FIFO_CTRL = 0x06;
typedef struct fifo_ctrl_block {
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

const fifo_ctrl_block fifo_values = {
   .waterline = 0x0FFF,
   .acc_decimation = KEEP_ALL, // save all data points
   .gyro_decimation = KEEP_ALL, // save all data points
   .third_set_decimation = DISCARD_ALL, // discard all data points
   .fourth_set_decimation = DISCARD_ALL, // discard all data points
   .high_byte_only = BOTH_BYTES, //save high and low byte
   .fifo_mode = BYPASS_MODE, // Don't collect data yet.
   .fifo_odr = HZ_416
};

unsigned char gyro_read_single_byte (unsigned char address) {
    unsigned char mosi = 0x55;
    unsigned char miso = 0x55;
    gyro_spi(address | GYRO_READ_MASK, 1, &mosi, &miso);
    return miso;
}

diff_location gyro_set_fifo_to_default () {
    return gyro_write_and_confirm(FIFO_CTRL, sizeof(fifo_ctrl_block), (const unsigned char*)&fifo_values);
}

const unsigned char INT_CTRL = 0x0D;
const unsigned char INT1_FTH = BIT3;
const unsigned char INT1_DRDY_G = BIT1;
const unsigned char INT1_DRDY_XL = BIT0;

diff_location gyro_set_interupt_ctrl(unsigned char int1_mask, unsigned char int2_mask) {
    char data[2] = {int1_mask,  int2_mask};
    return gyro_write_and_confirm (INT_CTRL, 2, (void*)data);
}

typedef enum {
  DPS_250 = 0b00,
  DPS_500 = 0b01,
  DPS_1000 = 0b10,
  DPS_2000 = 0b11
} gyro_scale;

// Main control block (registers 0x10 - 0x1A inclusive)
const unsigned char CTR_BLOCK = 0x10;

// these are grouped by byte, not by function. So some controls are in sub-structs whose names don't seem to fit.
// I think the "__attribute__((__packed__))" isn't needed because all the members are bit fields, but I am not sure.

// CTRL1_XL
struct __attribute__((__packed__)) accel_data_rate_ctrl {
    //byte 0 -- CTRL1_XL
        unsigned accel_bandwidth:2;
        unsigned accel_scale:2;
        unsigned accel_data_rate:4;
};

// CTRL2_G
struct __attribute__((__packed__)) gyro_data_rate_ctrl {
    //byte 1 -- CTRL2_G
    unsigned reserved:1;
    unsigned gyro_scale:3;
    unsigned gyro_data_rate:4;
};

// CTRL3_C
struct __attribute__((__packed__)) comms_ctrl {
    unsigned software_reset:1;
    unsigned endian_data_selection:1;
    unsigned enable_multi_byte_read:1;
    unsigned spi_wires_mode:1;
    unsigned push_pull_or_open_drain:1;
    unsigned interrupt_polarity:1;
    unsigned block_data_update:1;
    unsigned boot:1;
};

// CTRL4_C
struct __attribute__((__packed__)) ctrl_misc {
    unsigned stop_on_fifo_threshold:1;
    unsigned reserved:1;
    unsigned disable_i2c:1;
    unsigned data_read_mask:1;
    unsigned enable_temp_fifo:1;
    unsigned rout_int2_signals_to_int1:1;
    unsigned gyros_enable_sleep:1;
    unsigned accel_enable_bandwidth_selection:2;
};

// CTRL5_C
struct __attribute__((__packed__)) rounding_and_self_test {
    unsigned accel_self_test:2;
    unsigned gyro_self_test:2;
    unsigned reserved:1;
    unsigned include_accel;
    unsigned include_gyro;
    unsigned include_hub; //Note if accel is off and gyro and hub are both on, instead both hubs 1 and 2 and the accel and the gyro are included in the rotation.
};

// CTRL6_C
struct __attribute__((__packed__)) hp_and_triggers {
    unsigned reserved:4;
    unsigned accel_dsable_high_performance:1;
    unsigned gyro_enable_level_sensitive_trigger:1;
    unsigned gyro_enable_level_sensitive_latch:1;
    unsigned gyro_enable_edge_sensitive_trigger:1;
};

// CTRL7_G
struct __attribute__((__packed__)) gyro_filters {
    unsigned reserved:2;
    unsigned enable_rounding_function:1;
    unsigned reset_highpass_filter:1;
    unsigned highpass_filter_frequency:2;
    unsigned enable_highpass_filter:1;
    unsigned disable_high_performance:1;
};

// CTRL8_XL
struct __attribute__((__packed__)) accel_filters {
    unsigned enable_orientation_lowpass_filter:1;
    unsigned reserved0:1;
    unsigned enable_filters:1;
    unsigned reserved1:2;
    unsigned slope_highpass_cutoff:2;
    unsigned lowpass_selection:1;
};

// CTRL9_XL
struct __attribute__((__packed__)) accel_enable {
    unsigned reserved0:2;
    unsigned enable_soft_iron_correction:1;
    unsigned accel_enable_x_axis:1;
    unsigned accel_enable_y_axis:1;
    unsigned accel_enable_z_axis:1;
    unsigned reserved1:2;
};

// CTRL10_C
struct __attribute__((__packed__)) gyro_enable {
    unsigned enable_significant_motion_function:1;
    unsigned reset_step_counter:1;
    unsigned enable_embedded_functions_and_filters:1;
    unsigned gyro_enable_x_axis:1;
    unsigned gyro_enable_y_axis:1;
    unsigned gyro_enable_z_axis:1;
};

const unsigned char CTRL_BLOCK = 0x10;

typedef struct __attribute__((__packed__)) {
    //byte 0
    struct accel_data_rate_ctrl accel_data_rate_ctrl;

    //byte 1 -- CTRL2_G
    struct gyro_data_rate_ctrl gyro_data_rate_ctrl;

    //byte 2 -- CTRL3_C
    struct comms_ctrl comms_ctrl;

    //byte 3 -- CTRL4_C
    struct ctrl_misc ctrl_misc;

    //byte 4 -- CTRL5_C
    struct rounding_and_self_test rounding_and_self_test;

    //byte 5 -- CTR6_C
    struct hp_and_triggers hp_and_triggers;

    //byte 6 -- CTR7_G
    struct gyro_filters gyro_filters;

    //byte 7 -- CTR8_XL
    struct accel_filters accel_filters;

    //byte 8 -- CTRL9_XL
    struct accel_enable accel_enable;

    //byte 9 -- CTRL10_C
    struct gyro_enable gyro_enable;
} gyro_ctrl_block;

// the weird values/order is what the doc says.
enum accel_scale {
    PLUS_MINUS_2 = 0b00,
    PLUS_MINUS_16 = 0b01,
    PLUS_MINUS_4 = 0b10,
    PLUS_MINUS_8 = 0b11
};

const unsigned char all_zero_arr[sizeof(gyro_ctrl_block)] = {0, 0, 0, 0, 0,
                                                    0, 0, 0, 0, 0};

diff_location gyro_set_ctrl_block() {
    gyro_ctrl_block readback;
    int all_zero, reserved_all_zero;

    memset((unsigned char*)&readback, 0x55, sizeof(readback));

    gyro_spi(CTRL_BLOCK | GYRO_READ_MASK,
             sizeof(gyro_ctrl_block),
             all_zero_arr,
             (unsigned char *)&readback);

    all_zero = 0 == memcmp(all_zero_arr,
                           (const char *)&readback,
                           sizeof(gyro_ctrl_block));
    reserved_all_zero =
//        (0 == readback.accel_enable.reserved0) &&
//        (0 == readback.accel_enable.reserved1) &&
        (0 == readback.accel_filters.reserved0) &&
        (0 == readback.accel_filters.reserved1) &&
        (0 == readback.ctrl_misc.reserved) &&
        (0 == readback.gyro_data_rate_ctrl.reserved) &&
        (0 == readback.gyro_filters.reserved) &&
        (0 == readback.hp_and_triggers.reserved) &&
        (0 == readback.rounding_and_self_test.reserved);



    // We got valid data back.
    if (!all_zero && reserved_all_zero) {

        // CTRL1_XL
        readback.accel_data_rate_ctrl.accel_data_rate = HZ_208;
        readback.accel_data_rate_ctrl.accel_scale = PLUS_MINUS_4;

        // CTRL2_G
        readback.gyro_data_rate_ctrl.gyro_data_rate = HZ_208;
        readback.gyro_data_rate_ctrl.gyro_scale = DPS_500;

        // CTRL3_C
        readback.comms_ctrl.block_data_update = 0;
        readback.comms_ctrl.enable_multi_byte_read = 1;
        // readback.comms_ctrl.push_pull_or_open_drain = 0 // PUSH_PULL; CSPAM: I don't know what this means. Ask Darren.

        // CTRL4_C
        readback.ctrl_misc.accel_enable_bandwidth_selection = 0;
        readback.ctrl_misc.rout_int2_signals_to_int1 = 1;
        readback.ctrl_misc.data_read_mask = 1;
        readback.ctrl_misc.disable_i2c = 1;
        readback.ctrl_misc.stop_on_fifo_threshold = 0;

        // CTRL5_C
        readback.rounding_and_self_test.include_accel = 1;
        readback.rounding_and_self_test.include_gyro = 1;
        readback.rounding_and_self_test.include_hub = 0;
        readback.rounding_and_self_test.accel_self_test = 0;
        readback.rounding_and_self_test.gyro_self_test = 0;

        // CTRL6_C
        readback.hp_and_triggers.accel_dsable_high_performance = 0; // Enable high performance.
        readback.hp_and_triggers.gyro_enable_edge_sensitive_trigger = 0;
        readback.hp_and_triggers.gyro_enable_level_sensitive_latch = 0;
        readback.hp_and_triggers.gyro_enable_level_sensitive_trigger = 0;

        // CTRL7_G
        readback.gyro_filters.disable_high_performance = 0; //Enable high performance.
        readback.gyro_filters.enable_highpass_filter = 1;

        // CTR8_XL
        readback.accel_filters.lowpass_selection = 1; // Use the low-pass filter for the XL output.
        readback.accel_filters.enable_orientation_lowpass_filter = 1; // Use low-pass filtered data for the orientation sensing.

        // CTR9_XL
        readback.accel_enable.accel_enable_x_axis = 1;
        readback.accel_enable.accel_enable_y_axis = 1;
        readback.accel_enable.accel_enable_z_axis = 1;
        readback.accel_enable.enable_soft_iron_correction = 1; //This might not do anything because we haven't set the MASTER_CONFIG

        // CTR10_C
        readback.gyro_enable.enable_embedded_functions_and_filters = 0;
        readback.gyro_enable.enable_significant_motion_function = 1;
        readback.gyro_enable.gyro_enable_x_axis = 1;
        readback.gyro_enable.gyro_enable_y_axis = 1;
        readback.gyro_enable.gyro_enable_z_axis = 1;

        return gyro_write_and_confirm(CTRL_BLOCK, sizeof(gyro_ctrl_block), (unsigned char *)&readback);
    } else {
        return OTHER_ERROR;
    }
}

int gyro_init () {
    volatile unsigned char result = 0xFF;
    volatile signed char count = 0, count2 = 0;
    volatile diff_location compare_result = OTHER_ERROR;

    // wait until we get proper communication over spi
    do {
        result = gyro_read_single_byte(GYRO_WHO_AM_I);
        count++;
        if (count < 0) {
            count = 0;
            count2++;
        }
    } while (GYRO_WHO_AM_I_EXPECTED != result);

    count = 0;
    count2 = 0;
    // set up fifo
    do {
        compare_result = gyro_set_fifo_to_default();
        count++;
        if (count < 0) {
            count = 0;
            count2++;
        }
    } while(compare_result.whichByte != 0);

    count = 0;
    count2 = 0;
    // set up Interrupt control
    do {
        compare_result = gyro_set_interupt_ctrl(INT1_FTH | INT1_DRDY_G | INT1_DRDY_XL, 0x0);
        count++;
        if (count < 0) {
            count = 0;
            count2++;
        }
    } while(compare_result.whichByte != 0);

    count = 0;
    count2 = 0;
    // set main ctrl block
    do {
        compare_result = gyro_set_ctrl_block();
        count++;
        if (count < 0) {
            count = 0;
            count2++;
        }
    } while (compare_result.whichByte != 0 && count2 > 0);

    return 0;
}


void main(void) {
    volatile unsigned char Result = 0x55;
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
    gyro_init();

    while (1) {
        Result = gyro_read_single_byte(GYRO_WHO_AM_I);
        if (0x69 != Result) continue;
        Result = Result + 1;
//        MtrRb =  SPI_transferLong(0,0);
//        if (SUCCESS != MtrRb ) continue;
//        //Start conversion
//        ADC10CTL0 |= ENC + ADC10SC;//enable and start
//        __delay_cycles(1000000);
//        while(ADC10CTL1 & ADC10BUSY);
//        TempRb = ADC10MEM;
    }
}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
    //INterrupt on int flag from gyro
    //read gyro values
    //start adc conversion

}
