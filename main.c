/**
 * Ultra Low Power RTC using MSP430G2452
 * It's based on G2452_RTC_TEMP_SRC,
 * and without temperature convert and CPU speed selection,
 * and no UART output
 * It's just a pure RTC with I2C interface.
 *
 * HouYu Li <karajan_ii@hotmail.com>
 *
 * No license applied. Use as you wish.
 *
 * Data structure is following DS3231.
 *
 * Port definition
 *      P1.0            1-Hz output
 *      P1.1, P1.2      Not used (pull down to GND)
 *      P1.3            I2C slave address pin
 *                      High:   0x41 (default)
 *                      Low:    0x43 (= 0x41 | 0x02)
 *      P1.4            Not used (pull down to GND)
 *      P1.5            Unison alarm interrupt output for all 6 alarms
 *      P1.6, P1.7      USI I2C mode (with pull-up res enabled)
 *      P2.0            Individual alarm interrupt output for Alarm1
 *      P2.1            Individual alarm interrupt output for Alarm2
 *      P2.2            Individual alarm interrupt output for Alarm3
 *      P2.3            Individual alarm interrupt output for Alarm4
 *      P2.4            Individual alarm interrupt output for Alarm5
 *      P2.5            Individual alarm interrupt output for Alarm6
 */

#include <msp430.h>

#include "config.h"
#include "functions.h"
#include "USI_I2C_slave.h"

unsigned char _DATA_STORE[31];  // Data storage
                                // 0: RTC second in BCD
                                // 1: RTC minute in BCD
                                // 2: RTC hour in BCD 24-hour format
                                // 3: RTC day in BCD. 1~7: Mon~Sun
                                // 4: RTC date in BCD
                                // 5: RTC month in BCD
                                // 6: RTC year in BCD
                                // 7: RTC century in BCD
                                // 8~10: Alarm1: minute(BCD), hour(BCD), day(s)(Bit Mask)
                                    // MSB of byte 9 is the match enable bit
                                // 11~25: Same as 8~10 for Alarm2~Alarm6
                                // 26: Not used
                                // 27: Not used
                                // 28: Reserved for general configuration
                                    // BIT7: Dedicated interrupt output for Alarm1~6
                                // 29: Alarm interrupt enable bits
                                // 30: Alarm interrupt flags

const unsigned int _second_div = 2048;      // 1/16 of 1-Hz with a bit tuning
unsigned int _second_tick = 0;              // Ticker for a second
unsigned char _is_leap_year = 0;            // Leap year indicator

unsigned char _I2C_data_offset = 0;         // Offset for data accessing in I2C

unsigned char _RTC_action_bits = 0x00;      // For marking actions in interrupt
                                            // and run the action in the main loop
unsigned char _RTC_byte_l = 0, _RTC_byte_h = 0; // For calculation use

/***********************************************
 * Callback related variables (Mandatory)
 * Do not change the variable name
 ***********************************************/
unsigned char _USI_I2C_slave_n_byte = 0;
//**********************************************/

/*
 * main.c
 */
void main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer

    // Set MCLK and SMCLK
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;

    // Setup P1 pin directions
    P1DIR |= (BIT0 + BIT5);             // P1.0 for 1-Hz output
                                        // P1.5 as unison alarm interrupt output pin
    P1OUT &= ~(BIT0 + BIT5);            // P1.0, P1.5 are low initially
    // Enable pull resistors for other pins
    P1REN |= (BIT1 + BIT2 + BIT3 + BIT4 + BIT6 + BIT7);
    P1OUT |= (BIT3 + BIT6 + BIT7);      // Using pull-up resistor on P1.3, P1.6, P1.7
    P1OUT &= ~(BIT1 + BIT2 + BIT4);     // Using pull-down resistor on P1.1, P1.2, P1.4

    // Set P2.0~P2.5 as dedicated output for alarm1~6
    P2DIR |= (BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5);
    // Default low
    P2OUT &= ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5);

    // Configure for ACLK, no division applied
    BCSCTL3 |= XCAP_3;          // BCSCTL3 |= 0x0C;
                                // XCAPx = 11, Oscillator capacitor ~12.5 pF

    // Setup Timer
    TACTL |= (TASSEL_1 + MC_2); // TASSELx = 01, using ACLK as source
                                // MCx = 02, continuous mode

    TACCR0 = _second_div;       // The timer clock is 32768-Hz
                                // _second_div is 32768-Hz / 16
                                // so that we have enough space for doing different actions
    TACCTL0 |= CCIE;            // Enable timer capture interrupt

    // Initialize data store values
    _init_DS();
    // Check leap year with initial data
    _check_leap_year();

    // Start I2C slave
    if (P1IN & BIT3)
        USI_I2C_slave_init(_I2C_addr);
    else
        USI_I2C_slave_init(_I2C_addr_op1);

    __enable_interrupt();

    while(1) {
        if (_RTC_action_bits & BIT0) {  // The main timer increment
            _time_increment();
            _RTC_action_bits &= ~BIT0;
        }
        if (_RTC_action_bits & BIT3) {  // Check alarm logic
            _check_alarms();
            _RTC_action_bits &= ~BIT3;
        }
        if (_RTC_action_bits & BIT4) {  // Check alarm interrupt
            _alarm_interrupt();
            _RTC_action_bits &= ~BIT4;
        }
        if (_RTC_action_bits & BIT5) {  // Reset alarm interrupt output
            _alarm_reset_interrupt();
            _RTC_action_bits &= ~BIT5;
        }
    }
}

/**
 * Extra functions
 */

/**
 * Initialize data store values
 */
void _init_DS() {
    // The default RTC time is 2000-1-1 00:00:00, Saturday
    _DATA_STORE[3] = 0x06;  // Day = 6, Saturday
    _DATA_STORE[4] = 0x01;  // Date = 1
    _DATA_STORE[5] = 0x01;  // Month = 1
    _DATA_STORE[7] = 0x20;  // Century = 20
}

/**
 * Check whether current year is leap year
 */
void _check_leap_year() {
    // Reset leap year indicator first
    _is_leap_year = 0;

    _RTC_byte_l = _DATA_STORE[6] << 4;
    if (_DATA_STORE[6] & 0x10) {
        if (_RTC_byte_l == 0x20 || _RTC_byte_l == 0x60)
            _is_leap_year = 1;
    } else {
        if (_RTC_byte_l == 0x40 || _RTC_byte_l == 0x80)
            _is_leap_year = 1;
    }
}

/**
 * Do time increment
 */
void _time_increment() {
    // Do the main time increment logic here
    _DATA_STORE[0]++;
    if (_DATA_STORE[0] == 0x5A) {   // Check second
        _DATA_STORE[0] = 0x00;
        _DATA_STORE[1]++;   // Add 1 minute
        _RTC_action_bits |= BIT3;   // Let's check alarms when second becomes 0
    } else {
        _time_carry(_DATA_STORE);
    }

    if (_DATA_STORE[1] == 0x5A) {   // Check minute, same logic with minute
        _DATA_STORE[1] = 0x00;
        _DATA_STORE[2]++;   // Add 1 hour
    } else {
        _time_carry(_DATA_STORE + 1);
    }

    if (_DATA_STORE[2] == 0x24) {   // Check hour
        _DATA_STORE[2] = 0x00;
        _DATA_STORE[3]++;   // Add 1 day
        _DATA_STORE[4]++;   // Add 1 date
    } else {
        _time_carry(_DATA_STORE + 2);
    }

    if (_DATA_STORE[3] == 0x08)     // Check day
        _DATA_STORE[3] = 0x01;

    switch (_DATA_STORE[4]) {       // Check date
    case 0x29:
        if (_DATA_STORE[5] == 0x02 && !_is_leap_year) {   // It's February
            _DATA_STORE[4] = 0x01;
            _DATA_STORE[5]++;
        }
        break;
    case 0x30:
        if (_DATA_STORE[5] == 0x02) {   // It's February
            _DATA_STORE[4] = 0x01;
            _DATA_STORE[5]++;
        }
        break;
    case 0x31:
        if (_DATA_STORE[5] == 0x04
                || _DATA_STORE[5] == 0x06
                || _DATA_STORE[5] == 0x09
                || _DATA_STORE[5] == 0x11) {    // Apr, Jun, Sep, Nov with 30 days
            _DATA_STORE[4] = 0x01;
            _DATA_STORE[5]++;
        }
        break;
    case 0x32:
        if (_DATA_STORE[5] == 0x01
                || _DATA_STORE[5] == 0x03
                || _DATA_STORE[5] == 0x05
                || _DATA_STORE[5] == 0x07
                || _DATA_STORE[5] == 0x08
                || _DATA_STORE[5] == 0x10
                || _DATA_STORE[5] == 0x12) {    // Jan, Mar, May, Jul, Aug, Oct, Dec with 31 days
            _DATA_STORE[4] = 0x01;
            _DATA_STORE[5]++;
        }
        break;
    default:
        _time_carry(_DATA_STORE + 4);
    }

    if (_DATA_STORE[5] == 0x13) {   // Check month
        _DATA_STORE[5] = 0x01;
        _DATA_STORE[6]++;   // Add 1 year
        _RTC_action_bits |= BIT2;   // Let's check the leap year later
    } else {
        _time_carry(_DATA_STORE + 5);
    }

    if (_DATA_STORE[6] == 0x9A) {   // Check year
        _DATA_STORE[6] = 0x00;
        _DATA_STORE[7]++;   // Add 1 century
    } else {
        _time_carry(_DATA_STORE + 6);
    }
    // After changes with the year
    if (_RTC_action_bits & BIT2) {
        // let's check the leap year property
        _check_leap_year();
        _RTC_action_bits &= ~BIT2;
    }

    if (_DATA_STORE[7] == 0x9A) {   // Check century
        _DATA_STORE[7] = 0x00;  // Century start over
    } else {
        _time_carry(_DATA_STORE + 7);
    }
}

/**
 * Deal with carry
 */
void _time_carry(unsigned char * byte) {
    _RTC_byte_l = *byte << 4;
    if (_RTC_byte_l == 0xA0) {
        _RTC_byte_h = *byte >> 4;
        _RTC_byte_h++;
        *byte = _RTC_byte_h << 4;
    }
}

/**
 * Alarm logic here
 */
void _check_alarms() {
    unsigned char day_mask_bit = 0x00;

    switch (_DATA_STORE[3]) {
    case 0x01:
        day_mask_bit = MON;
        break;
    case 0x02:
        day_mask_bit = TUE;
        break;
    case 0x03:
        day_mask_bit = WED;
        break;
    case 0x04:
        day_mask_bit = THU;
        break;
    case 0x05:
        day_mask_bit = FRI;
        break;
    case 0x06:
        day_mask_bit = SAT;
        break;
    case 0x07:
        day_mask_bit = SUN;
        break;
    }

    // Alarm 1
    if (_DATA_STORE[1] == _DATA_STORE[8] &&
            _DATA_STORE[2] + 0x80 == _DATA_STORE[9] &&
            (_DATA_STORE[10] & 0x80 ||
                    _DATA_STORE[10] & day_mask_bit))
        _DATA_STORE[30] |= BIT0;
    // Alarm 2
    if (_DATA_STORE[1] == _DATA_STORE[11] &&
            _DATA_STORE[2] + 0x80 == _DATA_STORE[12] &&
            (_DATA_STORE[13] & 0x80 ||
                    _DATA_STORE[13] & day_mask_bit))
        _DATA_STORE[30] |= BIT1;
    // Alarm 3
    if (_DATA_STORE[1] == _DATA_STORE[14] &&
            _DATA_STORE[2] + 0x80 == _DATA_STORE[15] &&
            (_DATA_STORE[16] & 0x80 ||
                    _DATA_STORE[16] & day_mask_bit))
        _DATA_STORE[30] |= BIT2;
    // Alarm 4
    if (_DATA_STORE[1] == _DATA_STORE[17] &&
            _DATA_STORE[2] + 0x80 == _DATA_STORE[18] &&
            (_DATA_STORE[19] & 0x80 ||
                    _DATA_STORE[19] & day_mask_bit))
        _DATA_STORE[30] |= BIT3;
    // Alarm 5
    if (_DATA_STORE[1] == _DATA_STORE[20] &&
            _DATA_STORE[2] + 0x80 == _DATA_STORE[21] &&
            (_DATA_STORE[22] & 0x80 ||
                    _DATA_STORE[22] & day_mask_bit))
        _DATA_STORE[30] |= BIT4;
    // Alarm 6
    if (_DATA_STORE[1] == _DATA_STORE[23] &&
            _DATA_STORE[2] + 0x80 == _DATA_STORE[24] &&
            (_DATA_STORE[25] & 0x80 ||
                    _DATA_STORE[25] & day_mask_bit))
        _DATA_STORE[30] |= BIT5;
}

/**
 * Check alarm interrupt flag and output interrupt
 */
void _alarm_interrupt() {
    unsigned char INT_uni = 0;
    unsigned char INT_A1 = 0, INT_A2 = 0, INT_A3 = 0, INT_A4 = 0, INT_A5 = 0, INT_A6 = 0;

    // Alarm 1
    if (_DATA_STORE[30] & BIT0 &&
            _DATA_STORE[29] & BIT0) {
        INT_uni = 1;
        if (_DATA_STORE[28] & 0x80)
            INT_A1 = 1;
    }
    // Alarm 2
    if (_DATA_STORE[30] & BIT1 &&
            _DATA_STORE[29] & BIT1) {
        INT_uni = 1;
        if (_DATA_STORE[28] & 0x80)
            INT_A2 = 1;
    }
    // Alarm 3
    if (_DATA_STORE[30] & BIT2 &&
            _DATA_STORE[29] & BIT2) {
        INT_uni = 1;
        if (_DATA_STORE[28] & 0x80)
            INT_A3 = 1;
    }
    // Alarm 4
    if (_DATA_STORE[30] & BIT3 &&
            _DATA_STORE[29] & BIT3) {
        INT_uni = 1;
        if (_DATA_STORE[28] & 0x80)
            INT_A4 = 1;
    }
    // Alarm 5
    if (_DATA_STORE[30] & BIT4 &&
            _DATA_STORE[29] & BIT4) {
        INT_uni = 1;
        if (_DATA_STORE[28] & 0x80)
            INT_A5 = 1;
    }
    // Alarm 6
    if (_DATA_STORE[30] & BIT5 &&
            _DATA_STORE[29] & BIT5) {
        INT_uni = 1;
        if (_DATA_STORE[28] & 0x80)
            INT_A6 = 1;
    }

    // Set the interrupt output pin to high
    if (INT_uni)
        P1OUT |= BIT5;
    if (INT_A1)
        P2OUT |= BIT0;
    if (INT_A2)
        P2OUT |= BIT1;
    if (INT_A3)
        P2OUT |= BIT2;
    if (INT_A4)
        P2OUT |= BIT3;
    if (INT_A5)
        P2OUT |= BIT4;
    if (INT_A6)
        P2OUT |= BIT5;
}

/**
 * Reset alarm interrupt output pin to low
 */
void _alarm_reset_interrupt() {
    P1OUT &= ~BIT5;
    P2OUT &= ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5);
}

/***********************************************
 * Mandatory functions for callback
 * You can modify codes in these functions
 *         to deal with data received
 *         or data to be sent
 *         but left function name unchanged
 ***********************************************/
unsigned char * USI_I2C_slave_TX_callback() {
    unsigned char _I2C_data_offset_1;
    _I2C_data_offset_1 = _I2C_data_offset;
    _I2C_data_offset++;
    return _DATA_STORE + _I2C_data_offset_1;
}

unsigned char USI_I2C_slave_RX_callback(unsigned char * byte) {
    unsigned char byte_data;
    byte_data = *byte;
    if (!_USI_I2C_slave_n_byte) {
        _I2C_data_offset = byte_data;
        _USI_I2C_slave_n_byte = 1;
    } else {
        if (_I2C_data_offset != 26 &&
                _I2C_data_offset != 27) {
            switch(_I2C_data_offset) {
            case 30:    // Do not allow 1 for alarm interrupt flags when flags are 0
                if (!(_DATA_STORE[30] & BIT0) &&
                        (byte_data & BIT0))
                    byte_data &= ~BIT0;
                if (!(_DATA_STORE[30] & BIT1) &&
                        (byte_data & BIT1))
                    byte_data &= ~BIT1;
                if (!(_DATA_STORE[30] & BIT2) &&
                        (byte_data & BIT2))
                    byte_data &= ~BIT2;
                if (!(_DATA_STORE[30] & BIT3) &&
                        (byte_data & BIT3))
                    byte_data &= ~BIT3;
                if (!(_DATA_STORE[30] & BIT4) &&
                        (byte_data & BIT4))
                    byte_data &= ~BIT4;
                if (!(_DATA_STORE[30] & BIT5) &&
                        (byte_data & BIT5))
                    byte_data &= ~BIT5;
                _DATA_STORE[30] = byte_data;
                break;
            default:
                *(_DATA_STORE + _I2C_data_offset) = byte_data;
            }
        }
        _I2C_data_offset++;
    }
    return 0;   // 0: No error; Not 0: Error in received data
}

void _USI_I2C_slave_reset_byte_count() {
    _USI_I2C_slave_n_byte = 0;
}
//**********************************************/

/**
 * The timer capture interrupt is set to happen every 0.5s
 */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0(void) {
    TACCR0 += _second_div;

    _second_tick++; // Increment the ticker
    switch (_second_tick) {
    case 2:
        _RTC_action_bits |= BIT4;   // Let's check alarm interrupt flag and output interrupt
        break;
    case 4:
        break;
    case 6:
        _RTC_action_bits |= BIT5;   // Reset alarm interrupt output after 4 division period
        break;
    case 8:
        // Toggle P1.0 output level every 0.5s
        // to form a full 1-Hz square wave output
        P1OUT ^= BIT0;
        break;
    case 10:
        break;
    case 12:
        _RTC_action_bits |= BIT0;   // Let's do time increment now
        break;
    case 14:
        break;
    case 16:
        // Toggle P1.0 output level every 0.5s
        // to form a full 1-Hz square wave output
        P1OUT ^= BIT0;
        _second_tick = 0;   // Reset ticker
    }
}
