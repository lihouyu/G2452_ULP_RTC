/*
 * USI I2C slave library
 *
 * HouYu Li <karajan_ii@hotmail.com>
 *
 * No license applied. Use as you wish.
 */

#include <msp430.h>

#include "USI_I2C_slave.h"
#include "functions.h"

unsigned char _USI_I2C_slave_own_addr;
unsigned char _USI_I2C_slave_state = 0;
unsigned char _USI_I2C_slave_RX_buff;

void USI_I2C_slave_init(unsigned char USI_I2C_slave_OA) {
    _USI_I2C_slave_own_addr = USI_I2C_slave_OA; // Assign the slave own address to local variable
                                                // The address should be a 7 bit address

    USICTL0 = (USIPE6 + USIPE7 + USISWRST); // Enable I2C pin & soft reset for USI module
    USICTL1 = (USII2C + USISTTIE + USIIE);  // Set I2C mode and enable related interrupt
    USICKCTL = USICKPL;                     // Use proper clock polarity
    USICTL0 &= ~USISWRST;                   // Exit reset status

    USICTL1 &= ~USISTTIFG;                  // Clear previous interrupt flag
    USICTL1 &= ~USIIFG;

    __enable_interrupt();                   // Enable global interrupt
}

#pragma vector = USI_VECTOR
__interrupt void USI_INT(void) {
    if (USICTL1 & USISTTIFG) {              // Start condition detected
        USICTL1 &= ~USISTP;                 // Force clear stop bit
        USICNT &= ~0x1F;                    // Clear SR count
        USISRL = 0x00;                      // Clear SR
        USICTL1 &= ~USIIFG;                 // Clear interrupt flag again
        _USI_I2C_slave_state = 2;           // Start in state 2
        _USI_I2C_slave_reset_byte_count();  // Clear data transaction byte count
    }

    switch (_USI_I2C_slave_state) {
    case 0: // Do nothing
        break;
    case 2: // Start of I2C transaction
        USICTL0 &= ~USIOE;          // Disable output for receiving byte
        USICNT |= 0x08;             // Receive the 1st byte, slave address
        USICTL1 &= ~USISTTIFG;      // Clear start interrupt flag
        _USI_I2C_slave_state = 3;   // Go to check slave address (state 3)
        break;
    case 3: // Check received slave address
        if ((USISRL >> 1) != _USI_I2C_slave_own_addr) {     // Slave address does not match
            USISRL = 0xff;              // Generate NACK
            _USI_I2C_slave_state = 6;   // Release
        } else {
            if ((_USI_I2C_slave_own_addr << 1) == USISRL)   // Slave receiver
                _USI_I2C_slave_state = 11;
            else                                            // Slave transmitter
                _USI_I2C_slave_state = 12;
            USISRL = 0x00;              // Generate ACK
        }
        USICTL0 |= USIOE;   // Enable output
        USICNT |= 0x01;     // Send ACKNACK
        break;
    case 6: // Release and prepare for next start
        USICTL0 &= ~USIOE;          // Set SDA as input
        USICTL1 &= ~USIIFG;         // Clear interrupt flag
        _USI_I2C_slave_state = 0;   // Reset state
        break;
    case 11: // Receive data byte
        USICTL0 &= ~USIOE;          // Set SDA as input
        USICNT |= 0x08;             // Prepare for receiving 8 bits
        _USI_I2C_slave_state = 13;  // Go to check received data and send ACKNACK
        break;
    case 12: // Send 1st data byte
        USISRL = *(USI_I2C_slave_TX_callback());    // Get prepared data to be sent
        USICTL0 |= USIOE;                           // Set SDA as output
        USICNT |= 0x08;                             // Prepare for transmitting 8 bits
        _USI_I2C_slave_state = 14;                  // Go to receive ACKNACK
        break;
    case 13: // Check received data and send ACKNACK
        _USI_I2C_slave_RX_buff = USISRL;    // Copy byte from SR to local variable
        if (!USI_I2C_slave_RX_callback(&_USI_I2C_slave_RX_buff)) {  // No error in data, returns 0
            USISRL = 0x00;                  // Generate ACK
            _USI_I2C_slave_state = 11;      // Go on receiving data
        } else {
            USISRL = 0xff;                  // Generate NACK
            _USI_I2C_slave_state = 6;       // Do not continue the transaction
        }
        USICTL0 |= USIOE;                   // Set SDA as output
        USICNT |= 0x01;                     // Prepare for transmitting 1 bit
        break;
    case 14: // Receive ACKNACK
        USICTL0 &= ~USIOE;          // Set SDA as input
        USICNT |= 0x01;             // Prepare for receiving 1 bit
        _USI_I2C_slave_state = 15;  // Check received ACKNACK
        break;
    case 15: // Check received ACKNACK
        if (USISRL & 1) {               // NACK received, release and prepare for another start
            USICTL0 &= ~USIOE;          // Set SDA as input
            USICTL1 &= ~USIIFG;         // Clear interrupt flag
            _USI_I2C_slave_state = 0;   // Reset state
        } else {                        // ACK received, go on send data byte
            USISRL = *(USI_I2C_slave_TX_callback());    // Get prepared data to be sent
            USICTL0 |= USIOE;                           // Set SDA as output
            USICNT |= 0x08;                             // Prepare for transmitting 8 bits
            _USI_I2C_slave_state = 14;                  // Go to receive ACKNACK
        }
        break;
    }
}
