/*
 * Global configuration for RTC and Temperature source
 *
 * HouYu Li <karajan_ii@hotmail.com>
 *
 * No license applied. Use as you wish.
 */

#ifndef CONFIG_H_
#define CONFIG_H_

/**
 * Own I2C slave address
 */
#define _I2C_addr        0x41
#define _I2C_addr_op1    0x43

/**
 * Day mask bit for alarm setting
 */
#define MON        BIT0
#define TUE        BIT1
#define WED        BIT2
#define THU        BIT3
#define FRI        BIT4
#define SAT        BIT5
#define SUN        BIT6

#endif /* CONFIG_H_ */
