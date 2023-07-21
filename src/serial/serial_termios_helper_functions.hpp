/**
 * Author: Peter Smith (P3TE)
 * Date: 2023-07-21
*/

#ifndef DATABRIDGETOOLS_SERIAL_SERIALTERMIOSHELPERFUNCTIONS_HPP
#define DATABRIDGETOOLS_SERIAL_SERIALTERMIOSHELPERFUNCTIONS_HPP

#include <termios.h>

#define TERMIOS_ERROR_INVALID_BAUD_RATE 1
#define TERMIOS_ERROR_FAILED_TO_SET_DEVICE_OPTION 2

struct BaudRate {
    int humanReadableBaud;
    speed_t termiosBaud;
    bool valid = false;
};

/**
 * Takes a baud rate as a human readable number (E.G 9600)
 * And converts it to the bitmask code for that baud (B9600)
 * (B9600 equates to 0000015 which is 13 in base 10. Which is confusing so this function exists...)
 * returns -1 if it's not a valid BAUD rate.
 */
BaudRate BaudRateIntToSpeed(int humanReadableBaud);

/**
 * Initialise the serial port in raw mode (no modification of the data)
 * https://man7.org/linux/man-pages/man3/termios.3.html
 * Raw mode
 * cfmakeraw() sets the terminal to something like the "raw" mode of
 * the old Version 7 terminal driver: input is available character
 * by character, echoing is disabled, and all special processing of
 * terminal input and output characters is disabled.
 */
int SetSerialDeviceToRawMode(int fd, BaudRate baud);


#endif //DATABRIDGETOOLS_SERIAL_SERIALTERMIOSHELPERFUNCTIONS_HPP