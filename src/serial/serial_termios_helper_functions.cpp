/**
 * Author: Peter Smith (P3TE)
 * Date: 2023-07-21
*/

#include "serial_termios_helper_functions.hpp"

#include <fcntl.h>


BaudRate BaudRateIntToSpeed(int baud) {

    BaudRate result;
    result.humanReadableBaud = baud;
    result.valid = true;

    switch (baud)
    {
    case 0: result.termiosBaud = B0; break; //Hang Up
    case 50: result.termiosBaud = B50; break;
    case 75: result.termiosBaud = B75; break;
    case 110: result.termiosBaud = B110; break;
    case 134: result.termiosBaud = B134; break;
    case 150: result.termiosBaud = B150; break;
    case 200: result.termiosBaud = B200; break;
    case 300: result.termiosBaud = B300; break;
    case 600: result.termiosBaud = B600; break;
    case 1200: result.termiosBaud = B1200; break;
    case 1800: result.termiosBaud = B1800; break;
    case 2400: result.termiosBaud = B2400; break;
    case 4800: result.termiosBaud = B4800; break;
    case 9600: result.termiosBaud = B9600; break;
    case 19200: result.termiosBaud = B19200; break;
    case 38400: result.termiosBaud = B38400; break;
    // Extra output baud rates (not in POSIX).
    case 57600: result.termiosBaud = B57600; break;
    case 115200: result.termiosBaud = B115200; break;
    case 230400: result.termiosBaud = B230400; break;
    case 460800: result.termiosBaud = B460800; break;
    case 500000: result.termiosBaud = B500000; break;
    case 576000: result.termiosBaud = B576000; break;
    case 921600: result.termiosBaud = B921600; break;
    case 1000000: result.termiosBaud = B1000000; break;
    case 1152000: result.termiosBaud = B1152000; break;
    case 1500000: result.termiosBaud = B1500000; break;
    case 2000000: result.termiosBaud = B2000000; break;
    case 2500000: result.termiosBaud = B2500000; break;
    case 3000000: result.termiosBaud = B3000000; break;
    case 3500000: result.termiosBaud = B3500000; break;
    case 4000000: result.termiosBaud = B4000000; break;
    default: 
        result.termiosBaud = B0; 
        result.valid = false; 
        break;
    };

    return result;
}


int SetSerialDeviceToRawMode(int fd, BaudRate baud) {
    struct termios options;

    if (!baud.valid) {
        // Invalid baud rate.
        return TERMIOS_ERROR_INVALID_BAUD_RATE;
    }

    fcntl( fd, F_SETFL, 0 );
    tcgetattr( fd, &options );

    // https://man7.org/linux/man-pages/man3/termios.3.html
    // Raw mode
    // cfmakeraw() sets the terminal to something like the "raw" mode of
    // the old Version 7 terminal driver: input is available character
    // by character, echoing is disabled, and all special processing of
    // terminal input and output characters is disabled.  
    
    // The terminal attributes are set as follows:

    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                        | INLCR | IGNCR | ICRNL | IXON);
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    options.c_cflag &= ~(CSIZE | PARENB);
    options.c_cflag |= CS8;

    // http://unixwiz.net/techtips/termios-vmin-vtime.html
    // Return from read() after any number of bytes (VMIN = 0)
    // Wait for up to 5 tenths of a second without returning anything (VTIME)
    // This does mean after 5 tenths of a second with nothing, it will return from read with 0 bytes
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 5;

    cfsetispeed( &options, baud.termiosBaud );
    cfsetospeed( &options, baud.termiosBaud );

    if ( tcsetattr( fd, TCSANOW, &options ) < 0 ) {
        return TERMIOS_ERROR_FAILED_TO_SET_DEVICE_OPTION;
    }

    // Flush the serial port input and output buffers
    tcflush( fd, TCIOFLUSH );
    return 0;
}