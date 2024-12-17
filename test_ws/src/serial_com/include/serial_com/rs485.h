#ifndef RS485_H
#define RS485_H

#include <iostream>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

class SerialPort {
public:
    SerialPort(const char* port, int baud_rate);
    ~SerialPort();

private:
    int serial_port;
};

#endif // RS485_H