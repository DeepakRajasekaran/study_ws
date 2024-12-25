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
    void writeData(const unsigned char* data, size_t size);

private:
    int serial_port;
    int baud_rate;
    std::string port;
    struct termios tty;
};

#endif // RS485_H