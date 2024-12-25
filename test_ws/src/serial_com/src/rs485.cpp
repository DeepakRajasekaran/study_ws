#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include "rs485.h"

SerialPort::SerialPort(const char* port, int baud_rate) {
    // Open the serial port
    serial_port = open(port, O_RDWR);

    // Check for errors
    if (serial_port < 0) {
        std::cerr << "Error " << errno << " opening " << port << ": " << strerror(errno) << std::endl;
        exit(1);
    }

    // Configure the serial port
    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
        exit(1);
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    // Set in/out baud rate to be 115200
    cfsetispeed(&tty, baud_rate);
    cfsetospeed(&tty, baud_rate);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cerr << "Error " << errno << " from tcsetattr: " << strerror(errno) << std::endl;
        exit(1);
    }
}

SerialPort::~SerialPort() {
    close(serial_port);
}

void SerialPort::writeData(const unsigned char* data, size_t size) {
    write(serial_port, data, size);
}

int main() {
    SerialPort serial("/dev/ttyUSB0", B115200);

    // Prepare data to send
    unsigned char request1 = ~236;
    unsigned char request2 = ~200;

    while (true) {
        // Write data to the serial port
        serial.writeData(&request1, sizeof(request1));
        serial.writeData(&request2, sizeof(request2));
        usleep(100000); // Sleep for 100ms
    }

    return 0;
}