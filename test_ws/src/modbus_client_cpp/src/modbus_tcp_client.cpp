#include <iostream>
#include <modbus/modbus.h>

int main() {
    // Create a new Modbus TCP context
    modbus_t *ctx = modbus_new_tcp("192.168.0.1", 502);
    if (ctx == nullptr) {
        std::cerr << "Unable to create the libmodbus context" << std::endl;
        return -1;
    }

    // Connect to the PLC
    if (modbus_connect(ctx) == -1) {
        std::cerr << "Connection failed: " << modbus_strerror(errno) << std::endl;
        modbus_free(ctx);
        return -1;
    }

    // Read 10 holding registers starting from address 0
    uint16_t tab_reg[10];
    int rc = modbus_read_registers(ctx, 0, 10, tab_reg);
    if (rc == -1) {
        std::cerr << "Failed to read registers: " << modbus_strerror(errno) << std::endl;
        modbus_close(ctx);
        modbus_free(ctx);
        return -1;
    }

    // Print the values of the registers
    for (int i = 0; i < 10; i++) {
        std::cout << "Register " << i << ": " << tab_reg[i] << std::endl;
    }

    // Close the connection and free the Modbus context
    modbus_close(ctx);
    modbus_free(ctx);

    return 0;
}