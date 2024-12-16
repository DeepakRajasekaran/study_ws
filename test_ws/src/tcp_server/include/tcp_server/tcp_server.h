#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include <netinet/in.h>

// Constants
#define PORT 9090
#define BUFFER_SIZE 1024

// Function declarations
int create_socket();
void set_socket_options(int server_fd);
void bind_socket(int server_fd, struct sockaddr_in address);
void start_server(int server_fd);

#endif // TCP_SERVER_H