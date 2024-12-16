#include "tcp_server.hpp"
#include <iostream>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

TcpServer::TcpServer(int port) : port(port) {
    server_fd = create_socket();
    set_socket_options();

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    bind_socket();
}

TcpServer::~TcpServer() {
    close(server_fd);
}

int TcpServer::create_socket() {
    int fd;
    if ((fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
    return fd;
}

void TcpServer::set_socket_options() {
    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        perror("setsockopt");
        close(server_fd);
        exit(EXIT_FAILURE);
    }
}

void TcpServer::bind_socket() {
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("bind failed");
        close(server_fd);
        exit(EXIT_FAILURE);
    }
}

void TcpServer::start() {
    if (listen(server_fd, 3) < 0) {
        perror("listen");
        close(server_fd);
        exit(EXIT_FAILURE);
    }

    int addrlen = sizeof(address);
    while (true) {
        int new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen);
        if (new_socket < 0) {
            perror("accept");
            close(server_fd);
            exit(EXIT_FAILURE);
        }

        handle_client(new_socket);
    }
}

void TcpServer::handle_client(int client_socket) {
    char buffer[BUFFER_SIZE] = {0};
    const char *feedback = "Data received";

    read(client_socket, buffer, BUFFER_SIZE);
    std::cout << "Received: " << buffer << std::endl;
    send(client_socket, feedback, strlen(feedback), 0);
    std::cout << "Feedback sent" << std::endl;
    close(client_socket);
}

int main() {
    TcpServer server(PORT);
    server.start();
    return 0;
}