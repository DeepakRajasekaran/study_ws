#ifndef TCP_SERVER_HPP
#define TCP_SERVER_HPP

#include <netinet/in.h>
#include <string>

#define PORT 9090

class TcpServer {
public:
    TcpServer(int port);
    ~TcpServer();

    void start();

private:
    int create_socket();
    void set_socket_options();
    void bind_socket();
    void handle_client(int client_socket);

    int server_fd;
    int port;
    struct sockaddr_in address;
    static const int BUFFER_SIZE = 1024;
};

#endif // TCP_SERVER_HPP