#ifndef ETHERNETUDP_H
#define ETHERNETUDP_H
#include <cstring>
#include <string>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <array>
#include <unistd.h>
#include <iostream>
using namespace std;
#define MAXLINE 1024
#define PORT 5555
int sockfd;
struct sockaddr_in serv_addr;
class Device {
public:
    struct sockaddr_in address{}, client{};
    std::array<double, 4> en{}, enprev{};
    int receive_some{}, sonic{};
    std::string data, meg;
    std::array<char, MAXLINE> buffer{};
    std::array<char, MAXLINE> terima{};
    socklen_t clasLen = sizeof(client);
    std::string addIP, deviceIP;

    Device(const std::string& IP, unsigned long port) 
        : deviceIP(IP) 
    {
        address.sin_family = AF_INET;
        if (inet_pton(AF_INET, IP.c_str(), &address.sin_addr.s_addr) <= 0) {
            std::cerr << "Invalid address/ Address not supported" << std::endl;
        }
        address.sin_port = htons(port);
    }

    void kirimData(int socket, const std::string& msg) {
        ssize_t sent_bytes = sendto(socket, msg.c_str(), msg.size(), MSG_WAITALL,
                                    reinterpret_cast<struct sockaddr*>(&address), sizeof(address));
        if (sent_bytes < 0) {
            std::cerr << "Failed to send data" << std::endl;
        }
    }

    void terimaData(int socket) {
        receive_some = recvfrom(socket, buffer.data(), buffer.size(), 0,
                                reinterpret_cast<struct sockaddr*>(&client), &clasLen);
        if (receive_some < 0) {
            std::cerr << "Failed to receive data" << std::endl;
            return;
        }

        buffer[receive_some] = '\0';
        addIP = inet_ntoa(client.sin_addr);
        strncpy(terima.data(), buffer.data(), terima.size() - 1);
        terima[terima.size() - 1] = '\0';  // Ensure null-termination

        if (addIP == "192.168.0.70") {
            meg = terima.data();
        } else {
            updateEncoderValue();
        }
    }

private:
    void updateEncoderValue() {
        if (addIP == "192.168.0.67") {
            en[0] = atof(terima.data());
        } else if (addIP == "192.168.0.66") {
            en[1] = atof(terima.data());
        } else if (addIP == "192.168.0.69") {
            en[2] = atof(terima.data());
        } else if (addIP == "192.168.0.68") {
            en[3] = atof(terima.data());
        }
    }
};
void startEthernet(){
// buatkan socket untuk deskripsi file
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }
    memset(&serv_addr, 0, sizeof(serv_addr));
    // info server semacam addressnya
    serv_addr.sin_family = AF_INET; //IPv4
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(PORT);
    //ikat socket dengan servernya
    if ( bind(sockfd, (const struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0 ){
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
}
#endif
