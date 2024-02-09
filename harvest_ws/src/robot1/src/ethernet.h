#include <bits/stdc++.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h>

#define PORT 5555
#define MAXLINE 1024
char buffer[MAXLINE];
int sockfd,sizeReceive;
struct sockaddr_in serv_addr,cli_addr;
socklen_t len;
struct ps{
	char buffer[50];
  	int counter;
  	float heading;
}ps;
class Device {
	public:
		struct sockaddr_in address, client;
		double en1,en2,en3,en4, enprev1,enprev2,enprev3,enprev4;
		double en[4],enprev[4];
		int receive_some,sonic;
		std::string data;
		char buffer[MAXLINE];
		char terima[MAXLINE];
		socklen_t clasLen, clientLen;
		std::string addIP, deviceIP;

		Device(const char *IP, unsigned long colok) {
			memset(&address, 0, sizeof(address));
			memset(&client, 0, sizeof(client));
			deviceIP = IP;
			address.sin_family = AF_INET;
			inet_pton(AF_INET, IP, &address.sin_addr.s_addr);
			address.sin_port = htons(colok);  // Use the provided port parameter
		}
		void kirimData(int soket, std::string msg) {
			sendto(soket, msg.c_str(), strlen(msg.c_str()), MSG_WAITALL, (struct sockaddr *)&address, sizeof(address));
		}

		void terimaData(int socket) {
			receive_some = recvfrom(socket, (char *)buffer, MAXLINE,
									0, (struct sockaddr *)&client,
									&clasLen);
			buffer[receive_some] = '\0';
			addIP = inet_ntoa(client.sin_addr);
			strncpy(terima, buffer, MAXLINE);
			if (addIP == "192.168.0.70") {
				data = terima;
			}else if(addIP == "192.168.0.67"){
				en1 = atof(terima);
			}else if(addIP == "192.168.0.66"){
				en2 = atof(terima);
			}else if(addIP == "192.168.0.69"){
				en3 = atof(terima);
			}else if(addIP == "192.168.0.68"){
				en4 = atof(terima);
			}else if(addIP == "192.168.0.2"){
				sonic = atoi(terima);
			}
		}
};
float toRad(float degree) {
  return degree * M_PI / 180;
}

void mkIkat(){
		// buatkan socket untuk deskripsi file
	if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
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
